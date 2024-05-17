#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>
#include <rmw/types.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <sensor_msgs/msg/battery_state.h>
#include <sensor_msgs/msg/range.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>
#include "config.h"
#include "syslog.h"
#include "motor.h"
#include "kinematics.h"
#include "PID_v1.h"
#include "odometry.h"
#include "imu.h"
#include "mag.h"
#define ENCODER_USE_INTERRUPTS
#define ENCODER_OPTIMIZE_INTERRUPTS
#include "encoder.h"
#include "battery.h"
#include "range.h"
#include "lidar.h"
#include "wifis.h"
#include "ota.h"
#include <std_msgs/msg/float32_multi_array.h>

rcl_publisher_t motor_speeds_publisher;
std_msgs__msg__Float32MultiArray motor_speeds_msg;

rcl_publisher_t setpoint_publisher;
std_msgs__msg__Float32MultiArray setpoint_msg;

#ifdef WDT_TIMEOUT
#include <esp_task_wdt.h>
#endif
#ifdef USE_WIFI_TRANSPORT
static inline void set_microros_net_transports(IPAddress agent_ip, uint16_t agent_port)
{
    static struct micro_ros_agent_locator locator;
    locator.address = agent_ip;
    locator.port = agent_port;
    rmw_uros_set_custom_transport(
        false,
        (void *) &locator,
        platformio_transport_open,
        platformio_transport_close,
        platformio_transport_write,
        platformio_transport_read
    );
}
#endif

#ifndef BAUDRATE
#define BAUDRATE 115200
#endif

#ifndef NODE_NAME
#define NODE_NAME "linorobot_base_node"
#endif
#ifndef TOPIC_PREFIX
#define TOPIC_PREFIX
#endif

#ifndef RCCHECK
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){rclErrorLoop(temp_rc);}}
#endif
#ifndef RCSOFTCHECK
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){syslog(LOG_ERR, "Soft Error: %s", rcl_get_error_string().str); rcl_reset_error();}}
#endif
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)

rcl_publisher_t odom_publisher;
rcl_publisher_t imu_publisher;
rcl_publisher_t mag_publisher;
rcl_subscription_t twist_subscriber;
rcl_publisher_t battery_publisher;
rcl_publisher_t range_publisher;

nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__MagneticField mag_msg;
geometry_msgs__msg__Twist twist_msg;
sensor_msgs__msg__BatteryState battery_msg;
sensor_msgs__msg__Range range_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;
rcl_timer_t battery_timer;
rcl_timer_t range_timer;

rmw_qos_profile_t reliable_qos = rmw_qos_profile_default;

double setPoint1 = 0.0, input1 = 0.0, output1 = 0.0;
double setPoint2 = 0.0, input2 = 0.0, output2 = 0.0;

PID motor1_pid(&input1, &output1, &setPoint1, K_P_M1, K_I_M1, K_D_M1, DIRECT);
PID motor2_pid(&input2, &output2, &setPoint2, K_P_M2, K_I_M2, K_D_M2, DIRECT);

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_update = 0;
unsigned long last_setpoint_time = 0;  // Add this line

enum states 
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

Encoder motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV1, MOTOR1_ENCODER_INV);
Encoder motor2_encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, COUNTS_PER_REV2, MOTOR2_ENCODER_INV);

Motor motor1_controller(PWM_FREQUENCY, PWM_BITS, MOTOR1_INV, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
Motor motor2_controller(PWM_FREQUENCY, PWM_BITS, MOTOR2_INV, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B);

Kinematics kinematics(
    Kinematics::LINO_BASE, 
    MOTOR_MAX_RPM, 
    MAX_RPM_RATIO, 
    MOTOR_OPERATING_VOLTAGE, 
    MOTOR_POWER_MAX_VOLTAGE, 
    WHEEL_DIAMETER, 
    LR_WHEELS_DISTANCE
);

Odometry odometry;
IMU imu;
MAG mag;

void setup() 
{
#ifdef BOARD_INIT
    BOARD_INIT;
#endif

    Serial.begin(BAUDRATE);
    pinMode(LED_PIN, OUTPUT);
#ifdef SDA_PIN
#ifdef ESP32
    Wire.begin(SDA_PIN, SCL_PIN);
#else
    Wire.setSDA(SDA_PIN);
    Wire.setSCL(SCL_PIN);
#endif
#endif
#ifdef WDT_TIMEOUT
    esp_task_wdt_init(WDT_TIMEOUT, true);
    esp_task_wdt_add(NULL);
#endif
    initWifis();
    initOta();

    bool imu_ok = imu.init();
    if(!imu_ok)
    {
        while(1)
        {
            flashLED(3);
        }
    }
    mag.init();
    initBattery();
    initRange();
    initLidar();

#ifdef USE_WIFI_TRANSPORT
    set_microros_net_transports(AGENT_IP, AGENT_PORT);
#else
    set_microros_serial_transports(Serial);
#endif

    syslog(LOG_INFO, "%s Ready %lu", __FUNCTION__, millis());
    
    motor1_pid.SetMode(AUTOMATIC);
    motor1_pid.SetOutputLimits(-255, 255);
    motor2_pid.SetMode(AUTOMATIC);
    motor2_pid.SetOutputLimits(-255, 255);

    reliable_qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    reliable_qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
    reliable_qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    reliable_qos.depth = 10;
}

void loop() {
    unsigned long current_time = millis();  // Add this line

    switch (state) 
    {
        case WAITING_AGENT:
            EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
            break;
        case AGENT_AVAILABLE:
            syslog(LOG_INFO, "%s agent available %lu", __FUNCTION__, millis());
            state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
            if (state == WAITING_AGENT) 
            {
                destroyEntities();
            }
            break;
        case AGENT_CONNECTED:
            EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
            if (state == AGENT_CONNECTED) 
            {
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
            }
            break;
        case AGENT_DISCONNECTED:
            syslog(LOG_INFO, "%s agent disconnected %lu", __FUNCTION__, millis());
            fullStop();
            destroyEntities();
            state = WAITING_AGENT;
            break;
        default:
            break;
    }

    runWifis();
    runOta();
#ifdef WDT_TIMEOUT
    esp_task_wdt_reset();
#endif
}

void controlCallback(rcl_timer_t * timer, int64_t last_call_time) 
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) 
    {
       moveBase();
       publishData();
    }
}

void batteryCallback(rcl_timer_t * timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        battery_msg = getBattery();
        struct timespec time_stamp = getTime();
        battery_msg.header.stamp.sec = time_stamp.tv_sec;
        battery_msg.header.stamp.nanosec = time_stamp.tv_nsec;
        RCSOFTCHECK(rcl_publish(&battery_publisher, &battery_msg, NULL));
    }
}

void rangeCallback(rcl_timer_t * timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        range_msg = getRange();
        struct timespec time_stamp = getTime();
        range_msg.header.stamp.sec = time_stamp.tv_sec;
        range_msg.header.stamp.nanosec = time_stamp.tv_nsec;
        RCSOFTCHECK(rcl_publish(&range_publisher, &range_msg, NULL));
    }
}

void twistCallback(const void * msgin) 
{
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    prev_cmd_time = millis();
    last_setpoint_time = millis();  // Update the last setpoint time
}

bool createEntities() 
{
    syslog(LOG_INFO, "%s %lu", __FUNCTION__, millis());
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));

    RCCHECK(rclc_publisher_init(
        &odom_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        TOPIC_PREFIX "odom/unfiltered",
        &reliable_qos
    ));

    RCCHECK(rclc_publisher_init(
        &imu_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        TOPIC_PREFIX "imu/data",
        &reliable_qos
    ));

    RCCHECK(rclc_publisher_init(
        &mag_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
        TOPIC_PREFIX "imu/mag",
        &reliable_qos
    ));

    RCCHECK(rclc_publisher_init(
        &battery_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
        TOPIC_PREFIX "battery",
        &reliable_qos
    ));

    RCCHECK(rclc_publisher_init(
        &range_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
        TOPIC_PREFIX "ultrasound",
        &reliable_qos
    ));

    RCCHECK(rclc_subscription_init(
        &twist_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        TOPIC_PREFIX "cmd_vel",
        &reliable_qos
    ));

    const unsigned int control_timeout = 20;
    RCCHECK(rclc_timer_init_default(
        &control_timer,
        &support,
        RCL_MS_TO_NS(control_timeout),
        controlCallback
    ));

    const unsigned int battery_timer_timeout = 2000;
    RCCHECK(rclc_timer_init_default(
        &battery_timer,
        &support,
        RCL_MS_TO_NS(battery_timer_timeout),
        batteryCallback
    ));

    const unsigned int range_timer_timeout = 100;
    RCCHECK(rclc_timer_init_default(
        &range_timer,
        &support,
        RCL_MS_TO_NS(range_timer_timeout),
        rangeCallback
    ));

    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &twist_subscriber,
        &twist_msg,
        &twistCallback,
        ON_NEW_DATA
    ));

    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));
    RCCHECK(rclc_executor_add_timer(&executor, &battery_timer));
    RCCHECK(rclc_executor_add_timer(&executor, &range_timer));

    syncTime();
    digitalWrite(LED_PIN, HIGH);

    RCCHECK(rclc_publisher_init_best_effort(
        &motor_speeds_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "motor_speeds"
    ));
    std_msgs__msg__Float32MultiArray__init(&motor_speeds_msg);
    motor_speeds_msg.data.capacity = 4;
    motor_speeds_msg.data.size = 4;
    motor_speeds_msg.data.data = (float*)malloc(motor_speeds_msg.data.capacity * sizeof(float));

    RCCHECK(rclc_publisher_init_best_effort(
        &setpoint_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "setpoints"
    ));
    std_msgs__msg__Float32MultiArray__init(&setpoint_msg);
    setpoint_msg.data.capacity = 8;
    setpoint_msg.data.size = 8;
    setpoint_msg.data.data = (float*)realloc(setpoint_msg.data.data, setpoint_msg.data.capacity * sizeof(float));

    return true;
}

bool destroyEntities()
{
    syslog(LOG_INFO, "%s %lu", __FUNCTION__, millis());
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_publisher_fini(&odom_publisher, &node);
    rcl_publisher_fini(&imu_publisher, &node);
    rcl_publisher_fini(&mag_publisher, &node);
    rcl_publisher_fini(&battery_publisher, &node);
    rcl_publisher_fini(&range_publisher, &node);
    rcl_subscription_fini(&twist_subscriber, &node);
    rcl_timer_fini(&control_timer);
    rcl_timer_fini(&battery_timer);
    rcl_timer_fini(&range_timer);
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);

    digitalWrite(LED_PIN, HIGH);
    
    rcl_publisher_fini(&motor_speeds_publisher, &node);
    std_msgs__msg__Float32MultiArray__fini(&motor_speeds_msg);
    free(motor_speeds_msg.data.data);

    rcl_publisher_fini(&setpoint_publisher, &node);
    std_msgs__msg__Float32MultiArray__fini(&setpoint_msg);
    free(setpoint_msg.data.data);
    
    return true;
}

void fullStop()
{
    twist_msg.linear.x = 0.0;
    twist_msg.linear.y = 0.0;
    twist_msg.angular.z = 0.0;

    motor1_controller.brake();
    motor2_controller.brake();
}

void moveBase()
{
    if(((millis() - prev_cmd_time) >= 200)) 
    {
        twist_msg.linear.x = 0.0;
        twist_msg.linear.y = 0.0;
        twist_msg.angular.z = 0.0;
        
        setPoint1 = 0;
        setPoint2 = 0;

        digitalWrite(LED_PIN, HIGH);
    }
    else
    {
        Kinematics::rpm req_rpm = kinematics.getRPM(
            twist_msg.linear.x, 
            twist_msg.linear.y, 
            twist_msg.angular.z
        );
    
        setPoint1 = req_rpm.motor1;
        setPoint2 = req_rpm.motor2;
    }

    float current_rpm1 = motor1_encoder.getRPM();
    float current_rpm2 = motor2_encoder.getRPM();

    if(twist_msg.linear.x == 0.0 && twist_msg.angular.z == 0.0){
        fullStop();
    }else{
        input1 = current_rpm1;
        input2 = current_rpm2;
        
        motor1_pid.Compute();
        motor2_pid.Compute();
        
        motor1_controller.spin(output1);
        motor2_controller.spin(output2);
    }

    Kinematics::velocities current_vel = kinematics.getVelocities(
        current_rpm1, 
        current_rpm2, 
        0, // Add default values for motor3 and motor4
        0  // Add default values for motor3 and motor4
    );

    publishSetpointsAndOutputs();

    motor_speeds_msg.data.data[0] = input1;
    motor_speeds_msg.data.data[1] = input2;

    RCSOFTCHECK(rcl_publish(&motor_speeds_publisher, &motor_speeds_msg, NULL));

    unsigned long now = millis();
    float vel_dt = (now - prev_odom_update) / 1000.0;
    prev_odom_update = now;
    odometry.update(
        vel_dt, 
        current_vel.linear_x, 
        current_vel.linear_y, 
        current_vel.angular_z
    );
}

void publishSetpointsAndOutputs() {
    setpoint_msg.data.data[0] = setPoint1;
    setpoint_msg.data.data[1] = setPoint2;

    setpoint_msg.data.data[2] = input1;
    setpoint_msg.data.data[3] = input2;

    RCSOFTCHECK(rcl_publish(&setpoint_publisher, &setpoint_msg, NULL));
}

void publishData()
{
    odom_msg = odometry.getData();
    imu_msg = imu.getData();
    mag_msg = mag.getData();
#ifdef MAG_BIAS
    const float mag_bias[3] = MAG_BIAS;
    mag_msg.magnetic_field.x -= mag_bias[0];
    mag_msg.magnetic_field.y -= mag_bias[1];
    mag_msg.magnetic_field.z -= mag_bias[2];
#endif
    double roll, pitch, yaw;
    roll = atan2(imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z);
    pitch = atan2(-imu_msg.linear_acceleration.y,
                (sqrt(imu_msg.linear_acceleration.y * imu_msg.linear_acceleration.y +
                      imu_msg.linear_acceleration.z * imu_msg.linear_acceleration.z)));
    yaw = atan2(mag_msg.magnetic_field.y, mag_msg.magnetic_field.x);
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    imu_msg.orientation.x = cy * cp * sr - sy * sp * cr;
    imu_msg.orientation.y = sy * cp * sr + cy * sp * cr;
    imu_msg.orientation.z = sy * cp * cr - cy * sp * sr;
    imu_msg.orientation.w = cy * cp * cr + sy * sp * sr;

    struct timespec time_stamp = getTime();

    odom_msg.header.stamp.sec = time_stamp.tv_sec;
    odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    imu_msg.header.stamp.sec = time_stamp.tv_sec;
    imu_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    mag_msg.header.stamp.sec = time_stamp.tv_sec;
    mag_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
    RCSOFTCHECK(rcl_publish(&mag_publisher, &mag_msg, NULL));
    RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
}

bool syncTime()
{
    const int timeout_ms = 1000;
    if (rmw_uros_epoch_synchronized()) return true;
    RCCHECK(rmw_uros_sync_session(timeout_ms));
    if (rmw_uros_epoch_synchronized()) {
#if (_POSIX_TIMERS > 0)
        int64_t time_ns = rmw_uros_epoch_nanos();
        timespec tp;
        tp.tv_sec = time_ns / 1000000000;
        tp.tv_nsec = time_ns % 1000000000;
        clock_settime(CLOCK_REALTIME, &tp);
#else
        unsigned long long ros_time_ms = rmw_uros_epoch_millis();
        time_offset = ros_time_ms - millis();
#endif
        return true;
    }
    return false;
}

struct timespec getTime()
{
    struct timespec tp = {0};
#if (_POSIX_TIMERS > 0)
    clock_gettime(CLOCK_REALTIME, &tp);
#else
    unsigned long long now = millis() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;
#endif
    return tp;
}

void rclErrorLoop(rcl_ret_t error_code) 
{
    syslog(LOG_ERR, "RCL Error: %s", rcl_get_error_string().str);
    rcl_reset_error();
    while(true)
    {
        flashLED(2);
        runOta();
    }
}

void flashLED(int n_times)
{
    for(int i=0; i<n_times; i++)
    {
        digitalWrite(LED_PIN, HIGH);
        delay(150);
        digitalWrite(LED_PIN, LOW);
        delay(150);
    }
    delay(1000);
}

