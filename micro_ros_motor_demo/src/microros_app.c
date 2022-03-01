#include <time.h>

#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/float32.h>

#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

#include <rmw_microros/rmw_microros.h>

#include "./utils.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){bool l = true; while(1){set_led_status(LED_3, l = !l); sleep_ms(100);}}}
#define MAX_RPM 1200.0
#define MIN_RPM 300.0

void microros_app(void);
void subscription_callback(const void * msgin);
void publish_joint();
void publish_vel();

rcl_subscription_t sub_commands;
rcl_publisher_t pub_vel;
rcl_publisher_t pub_joint;

std_msgs__msg__Float32 msg_vel_input;
sensor_msgs__msg__JointState msg_joint;

float motor_velocity = 0.0;

// Set motor speed to received values
void subscription_callback(const void * msgin)
{
    const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
    float command = msg->data;

    uint8_t motor_status;

    // Get current motor status
    g_motor_120_degree0.p_api->statusGet(g_motor_120_degree0.p_ctrl, &motor_status);

    if (motor_status == MOTOR_120_DEGREE_CTRL_STATUS_ERROR )
    {
        // Reset motor on error
        g_motor_120_degree0.p_api->reset(g_motor_120_degree0.p_ctrl);
    }

    if (fabs(command) < MIN_RPM && motor_status != MOTOR_120_DEGREE_CTRL_STATUS_STOP)
    {
        // Stop motor
        g_motor_120_degree0.p_api->stop(g_motor_120_degree0.p_ctrl);
    }
    else if (fabs(command) < MAX_RPM)
    {
        // Set motor speed
        if ( motor_status != MOTOR_120_DEGREE_CTRL_STATUS_RUN)
        {
            g_motor_120_degree0.p_api->run(g_motor_120_degree0.p_ctrl);
        }

        g_motor_120_degree0.p_api->speedSet(g_motor_120_degree0.p_ctrl, command);
    }
}

// Get and publish motor speed on rpm
void publish_vel()
{
    // Get motor velocity
    RM_MOTOR_120_DEGREE_SpeedGet(g_motor_120_degree0.p_ctrl, &motor_velocity);

    // Publish speed
    std_msgs__msg__Float32 msg_vel_output;
    msg_vel_output.data = motor_velocity;
    rcl_publish(&pub_vel, &msg_vel_output, NULL);
}

// Apply gear reduction and publish joint position
void publish_joint()
{
    // Aux vars
    const float rpm_to_rad = (float) (2.0*3.1417)/60.0;
    static int64_t last_timestamp = 0;

    // Gear reduction for manipulator demo
    const float gear_reduction = 50.0;

    // Stamp header timestamp
    int64_t nanoseconds = rmw_uros_epoch_nanos();
    msg_joint.header.stamp.sec = (int32_t) (nanoseconds / 1000000000);
    msg_joint.header.stamp.nanosec = (uint32_t) nanoseconds % 1000000000;

    if (last_timestamp != 0)
    {
        // Calculate position
        double reducted_vel = rpm_to_rad * motor_velocity/gear_reduction;
        double delta_time = (double) (nanoseconds - last_timestamp)/1000000000.0;

        msg_joint.position.data[0] += reducted_vel*delta_time;
        msg_joint.velocity.data[0] = reducted_vel;
    }

    last_timestamp = nanoseconds;

    // Publish current joint position
    rcl_publish(&pub_joint, &msg_joint, NULL);
}

void microros_app(void)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();

    rclc_support_t support;
    rclc_support_init(&support, 0, NULL, &allocator);

    // Create node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "motor_node", "motor", &support));

    // Set subscriber custom QoS
    rmw_qos_profile_t sub_qos = rmw_qos_profile_default;
    sub_qos.depth = 1;

    // Create speed commands subscriber
    rclc_subscription_init(
        &sub_commands,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "cmd",
        &sub_qos);

    // Create motor speed publisher for graph
    RCCHECK(rclc_publisher_init_default(
        &pub_vel,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "speed"));

    // Create joint publisher for RViz
    RCCHECK(rclc_publisher_init_default(
        &pub_joint,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "joint"));

    // Allocate joint msg memory
    micro_ros_utilities_memory_conf_t conf = {
            .max_string_capacity = 20,
            .max_ros2_type_sequence_capacity = 1,
            .max_basic_type_sequence_capacity = 1
        };

    micro_ros_utilities_create_message_memory(
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        (void*) &msg_joint,
        conf);

    msg_joint.header.frame_id = micro_ros_string_utilities_set(msg_joint.header.frame_id, "manipulator");
    msg_joint.name.data[0] = micro_ros_string_utilities_set(msg_joint.name.data[0], "motor1");
    msg_joint.name.size = 1;
    msg_joint.position.size = 1;
    msg_joint.velocity.size = 1;

    // Create executor
    rclc_executor_t executor;
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &sub_commands, &msg_vel_input, &subscription_callback, ON_NEW_DATA);

    for(;;) {
        // micro-ROS loop
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50));
        publish_joint();
        publish_vel();
    }
}
