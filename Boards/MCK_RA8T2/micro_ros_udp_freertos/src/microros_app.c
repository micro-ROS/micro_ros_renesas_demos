#include "./utils.h"

#include <time.h>

#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){bool l = true; while(1){set_led_status(LED_RED, l = !l); sleep_ms(100);}}}

void microros_app(void);
void motor_callback(const void * msgin);
void process_motor_command(int cmd);

std_msgs__msg__Int32 motor_command_msg;

/***********************************************************************************************************************
 * Subscription callback
 * This function is called when a new message is received on the motor command topic.
 * It extracts the speed command from the message and calls the motor control function.
 ***********************************************************************************************************************/
void motor_callback(const void * msgin)
{
    const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
    int command = msg->data;

    process_motor_command(command);
}

/***********************************************************************************************************************
 * Micro-ROS subscription node creation and execution
 ***********************************************************************************************************************/
void microros_app(void)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();

    // create init_options
    rclc_support_t support;
    rclc_support_init(&support, 0, NULL, &allocator);

    // create nodes
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "renesas_motor_node", "", &support));

    // create subscriber
    rcl_subscription_t subscriber = rcl_get_zero_initialized_subscription();;
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "renesas_motor_topic"));

    // Create executor
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();;
    RCCHECK(rclc_executor_init(
        &executor,
        &support.context,
        1,
        &allocator));

    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &motor_command_msg, &motor_callback, ON_NEW_DATA));
    rclc_executor_spin(&executor);

    RCCHECK(rcl_subscription_fini(&subscriber, &node));
    RCCHECK(rcl_node_fini(&node));
}
