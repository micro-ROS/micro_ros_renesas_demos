#include "./utils.h"

#include <time.h>

#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){bool l = true; while(1){set_led_status(LED_RED, l = !l); sleep_ms(100);}}}

void microros_app(void);
void timer_callback_1(rcl_timer_t * timer, int64_t last_call_time);
void timer_callback_2(rcl_timer_t * timer, int64_t last_call_time);

rcl_publisher_t publisher_1;
rcl_publisher_t publisher_2;

void timer_callback_1(rcl_timer_t * timer, int64_t last_call_time)
{
	(void) last_call_time;
	(void) timer;

	static std_msgs__msg__Int32 msg = {0};

	set_led_status(LED_BLUE, msg.data % 2 == 0);
	rcl_publish(&publisher_1, &msg, NULL);
	msg.data++;
}

void timer_callback_2(rcl_timer_t * timer, int64_t last_call_time)
{
	(void) last_call_time;
	(void) timer;

	static std_msgs__msg__Int32 msg = {0};

	set_led_status(LED_GREEN, msg.data % 2 == 0);
	rcl_publish(&publisher_2, &msg, NULL);
	msg.data++;
}

void microros_app(void)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();

    //create init_options
    rclc_support_t support;
    rclc_support_init(&support, 0, NULL, &allocator);

    // create nodes
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "my_renesas_node", "", &support));

    // create publishers
    RCCHECK(rclc_publisher_init_default(
        &publisher_1,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "int_publisher_1"));

    RCCHECK(rclc_publisher_init_default(
        &publisher_2,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "int_publisher_2"));

	// create timer,
	rcl_timer_t timer_1;
	RCCHECK(rclc_timer_init_default(
		&timer_1,
		&support,
		RCL_MS_TO_NS(1000),
		timer_callback_1));

	rcl_timer_t timer_2;
	RCCHECK(rclc_timer_init_default(
		&timer_2,
		&support,
		RCL_MS_TO_NS(500),
		timer_callback_2));

	// create executor
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer_1));
	RCCHECK(rclc_executor_add_timer(&executor, &timer_2));

	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        sleep_ms(100);
	}
}
