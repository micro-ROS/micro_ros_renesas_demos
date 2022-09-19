#include "./utils.h"

#include <time.h>

#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc_parameter/rclc_parameter.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){bool l = true; while(1){set_led_status(LED_RED, l = !l); sleep_ms(100);}}}

void microros_app(void);
bool on_parameter_changed(const Parameter * old_param, const Parameter * new_param, void * context);

bool on_parameter_changed(const Parameter * old_param, const Parameter * new_param, void * context)
{
	(void) context;

	bool ret = true;

	if (old_param == NULL || new_param == NULL)
	{
		return false;
	}

	if (strcmp(new_param->name.data, "red") == 0)
	{
		set_led_status(LED_RED, new_param->value.bool_value);
	}
	else if (strcmp(new_param->name.data, "blue") == 0)
	{
		set_led_status(LED_BLUE, new_param->value.bool_value);
	}
	else if (strcmp(new_param->name.data, "green") == 0)
	{
		set_led_status(LED_GREEN, new_param->value.bool_value);
	}
	else
	{
		ret = false;
	}

	return ret;
}

void microros_app(void)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();

    //create init_options
    rclc_support_t support;
    rclc_support_init(&support, 0, NULL, &allocator);

    // create nodes
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "param_node", "", &support));

  	// Create parameter service
	rclc_parameter_server_t param_server;
    rclc_parameter_server_init_default(&param_server, &node);

	// create executor
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES, &allocator));

	RCCHECK(rclc_executor_add_parameter_server(&executor, &param_server, on_parameter_changed));

    rclc_add_parameter(&param_server, "red", RCLC_PARAMETER_BOOL);
    rclc_add_parameter(&param_server, "green", RCLC_PARAMETER_BOOL);
    rclc_add_parameter(&param_server, "blue", RCLC_PARAMETER_BOOL);


	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        sleep_ms(10);
	}
}
