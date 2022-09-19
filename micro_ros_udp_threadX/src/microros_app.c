#include "./utils.h"

#include <time.h>

#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "example_interfaces/srv/add_two_ints.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){bool l = true; while(1){set_led_status(LED_RED, l = !l); sleep_ms(100);}}}

void microros_app(void);
void service_callback(const void * req, void * res);


void service_callback(const void * req, void * res){
  example_interfaces__srv__AddTwoInts_Request * req_in = (example_interfaces__srv__AddTwoInts_Request *) req;
  example_interfaces__srv__AddTwoInts_Response * res_in = (example_interfaces__srv__AddTwoInts_Response *) res;

  // Heavy processing
  set_led_status(LED_RED, true);
  sleep_ms(500);
  set_led_status(LED_GREEN, true);
  sleep_ms(500);
  set_led_status(LED_BLUE, true);
  sleep_ms(500);

  res_in->sum = req_in->a + req_in->b;

  set_led_status(LED_RED, false);
  set_led_status(LED_GREEN, false);
  set_led_status(LED_BLUE, false);
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

    // create service
    rcl_service_t service;
    RCCHECK(rclc_service_init_default(&service,
    		&node,
			ROSIDL_GET_SRV_TYPE_SUPPORT(example_interfaces, srv, AddTwoInts),
			"/add_two_ints"));

	// create executor
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

    example_interfaces__srv__AddTwoInts_Response res;
    example_interfaces__srv__AddTwoInts_Request req;
    RCCHECK(rclc_executor_add_service(&executor, &service, &req, &res, service_callback));

	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        sleep_ms(100);
	}
}
