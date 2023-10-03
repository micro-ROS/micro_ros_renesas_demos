#include "./utils.h"

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include <time.h>

#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <tf2_msgs/msg/tf_message.h>
#include <geometry_msgs/msg/pose.h>
#include <geometry_msgs/msg/twist.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){bool l = true; while(1){set_led_status(LED_RED, l = !l); sleep_ms(100);}}}

// Function definitions
void microros_app(void);
void get_tf2_data(tf2_msgs__msg__TFMessage * tf_msg);
void tf2_publisher_thread(void * args);
void get_odom_data(geometry_msgs__msg__Pose * odom_msg);
void odom_publisher_thread(void * args);
void subscription_callback(const void * msgin);
void twist_subscription_thread(void * args);

/*
* This application shows how to send a TF2 message and a Odometry message from different threads in micro-ROS
* while subscribing to a Twist message in a third thread.
*
* The main thread is kept free to run the required "user code".
*/

SemaphoreHandle_t microROS_mutex;
rclc_support_t support;

// ****************************
// *** TF2 publisher thread ***
// ****************************

#define MAX_TRANSFORMS 4
#define MAX_STRING_SIZE 10

void get_tf2_data(tf2_msgs__msg__TFMessage * tf_msg)
{
    // Fake TF2 data
    static int counter = 0;
    counter++;

    tf_msg->transforms.size = 1;
    tf_msg->transforms.data[0].transform.translation.x = counter;
    tf_msg->transforms.data[0].transform.translation.y = counter;
    tf_msg->transforms.data[0].transform.translation.z = counter;
}

void tf2_publisher_thread(void * args)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();

    // Get publisher from args
    rcl_publisher_t * tf2_publisher = (rcl_publisher_t *) args;

    // Initialize TF2 message
    tf2_msgs__msg__TFMessage tf_msg  = {};
    tf_msg.transforms.data = (geometry_msgs__msg__TransformStamped *) allocator.zero_allocate(MAX_TRANSFORMS, sizeof(geometry_msgs__msg__TransformStamped), NULL);
    tf_msg.transforms.size = 0;
    tf_msg.transforms.capacity = MAX_TRANSFORMS;

    for (size_t i = 0; i < tf_msg.transforms.capacity; i++)
    {
        tf_msg.transforms.data[i].header.frame_id.data = (char *) allocator.zero_allocate(MAX_STRING_SIZE, sizeof(char), NULL);
        tf_msg.transforms.data[i].header.frame_id.size = 0;
        tf_msg.transforms.data[i].header.frame_id.capacity = MAX_STRING_SIZE;

        tf_msg.transforms.data[i].child_frame_id.data = (char *) allocator.zero_allocate(MAX_STRING_SIZE, sizeof(char), NULL);
        tf_msg.transforms.data[i].child_frame_id.size = 0;
        tf_msg.transforms.data[i].child_frame_id.capacity = MAX_STRING_SIZE;
    }

    // Publish loop at 10 Hz
    for(;;){
        get_tf2_data(&tf_msg);

        xSemaphoreTake(microROS_mutex, portMAX_DELAY);
        rcl_publish(tf2_publisher, &tf_msg, NULL);
        xSemaphoreGive(microROS_mutex);

        vTaskDelay(100 * configTICK_RATE_HZ / 1000);
    }
}

// *****************************
// *** Odom publisher thread ***
// *****************************

void get_odom_data(geometry_msgs__msg__Pose * odom_msg)
{
    // Fake Odometry data
    static int counter = 0;
    counter++;

    odom_msg->position.x = counter;
    odom_msg->position.y = counter;
    odom_msg->position.z = counter;
}

void odom_publisher_thread(void * args)
{
    // Get publisher from args
    rcl_publisher_t * odom_publisher = (rcl_publisher_t *) args;

    // Initialize TF2 message
    geometry_msgs__msg__Pose odom_msg  = {};

    // Publish loop at 20 Hz
    for(;;){
        get_odom_data(&odom_msg);

        xSemaphoreTake(microROS_mutex, portMAX_DELAY);
        rcl_publish(odom_publisher, &odom_msg, NULL);
        xSemaphoreGive(microROS_mutex);

        vTaskDelay(50 * configTICK_RATE_HZ / 1000);
    }
}

// *********************************
// *** Twist subscription thread ***
// *********************************

void subscription_callback(const void * msgin)
{
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

    // Use twist here
    (void) msg;

    static bool led = 1;
    set_led_status(LED_GREEN, led);
    led = !led;
}

void twist_subscription_thread(void * args)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_executor_t executor;

    // Get subscriber from args
    rcl_subscription_t * subscriber = (rcl_subscription_t *) args;

    // Prepare incoming message memory
    geometry_msgs__msg__Twist msg;

    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, subscriber, &msg, &subscription_callback, ON_NEW_DATA));

    // Spin loop
    while(1){
        xSemaphoreTake(microROS_mutex, portMAX_DELAY);
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        xSemaphoreGive(microROS_mutex);

        vTaskDelay(10 * configTICK_RATE_HZ / 1000);
    }
}

// *****************
// *** Main loop ***
// *****************

void microros_app(void)
{
    // Create an application mutex
    microROS_mutex = xSemaphoreCreateMutex();

    //get allocator
    rcl_allocator_t allocator = rcl_get_default_allocator();

    //create init_options
    rclc_support_init(&support, 0, NULL, &allocator);

    // create nodes
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "my_renesas_node", "", &support));

    // create publishers
    rcl_publisher_t tf2_publisher;
    RCCHECK(rclc_publisher_init_default(
        &tf2_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
        "microros_tf2"));

    rcl_publisher_t odom_publisher;
    RCCHECK(rclc_publisher_init_default(
        &odom_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose),
        "microros_odom"));

    // create subscriber
    rcl_subscription_t twist_subscriber;
    RCCHECK(rclc_subscription_init_default(
        &twist_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "microros_twist"));

    // Launch TF2 publisher thread
    xTaskCreate(tf2_publisher_thread, "tf2_publisher", 1024, (void *) &tf2_publisher, 1, NULL);

    // Launch Odom publisher thread
    xTaskCreate(odom_publisher_thread, "odom_publisher", 1024, (void *) &odom_publisher, 1, NULL);

    // Launch Twist subscriber thread
    xTaskCreate(twist_subscription_thread, "twist_subscriber", 2048, (void *) &twist_subscriber, 2, NULL);

    // User application login here
    for(;;){
        sleep_ms(1000);
    }
}
