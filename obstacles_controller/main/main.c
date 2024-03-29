
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"

#include <uros_network_interfaces.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "example_interfaces/srv/set_bool.h"
#include "rosidl_runtime_c/string_functions.h"

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#define RCCHECK(fn)                                                                      \
    {                                                                                    \
        rcl_ret_t temp_rc = fn;                                                          \
        if ((temp_rc != RCL_RET_OK))                                                     \
        {                                                                                \
            printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
        }                                                                                \
    }
#define RCSOFTCHECK(fn)                                                                    \
    {                                                                                      \
        rcl_ret_t temp_rc = fn;                                                            \
        if ((temp_rc != RCL_RET_OK))                                                       \
        {                                                                                  \
            printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc); \
        }                                                                                  \
    }

void service_callback1(const void *req, void *res)
{
    example_interfaces__srv__SetBool_Request *req_in = (example_interfaces__srv__SetBool_Request *)req;
    example_interfaces__srv__SetBool_Response *res_in = (example_interfaces__srv__SetBool_Response *)res;

    printf("Obstacle 1 received request: %s\n", req_in->data ? "open" : "close");

    if (req_in->data)
    {
        res_in->success = true;
    }
    else
    {
        res_in->success = true;
    }
}

void service_callback2(const void *req, void *res)
{
    example_interfaces__srv__SetBool_Request *req_in = (example_interfaces__srv__SetBool_Request *)req;
    example_interfaces__srv__SetBool_Response *res_in = (example_interfaces__srv__SetBool_Response *)res;

    printf("Obstacle 2 received request: %s\n", req_in->data ? "open" : "close");

    if (req_in->data)
    {
        res_in->success = true;
    }
    else
    {
        res_in->success = true;
    }
}

void micro_ros_task(void *arg)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    // Create init_options.
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

    // Static Agent IP and port can be used instead of autodisvery.
    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
    // RCCHECK(rmw_uros_discover_agent(rmw_options));
#endif

    // Setup support structure.
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    // create node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "obstacles_controller", "", &support));

    // create service
    rcl_service_t service1;
    rcl_service_t service2;
    RCCHECK(rclc_service_init_default(&service1, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(example_interfaces, srv, SetBool), "/obstacle_1"));
    RCCHECK(rclc_service_init_default(&service2, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(example_interfaces, srv, SetBool), "/obstacle_2"));

    // create executor
    rclc_executor_t executor;
    // 2 is num_handles
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));

    example_interfaces__srv__SetBool_Response res1;
    example_interfaces__srv__SetBool_Request req1;
    RCCHECK(rclc_executor_add_service(&executor, &service1, &req1, &res1, service_callback1));

    example_interfaces__srv__SetBool_Response res2;
    example_interfaces__srv__SetBool_Request req2;
    RCCHECK(rclc_executor_add_service(&executor, &service2, &req2, &res2, service_callback2));

    // Spin forever
    while (1)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(100000);
    }

    // Free resources
    RCCHECK(rcl_service_fini(&service1, &node));
    RCCHECK(rcl_service_fini(&service2, &node));
    RCCHECK(rcl_node_fini(&node));
}

void app_main(void)
{
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

    // pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
    xTaskCreate(micro_ros_task,
                "uros_task",
                CONFIG_MICRO_ROS_APP_STACK,
                NULL,
                CONFIG_MICRO_ROS_APP_TASK_PRIO,
                NULL);
}