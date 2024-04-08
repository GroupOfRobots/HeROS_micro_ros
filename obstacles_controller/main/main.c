
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

// Motors control BEGIN 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

// Define GPIO pins connected to TB6612FNG motor driver
#define MOTOR_A_IN1_GPIO 18
#define MOTOR_A_IN2_GPIO 19
#define MOTOR_B_IN3_GPIO 21
#define MOTOR_B_IN4_GPIO 22
#define PWM_PIN_A 9
#define PWM_PIN_B 10

// Define PWM channels for motor speed control
#define MOTOR_A_PWM_CHANNEL LEDC_CHANNEL_0
#define MOTOR_B_PWM_CHANNEL LEDC_CHANNEL_1
#define PWM_FREQUENCY 1000 // PWM frequency in Hz

#define MOTOR_STBY_GPIO 23
#define ROT_VEL 150
#define PERIOD 3200

void motor_init() {
    // Set the direction of the STBY pin as an output
    gpio_set_direction(MOTOR_STBY_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(MOTOR_STBY_GPIO, 1);

    // Configure GPIO pins for motor control (IN1, IN2, IN3, IN4)
    gpio_set_direction(MOTOR_A_IN1_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_A_IN2_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_B_IN3_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_B_IN4_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(PWM_PIN_A, GPIO_MODE_OUTPUT);
    gpio_set_direction(PWM_PIN_B, GPIO_MODE_OUTPUT);

    // Configure PWM for motor speed control
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_10_BIT, // PWM duty resolution
        .freq_hz = PWM_FREQUENCY,             // PWM frequency
        .speed_mode = LEDC_HIGH_SPEED_MODE,   // High-speed mode
        .timer_num = LEDC_TIMER_0             // Timer to use
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel_a = {
        .channel = MOTOR_A_PWM_CHANNEL,
        .duty = 0,
        .gpio_num = PWM_PIN_A,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0
    };
    ledc_channel_config(&ledc_channel_a);

    ledc_channel_config_t ledc_channel_b = {
        .channel = MOTOR_B_PWM_CHANNEL,
        .duty = 0,
        .gpio_num = PWM_PIN_B,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0
    };
    ledc_channel_config(&ledc_channel_b);
}

void motor_control(int speed_a, int speed_b, int dir_a, int dir_b) {
    // Set motor direction
    gpio_set_level(MOTOR_A_IN1_GPIO, dir_a);
    gpio_set_level(MOTOR_A_IN2_GPIO, !dir_a);
    gpio_set_level(MOTOR_B_IN3_GPIO, dir_b);
    gpio_set_level(MOTOR_B_IN4_GPIO, !dir_b);

    // Set motor speed using PWM duty cycle
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, MOTOR_A_PWM_CHANNEL, speed_a);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, MOTOR_A_PWM_CHANNEL);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, MOTOR_B_PWM_CHANNEL, speed_b);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, MOTOR_B_PWM_CHANNEL);
}
// Motors control END 

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
        motor_control(ROT_VEL, 0, 1, 1);
        vTaskDelay(PERIOD / portTICK_PERIOD_MS);
        motor_control(0, 0, 0, 0);
        res_in->success = true;
    }
    else
    {
        motor_control(ROT_VEL, 0, 0, 0);
        vTaskDelay(PERIOD / portTICK_PERIOD_MS);
        motor_control(0, 0, 0, 0);
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
        motor_control(0, ROT_VEL, 1, 1);
        vTaskDelay(PERIOD / portTICK_PERIOD_MS);
        motor_control(0, 0, 0, 0);
        res_in->success = true;
    }
    else
    {
        motor_control(0, ROT_VEL, 0, 0);
        vTaskDelay(PERIOD / portTICK_PERIOD_MS);
        motor_control(0, 0, 0, 0);
        res_in->success = true;
    }
}

void micro_ros_task(void *arg)
{
    // Motors control BEGIN 
    motor_init();
    // Motors control END 

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