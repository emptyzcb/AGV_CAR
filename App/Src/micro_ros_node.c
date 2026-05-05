/**
  * @file    micro_ros_node.c
  * @brief   micro-ROS node: cmd_vel -> AGV, odom -> ROS2
  *
  * Creates a micro-ROS node "agv_car" that:
  *   - Subscribes to /cmd_vel (geometry_msgs/Twist)
  *   - Publishes  /odom      (nav_msgs/Odometry)     at 20Hz
  *   - Publishes  /agv_status(std_msgs/String)       at 1Hz
  */

#include "micro_ros_node.h"
#include "micro_ros_transport.h"
#include "agv_task.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <std_msgs/msg/string.h>

#include <rosidl_runtime_c/string_functions.h>

#include <math.h>

/* ---- Shared odom data (written by Motion task) ---- */
Odom_Data_t g_odom_data = {0};

/* ---- micro-ROS objects ---- */
static rcl_allocator_t        allocator;
static rclc_support_t         support;
static rcl_node_t             node;
static rcl_publisher_t        odom_pub;
static rcl_publisher_t        status_pub;
static rcl_subscription_t     cmd_vel_sub;
static rclc_executor_t        executor;

/* Message instances (static to avoid heap fragmentation) */
static nav_msgs__msg__Odometry   odom_msg;
static geometry_msgs__msg__Twist cmd_vel_msg;
static std_msgs__msg__String     status_msg;

/* ---- Error handling ---- */
#define RCCHECK(fn) do { rcl_ret_t rc = (fn); if (rc != RCL_RET_OK) { error_loop(); } } while(0)
#define RCSOFTCHECK(fn) do { (void)(fn); } while(0)

static void error_loop(void)
{
    for (;;)
    {
        osDelay(200);
    }
}

/* ---- cmd_vel subscription callback ---- */
static void cmd_vel_callback(const void *msgin)
{
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    float linear  = (float)msg->linear.x;
    float angular = (float)msg->angular.z;

    AGV_SetVelocity(linear, angular);
    AGV_SetMode(AGV_MODE_REMOTE);
}

/* ---- Publish odometry ---- */
static void publish_odom(uint32_t now_ms)
{
    odom_msg.header.stamp.sec     = (int32_t)(now_ms / 1000);
    odom_msg.header.stamp.nanosec = (uint32_t)((now_ms % 1000) * 1000000);

    odom_msg.pose.pose.position.x = g_odom_data.x;
    odom_msg.pose.pose.position.y = g_odom_data.y;
    odom_msg.pose.pose.position.z = 0.0;

    odom_msg.pose.pose.orientation.z = sinf(g_odom_data.theta * 0.5f);
    odom_msg.pose.pose.orientation.w = cosf(g_odom_data.theta * 0.5f);

    odom_msg.twist.twist.linear.x  = g_odom_data.linear_vel;
    odom_msg.twist.twist.linear.y  = 0.0;
    odom_msg.twist.twist.angular.z = g_odom_data.angular_vel;

    RCSOFTCHECK(rcl_publish(&odom_pub, &odom_msg, NULL));
}

/* ---- Publish status ---- */
static void publish_status(void)
{
    const AGV_State_t *state = AGV_GetState();

    char buf[64];
    int len = snprintf(buf, sizeof(buf), "{\"mode\":%d,\"err\":%lu}",
                       (int)state->mode, (unsigned long)state->error_code);

    rosidl_runtime_c__String__fini(&status_msg.data);
    rosidl_runtime_c__String__init(&status_msg.data);
    rosidl_runtime_c__String__assignn(&status_msg.data, buf, (size_t)len);

    RCSOFTCHECK(rcl_publish(&status_pub, &status_msg, NULL));
}

/* ---- Main task entry ---- */
void MicroROS_TaskEntry(void *argument)
{
    (void)argument;

    /* Phase 1: Wait for agent connection */
    while (rmw_uros_ping_agent_timeout(1000, 3) != RCL_RET_OK)
    {
        osDelay(1000);
    }

    /* Phase 2: Init transport */
    micro_ros_transport_init();

    /* Phase 3: Create micro-ROS infrastructure */
    allocator = rcl_get_default_allocator();

    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "agv_car", "", &support));

    /* Publishers */
    RCCHECK(rclc_publisher_init_default(
        &odom_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom"));

    RCCHECK(rclc_publisher_init_default(
        &status_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "agv_status"));

    /* Subscriber */
    RCCHECK(rclc_subscription_init_default(
        &cmd_vel_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"));

    /* Executor: 1 subscription */
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor, &cmd_vel_sub, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA));

    /* Phase 4: Spin loop */
    uint32_t last_odom_tick   = 0;
    uint32_t last_status_tick = 0;

    for (;;)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

        uint32_t now = osKernelGetTick();

        /* Publish odom at 20Hz */
        if ((now - last_odom_tick) >= 50)
        {
            last_odom_tick = now;
            publish_odom(now);
        }

        /* Publish status at 1Hz */
        if ((now - last_status_tick) >= 1000)
        {
            last_status_tick = now;
            publish_status();
        }

        osDelay(10);
    }
}
