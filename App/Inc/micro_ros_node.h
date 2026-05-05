/**
  * @file    micro_ros_node.h
  * @brief   micro-ROS node: cmd_vel subscriber, odom publisher
  */

#ifndef MICRO_ROS_NODE_H
#define MICRO_ROS_NODE_H

#include <stdint.h>

/**
 * @brief  Odometry data shared between Motion task and micro-ROS task.
 *         Written by Motion_TaskEntry, read by MicroROS_TaskEntry.
 */
typedef struct {
    float x;
    float y;
    float theta;
    float linear_vel;
    float angular_vel;
    uint32_t timestamp_ms;
} Odom_Data_t;

extern Odom_Data_t g_odom_data;

/**
 * @brief  micro-ROS FreeRTOS task entry point.
 */
void MicroROS_TaskEntry(void *argument);

#endif /* MICRO_ROS_NODE_H */
