/**
  * @file    micro_ros_transport.h
  * @brief   micro-ROS custom serial transport over USART1
  */

#ifndef MICRO_ROS_TRANSPORT_H
#define MICRO_ROS_TRANSPORT_H

/**
 * @brief  Register USART1 as the micro-ROS custom transport.
 *         Call before any rcl operations.
 */
void micro_ros_transport_init(void);

#endif /* MICRO_ROS_TRANSPORT_H */
