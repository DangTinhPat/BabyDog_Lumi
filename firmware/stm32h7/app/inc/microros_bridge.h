#ifndef MICROROS_BRIDGE_H
#define MICROROS_BRIDGE_H

#include <stdbool.h>
#include <stdint.h>

/* micro-ROS node "stm32_joint_node" voi subscriber "/joint_cmd" + publishers
 * "/joint_fb" and compact "/imu/raw" (main_bot_hardware_msgs), qua UART1/
 * micro_ros_agent thay vi CAN. Cung reconnect
 * state machine (ping dinh ky khi da connected) voi OUT_SAVE/testSTM/app/src/microros_bridge.c,
 * nhung KHONG FreeRTOS (dung Tick_GetMs() thay xTaskGetTickCount(), khong goi
 * MicroRos_InstallFreeRTOSAllocator() - xem microros_time.c va ghi chu trong .c).
 *
 * Callback subscription goi thang Actuator_SetTarget() (actuator_if.h) va tu cap nhat
 * timestamp lan cuoi nhan duoc /joint_cmd. main.c dung timestamp nay cho watchdog:
 * mat lenh thi ngat luc, khong chay FSM hay fallback ve goc co dinh. */
void MicroRosBridge_Begin(void);

/* Goi moi vong superloop cua main() (KHONG sleep/block ben trong) - tu quan ly ket noi/
 * tao entity/spin executor (xu ly callback subscription /joint_cmd neu co du lieu moi). */
void MicroRosBridge_SpinSome(void);

bool MicroRosBridge_IsConnected(void);

/* 0 neu chua bao gio nhan duoc /joint_cmd nao (kho boot) - dung y het gia tri khoi tao
 * g_last_joint_cmd_ms = 0 cua ban CAN cu. */
uint32_t MicroRosBridge_LastJointCmdMs(void);

/* No-op (bo qua) neu chua connected - an toan goi vo dieu kien moi chu ky
 * JOINT_FB_SEND_PERIOD_MS trong main.c. Tu doc Actuator_GetMeasured() ben trong. */
void MicroRosBridge_PublishJointFb(void);

typedef enum
{
    MICROROS_IMU_STATUS_OK = 1U,
    MICROROS_IMU_STATUS_INIT_FAILED = 2U,
    MICROROS_IMU_STATUS_READ_FAILED = 4U,
} MicroRosImuStatus;

struct MPU6050_Reading;

/* Publish one compact fixed-size IMU sample. reading may be NULL for a status-only
 * error frame; the EC Kalman node rejects any frame without STATUS_OK. */
void MicroRosBridge_PublishImuRaw(const struct MPU6050_Reading *reading,
                                  MicroRosImuStatus status,
                                  uint32_t stamp_ms);

#endif /* MICROROS_BRIDGE_H */
