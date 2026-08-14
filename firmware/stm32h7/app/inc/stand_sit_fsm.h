/**
 ******************************************************************************
 * @file    stand_sit_fsm.h
 * @brief   FSM đứng lên / ngồi xuống chạy trên STM32H7 - đối xứng về hành vi
 *          với stand_sit_controller (gói ROS2 phía mô phỏng/RDK X5): cùng 3
 *          trạng thái Passive/Stand/Sit, cùng kiểu nội suy tanh, cùng quy ước
 *          command (xem protocol.h). Không có bộ ước lượng/cân bằng chủ động
 *          (chưa có IMU) - giữ tư thế bằng PD góc khớp cố định như phía sim.
 * ******************************************************************************
 */
#ifndef STAND_SIT_FSM_H
#define STAND_SIT_FSM_H

#include <stdint.h>
#include "protocol.h"

void FSM_Init(void);

/* Gọi mỗi khi nhận được 1 CMD frame hợp lệ qua CAN FD (đã qua decode). */
void FSM_OnCommandReceived(uint8_t command, uint8_t seq, uint32_t now_ms);

/* Gọi liên tục trong vòng lặp chính - cập nhật nội suy góc khớp + kiểm tra
 * mất kết nối (LINK_TIMEOUT_MS không nhận được CMD frame nào -> tự ESTOP). */
void FSM_Update(uint32_t now_ms);

FsmState_t FSM_GetState(void);
int FSM_IsSettled(void);
uint8_t FSM_GetFaultFlags(void);
uint8_t FSM_GetLastSeq(void);

#endif /* STAND_SIT_FSM_H */
