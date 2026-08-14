/**
 ******************************************************************************
 * @file    protocol.h
 * @brief   RDK X5 <-> STM32H7 wire protocol - stand up / sit down phase.
 *          Gửi qua CAN FD (fd_format=true trong lib/can.h's CAN_Frame),
 *          KHÔNG bật bit_rate_switch (BRS=false) - dùng định dạng FD để có
 *          sẵn payload dài hơn cho tương lai, nhưng tốc độ vẫn 1 mức duy
 *          nhất (nominal) cho cả 2 pha, giảm rủi ro bring-up. Khác với
 *          motor_protocol.h (STM32 <-> board driver mỗi khớp) dùng Classic
 *          CAN (fd_format=false) theo đúng yêu cầu.
 *
 *          Đây là bản sao PHẢI khớp byte-từng-byte với
 *          src/fdcan_bridge/fdcan_bridge/protocol.py (phía RDK X5/Python) -
 *          sửa 1 bên thì phải sửa bên kia theo, không có cơ chế tự đồng bộ.
 *
 *          CMD_CAN_ID (0x100), RDK -> STM32, gửi liên tục ~20Hz (không chỉ
 *          khi đổi lệnh) để firmware phát hiện mất kết nối:
 *            byte 0: command   (xem enum Command)
 *            byte 1: seq       (uint8, tăng dần mỗi lần gửi, quay vòng)
 *            byte 2-7: reserved, phải = 0
 *
 *          STATUS_CAN_ID (0x101), STM32 -> RDK, chỉ mang tính thông tin:
 *            byte 0: fsm_state (xem enum FsmState)
 *            byte 1: settled    (0 = đang nội suy, 1 = đã giữ ổn định)
 *            byte 2: fault_flags (bitfield, xem enum FaultFlag)
 *            byte 3: last_seq   (echo lại seq của CMD frame gần nhất đã xử lý)
 *            byte 4-7: reserved (vd IMU roll/pitch sau này)
 *
 *          Hợp đồng an toàn: nếu firmware không nhận được CMD frame hợp lệ
 *          trong LINK_TIMEOUT_MS, PHẢI tự chuyển về PASSIVE (ngắt lực) bất kể
 *          lệnh gần nhất là gì.
 *
 *          --- ros2_control (main_bot_hardware, phía RDK X5/laptop) ---
 *          KHÔNG còn đi qua CAN nữa (đã thay bằng micro-ROS/UART1 - xem
 *          microros_bridge.c: subscriber "/joint_cmd" + publisher "/joint_fb",
 *          main_bot_hardware_msgs/msg/JointCmd.msg + JointFb.msg - cùng field/
 *          đơn vị mrad/x100 như bản CAN cũ, chỉ đổi transport). "Chế độ relay"
 *          trong main.c (bỏ qua FSM_Update() khi /joint_cmd đang tới đều đặn,
 *          rơi về CMD_CAN_ID/STATUS_CAN_ID cũ khi mất liên kết) giữ nguyên ý
 *          nghĩa, chỉ đổi nguồn cập nhật timestamp sang
 *          MicroRosBridge_LastJointCmdMs().
 ******************************************************************************
 */
#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

#define CMD_CAN_ID          0x100U
#define STATUS_CAN_ID       0x101U

#define LINK_TIMEOUT_MS     500U   /* phải khớp fdcan_bridge/protocol.py LINK_TIMEOUT_S */
#define JOINT_LINK_TIMEOUT_MS 200U /* mat /joint_cmd qua khoang nay -> roi ve FSM cu */

typedef enum
{
    CMD_NONE  = 0,
    CMD_STAND = 1,
    CMD_SIT   = 2,
    CMD_ESTOP = 9
} Command_t;

typedef enum
{
    FSM_PASSIVE = 0,
    FSM_STAND   = 1,
    FSM_SIT     = 2
} FsmState_t;

typedef enum
{
    FAULT_LINK_TIMEOUT_RECOVERED = (1U << 0),
    FAULT_ESTOP_ACTIVE           = (1U << 1)
} FaultFlag_t;

typedef struct
{
    uint8_t command;
    uint8_t seq;
    uint8_t reserved[6];
} __attribute__((packed)) CmdFrame_t;

typedef struct
{
    uint8_t fsm_state;
    uint8_t settled;
    uint8_t fault_flags;
    uint8_t last_seq;
    uint8_t reserved[4];
} __attribute__((packed)) StatusFrame_t;

#endif /* PROTOCOL_H */
