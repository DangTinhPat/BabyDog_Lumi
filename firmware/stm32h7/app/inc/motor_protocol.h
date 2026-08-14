/**
 ******************************************************************************
 * @file    motor_protocol.h
 * @brief   Dinh danh 12 khop + tra cuu bus/ID/loai-khop/nhom-chan cho giao
 *          thuc BabyAlpha2 (CAN-FD, xem baby_alpha2_protocol.h) giua STM32H7
 *          (main board) va 12 board driver dong co that o moi khop. Ke thua
 *          topology tu du an oneLeg (/home/dvt/OUT_SAVE/babyDog_test/oneLeg,
 *          main_12joint_hold_proven.c.bak) - da chay that tren phan cung.
 *
 *          Topology BUS: 2 bus CAN-FD, moi bus 6 dong co, ID 1-6 TRONG bus
 *          (khong phai ID toan cuc nhu ban cu). CAN_INSTANCE_1 (FDCAN1,
 *          PA11/12) = 2 chan TRUOC (front_right id1-3, front_left id4-6).
 *          CAN_INSTANCE_2 (FDCAN2, PB5/6) = 2 chan SAU (hind_right id1-3,
 *          hind_left id4-6). Trong 1 chan, id 1=Hang(abad, roll), 2=Dui(hip,
 *          pitch), 3=Goi(knee, pitch) - LAP LAI cho chan thu 2 cua bus (id
 *          4=Hang,5=Dui,6=Goi).
 *
 *          QUAN TRONG: phan hoi (PING/HANDSHAKE/SETUP/telemetry) tu MOI dong
 *          co TRONG 1 bus deu ve CHUNG 1 CAN ID = 0 - phan biet dong co nao
 *          bang byte data[0] (gia tri 1-6 cho telemetry Page0, hoac
 *          0x08/0x18/0x38 + id cho cac phan hoi he thong), KHONG phai bang
 *          CAN ID nhu ban giao thuc tu bia truoc day. Xem actuator_if.c.
 ******************************************************************************
 */
#ifndef MOTOR_PROTOCOL_H
#define MOTOR_PROTOCOL_H

#include <stdint.h>
#include "can.h" /* CAN_INSTANCE_1/2 */

/* Thứ tự khớp 0-11, khớp với joints: trong
 * src/stand_sit_controller/.../config/controllers.yaml (phía ROS2) và
 * src/main_bot/description/babydog.xacro. */
typedef enum
{
    JOINT_FRONT_RIGHT_ABAD = 0,
    JOINT_FRONT_RIGHT_HIP = 1,
    JOINT_FRONT_RIGHT_KNEE = 2,
    JOINT_FRONT_LEFT_ABAD = 3,
    JOINT_FRONT_LEFT_HIP = 4,
    JOINT_FRONT_LEFT_KNEE = 5,
    JOINT_HIND_RIGHT_ABAD = 6,
    JOINT_HIND_RIGHT_HIP = 7,
    JOINT_HIND_RIGHT_KNEE = 8,
    JOINT_HIND_LEFT_ABAD = 9,
    JOINT_HIND_LEFT_HIP = 10,
    JOINT_HIND_LEFT_KNEE = 11,
    JOINT_COUNT = 12
} JointIndex_t;

/* Loai khop (dung tra bang gioi han hanh trinh theo LOAI, KHONG theo ID CAN
 * 1-6 - bay da tung sap: id 4,5,6 doc nham vao mang chi co 4 phan tu neu
 * dung id truc tiep, xem oneLeg/HW.md muc 4). Gia tri 1-indexed (index 0
 * khong dung) de khop dung quy uoc bang gioi han ke thua tu oneLeg. */
typedef enum
{
    JOINT_TYPE_HANG = 1,   /* Hang/abad - roll */
    JOINT_TYPE_DUI  = 2,   /* Dui/hip - pitch */
    JOINT_TYPE_GOI  = 3,   /* Goi/knee - pitch */
} JointType_t;

/* Nhom chan - dung cho logic mirror gioi han (xem motor_calib.h). Thu tu
 * TRUNG KHOP voi JointIndex_t o tren (joint/3 cho dung gia tri nay). */
typedef enum
{
    LEG_FRONT_RIGHT = 0,
    LEG_FRONT_LEFT  = 1,
    LEG_HIND_RIGHT  = 2,
    LEG_HIND_LEFT   = 3,
} LegGroup_t;

/* Bus CAN cho khop - front (0-5) tren CAN_INSTANCE_1, hind (6-11) tren
 * CAN_INSTANCE_2. Dung nguyen tu ban giao thuc cu (da dung sac tu dau). */
static inline uint32_t Motor_BusForJoint(JointIndex_t joint)
{
    return (joint < 6) ? CAN_INSTANCE_1 : CAN_INSTANCE_2;
}

/* ID dong co TRONG bus (1-6) - dung de dinh dia chi khung CAN gui DI (lenh
 * PD/PING/HANDSHAKE/SETUP_LIMITS). Phan hoi KHONG dung ID nay (xem module
 * header o tren - phan hoi luon ve ID=0, phan biet bang data[0]). */
static inline uint32_t Motor_IdForJoint(JointIndex_t joint)
{
    return ((uint32_t)joint % 6U) + 1U;
}

/* Loai khop (1=Hang,2=Dui,3=Goi) - dung tra bang gioi han hanh trinh theo
 * LOAI (motor_calib.h), KHONG dung Motor_IdForJoint() cho viec nay. */
static inline JointType_t Motor_JointTypeForJoint(JointIndex_t joint)
{
    return (JointType_t)(((uint32_t)joint % 3U) + 1U);
}

/* Nhom chan (0-3, xem LegGroup_t) - dung cho logic mirror gioi han. */
static inline LegGroup_t Motor_LegGroupForJoint(JointIndex_t joint)
{
    return (LegGroup_t)((uint32_t)joint / 3U);
}

#endif /* MOTOR_PROTOCOL_H */
