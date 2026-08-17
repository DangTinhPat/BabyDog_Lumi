#include "microros_bridge.h"

#include "tick.h"
#include "actuator_if.h"
#include "motor_topology.h"   /* JOINT_COUNT */
#include "microros_transport.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <main_bot_hardware_msgs/msg/joint_cmd.h>
#include <main_bot_hardware_msgs/msg/joint_fb.h>

/* Hạ từ 5ms xuống 1ms - benchmark thật đo được rclc_executor_spin_some() (bên trong,
 * thu vien rclc/rmw_microxrcedds) chiem trung binh 2.5ms/lan goi (nua ngan sach vong
 * lap 5ms cua JOINT_FB_SEND_PERIOD_MS), khien /joint_fb chi dat ~143Hz thuc te thay vi
 * 200Hz cau hinh. rx_ring da duoc ISR (USART1_IRQHandler) nap san khong dong bo voi
 * spin_some (xem microros_transport.c) nen khong can doi lau moi lan goi van bat kip du
 * lieu moi - dang do lai xem gia tri nay co giam duoc chi phi trung binh khong. */
#define EXECUTOR_SPIN_TIMEOUT_NS RCL_MS_TO_NS(1)
#define PING_INTERVAL_MS 2000U   /* health check khi da connected - ping moi lan spin se
                                   * chiem het bang thong UART */
#define PING_TIMEOUT_MS 500
#define PING_ATTEMPTS 3

typedef enum
{
    STATE_WAITING_AGENT,
    STATE_CONNECTED,
} BridgeState;

static BridgeState state = STATE_WAITING_AGENT;
static uint32_t last_ping_ms = 0U;
static uint32_t last_joint_cmd_ms = 0U;   /* 0 = chua bao gio nhan, khop gia tri khoi tao
                                            * g_last_joint_cmd_ms cu ben CAN */

static rclc_support_t support;
static rcl_allocator_t allocator;
static rcl_node_t node;
static rclc_executor_t executor;
static rcl_publisher_t joint_fb_pub;
static rcl_subscription_t joint_cmd_sub;
static main_bot_hardware_msgs__msg__JointFb joint_fb_msg;
static main_bot_hardware_msgs__msg__JointCmd joint_cmd_msg;

static void JointCmdCallback(const void *msgin)
{
    const main_bot_hardware_msgs__msg__JointCmd *cmd =
        (const main_bot_hardware_msgs__msg__JointCmd *)msgin;

    float angles_rad[JOINT_COUNT];
    for (uint32_t i = 0; i < JOINT_COUNT; i++)
    {
        angles_rad[i] = (float)cmd->target_angle_mrad[i] / 1000.0f;
    }
    const float kp = (float)cmd->kp_x100 / 100.0f;
    const float kd = (float)cmd->kd_x100 / 100.0f;

    Actuator_SetTarget(angles_rad, kp, kd);
    last_joint_cmd_ms = Tick_GetMs();
}

/* rclc_executor capacity=1 (chi 1 handle: subscription /joint_cmd) - khop
 * RMW_UXRCE_MAX_SUBSCRIPTIONS=1/MAX_PUBLISHERS=1 trong microros/firmware/mcu_ws/
 * colcon.meta. */
static bool CreateEntities(void)
{
    allocator = rcl_get_default_allocator();

    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK)
    {
        return false;
    }
    if (rclc_node_init_default(&node, "stm32_joint_node", "", &support) != RCL_RET_OK)
    {
        return false;
    }
    /* best_effort ca 2 chieu: mat 1 frame khong sao (frame sau bu, gui o nhip cao hon
     * timeout /joint_cmd nhieu lan) - tranh vong khu hoi ACK cua reliable lam giam
     * tan so thuc te, dung ly do da xac nhan qua IMU (xem OUT_SAVE/testSTM/README.md). */
    if (rclc_publisher_init_best_effort(
            &joint_fb_pub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(main_bot_hardware_msgs, msg, JointFb),
            "/joint_fb") != RCL_RET_OK)
    {
        return false;
    }
    if (rclc_subscription_init_best_effort(
            &joint_cmd_sub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(main_bot_hardware_msgs, msg, JointCmd),
            "/joint_cmd") != RCL_RET_OK)
    {
        return false;
    }
    if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK)
    {
        return false;
    }
    if (rclc_executor_add_subscription(
            &executor, &joint_cmd_sub, &joint_cmd_msg, &JointCmdCallback, ON_NEW_DATA) != RCL_RET_OK)
    {
        return false;
    }
    return true;
}

static void DestroyEntities(void)
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    (void)rcl_subscription_fini(&joint_cmd_sub, &node);
    (void)rcl_publisher_fini(&joint_fb_pub, &node);
    (void)rclc_executor_fini(&executor);
    (void)rcl_node_fini(&node);
    (void)rclc_support_fini(&support);
}

void MicroRosBridge_Begin(void)
{
    main_bot_hardware_msgs__msg__JointFb__init(&joint_fb_msg);
    main_bot_hardware_msgs__msg__JointCmd__init(&joint_cmd_msg);

    /* Khong goi allocator rieng nhu ban OUT_SAVE/testSTM (MicroRos_InstallFreeRTOSAllocator) -
     * khong FreeRTOS o day, rcutils's default allocator da bọc san malloc/free/realloc/
     * calloc cua newlib (hoat dong qua nosys.specs + symbol _end trong linker script,
     * xem README.md), dung thang duoc. */
    MicroRos_RegisterTransport();
    state = STATE_WAITING_AGENT;
}

void MicroRosBridge_SpinSome(void)
{
    switch (state)
    {
        case STATE_WAITING_AGENT:
            if (CreateEntities())
            {
                state = STATE_CONNECTED;
                last_ping_ms = Tick_GetMs();
            }
            else
            {
                DestroyEntities();
                /* rclc_support_init() da tu cho/retry noi bo (UCLIENT_MIN_SESSION_
                 * CONNECTION_INTERVAL, xem toolchain.cmake) - vong lap superloop ben
                 * ngoai (main.c) la du, khong can them delay/ping o day. */
            }
            break;

        case STATE_CONNECTED:
        {
            const uint32_t now = Tick_GetMs();
            if ((now - last_ping_ms) >= PING_INTERVAL_MS)
            {
                last_ping_ms = now;
                if (rmw_uros_ping_agent(PING_TIMEOUT_MS, PING_ATTEMPTS) != RMW_RET_OK)
                {
                    DestroyEntities();
                    state = STATE_WAITING_AGENT;
                    break;
                }
            }
            (void)rclc_executor_spin_some(&executor, EXECUTOR_SPIN_TIMEOUT_NS);
            break;
        }
    }
}

bool MicroRosBridge_IsConnected(void)
{
    return state == STATE_CONNECTED;
}

uint32_t MicroRosBridge_LastJointCmdMs(void)
{
    return last_joint_cmd_ms;
}

void MicroRosBridge_PublishJointFb(void)
{
    if (state != STATE_CONNECTED)
    {
        return;
    }

    float angles_rad[JOINT_COUNT];
    float velocities_rad_s[JOINT_COUNT];
    Actuator_GetMeasured(angles_rad, velocities_rad_s);
    for (uint32_t i = 0; i < JOINT_COUNT; i++)
    {
        joint_fb_msg.measured_angle_mrad[i] = (int16_t)(angles_rad[i] * 1000.0f);
        joint_fb_msg.measured_velocity_mrad_s[i] = (int16_t)(velocities_rad_s[i] * 1000.0f);
    }

    (void)rcl_publish(&joint_fb_pub, &joint_fb_msg, NULL);
}
