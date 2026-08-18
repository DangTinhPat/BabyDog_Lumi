#include "microros_bridge.h"

#include "tick.h"
#include "actuator_if.h"
#include "motor_topology.h"   /* JOINT_COUNT */
#include "microros_transport.h"
#include "mpu6050.h"

#include <limits.h>
#include <math.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <main_bot_hardware_msgs/msg/imu_raw.h>
#include <main_bot_hardware_msgs/msg/joint_cmd.h>
#include <main_bot_hardware_msgs/msg/joint_fb.h>

/* JointCmd.msg dung mang co dinh [12]. Neu topology firmware doi kich thuoc
 * ma message chua regenerate, dung build ngay thay vi doc/ghi lech mang. */
_Static_assert(JOINT_COUNT == 12, "JointCmd/JointFb contract requires exactly 12 joints");

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
static rcl_publisher_t imu_raw_pub;
static rcl_subscription_t joint_cmd_sub;
static main_bot_hardware_msgs__msg__ImuRaw imu_raw_msg;
static main_bot_hardware_msgs__msg__JointFb joint_fb_msg;
static main_bot_hardware_msgs__msg__JointCmd joint_cmd_msg;

/* Entity creation may fail at any step while the serial agent is absent.
 * Track exactly what became live so cleanup never passes a zero/partial entity
 * into an rcl fini function, then reset every handle before the next retry. */
typedef struct
{
    bool support;
    bool node;
    bool joint_fb_publisher;
    bool imu_raw_publisher;
    bool joint_cmd_subscription;
    bool executor;
} EntityLifecycle;

static EntityLifecycle entity_lifecycle;

static void ResetEntityHandles(void)
{
    support = (rclc_support_t){0};
    node = rcl_get_zero_initialized_node();
    executor = rclc_executor_get_zero_initialized_executor();
    joint_fb_pub = rcl_get_zero_initialized_publisher();
    imu_raw_pub = rcl_get_zero_initialized_publisher();
    joint_cmd_sub = rcl_get_zero_initialized_subscription();
    entity_lifecycle = (EntityLifecycle){0};
}

static void JointCmdCallback(const void *msgin)
{
    const main_bot_hardware_msgs__msg__JointCmd *cmd =
        (const main_bot_hardware_msgs__msg__JointCmd *)msgin;

    float angles_rad[JOINT_COUNT];
    float velocities_rad_s[JOINT_COUNT];
    float kp[JOINT_COUNT];
    float kd[JOINT_COUNT];
    float tau_ff_nm[JOINT_COUNT];
    for (uint32_t i = 0; i < JOINT_COUNT; i++)
    {
        angles_rad[i] = (float)cmd->target_angle_mrad[i] / 1000.0f;
        velocities_rad_s[i] = (float)cmd->target_velocity_mrad_s[i] / 1000.0f;
        kp[i] = (float)cmd->kp_x100[i] / 100.0f;
        kd[i] = (float)cmd->kd_x100[i] / 100.0f;
        tau_ff_nm[i] = (float)cmd->tau_ff_mnm[i] / 1000.0f;
    }

    Actuator_SetTarget(angles_rad, velocities_rad_s, kp, kd, tau_ff_nm);
    last_joint_cmd_ms = Tick_GetMs();
}

/* rclc_executor capacity=1 because publishers are not executor handles; the sole
 * handle is /joint_cmd. colcon.meta reserves 1 subscription + 2 publishers. */
static bool CreateEntities(void)
{
    ResetEntityHandles();
    allocator = rcl_get_default_allocator();

    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK)
    {
        return false;
    }
    entity_lifecycle.support = true;
    if (rclc_node_init_default(&node, "stm32_joint_node", "", &support) != RCL_RET_OK)
    {
        return false;
    }
    entity_lifecycle.node = true;
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
    entity_lifecycle.joint_fb_publisher = true;
    if (rclc_publisher_init_best_effort(
            &imu_raw_pub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(main_bot_hardware_msgs, msg, ImuRaw),
            "/imu/raw") != RCL_RET_OK)
    {
        return false;
    }
    entity_lifecycle.imu_raw_publisher = true;
    if (rclc_subscription_init_best_effort(
            &joint_cmd_sub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(main_bot_hardware_msgs, msg, JointCmd),
            "/joint_cmd") != RCL_RET_OK)
    {
        return false;
    }
    entity_lifecycle.joint_cmd_subscription = true;
    if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK)
    {
        return false;
    }
    entity_lifecycle.executor = true;
    if (rclc_executor_add_subscription(
            &executor, &joint_cmd_sub, &joint_cmd_msg, &JointCmdCallback, ON_NEW_DATA) != RCL_RET_OK)
    {
        return false;
    }
    return true;
}

static void DestroyEntities(void)
{
    rcl_ret_t cleanup_result = RCL_RET_OK;

    if (entity_lifecycle.support)
    {
        rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
        if (rmw_context != NULL)
        {
            (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
        }
    }

    if (entity_lifecycle.executor)
    {
        (void)rclc_executor_fini(&executor);
    }
    if (entity_lifecycle.joint_cmd_subscription)
    {
        cleanup_result = rcl_subscription_fini(&joint_cmd_sub, &node);
    }
    if (entity_lifecycle.imu_raw_publisher)
    {
        cleanup_result = rcl_publisher_fini(&imu_raw_pub, &node);
    }
    if (entity_lifecycle.joint_fb_publisher)
    {
        cleanup_result = rcl_publisher_fini(&joint_fb_pub, &node);
    }
    if (entity_lifecycle.node)
    {
        cleanup_result = rcl_node_fini(&node);
    }
    if (entity_lifecycle.support)
    {
        (void)rclc_support_fini(&support);
    }
    (void)cleanup_result;
    ResetEntityHandles();
}

void MicroRosBridge_Begin(void)
{
    main_bot_hardware_msgs__msg__ImuRaw__init(&imu_raw_msg);
    main_bot_hardware_msgs__msg__JointFb__init(&joint_fb_msg);
    main_bot_hardware_msgs__msg__JointCmd__init(&joint_cmd_msg);

    /* Khong goi allocator rieng nhu ban OUT_SAVE/testSTM (MicroRos_InstallFreeRTOSAllocator) -
     * khong FreeRTOS o day, rcutils's default allocator da bọc san malloc/free/realloc/
     * calloc cua newlib (hoat dong qua nosys.specs + symbol _end trong linker script,
     * xem README.md), dung thang duoc. */
    MicroRos_RegisterTransport();
    ResetEntityHandles();
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

    const rcl_ret_t publish_result = rcl_publish(&joint_fb_pub, &joint_fb_msg, NULL);
    (void)publish_result;
}

static int16_t scale_to_i16(float value, float scale)
{
    if (!isfinite(value))
    {
        return 0;
    }
    float scaled = value * scale;
    if (scaled > (float)INT16_MAX)
    {
        scaled = (float)INT16_MAX;
    }
    else if (scaled < (float)INT16_MIN)
    {
        scaled = (float)INT16_MIN;
    }
    return (int16_t)(scaled + ((scaled >= 0.0f) ? 0.5f : -0.5f));
}

void MicroRosBridge_PublishImuRaw(const MPU6050_Reading *reading,
                                  MicroRosImuStatus status,
                                  uint32_t stamp_ms)
{
    if (state != STATE_CONNECTED)
    {
        return;
    }

    for (uint32_t i = 0U; i < 3U; ++i)
    {
        const float acceleration = (reading != NULL) ? reading->linear_acceleration[i] : 0.0f;
        const float angular_velocity = (reading != NULL) ? reading->angular_velocity[i] : 0.0f;
        imu_raw_msg.linear_acceleration_milli_ms2[i] = scale_to_i16(acceleration, 1000.0f);
        imu_raw_msg.angular_velocity_mrad_s[i] = scale_to_i16(angular_velocity, 1000.0f);
    }
    imu_raw_msg.stamp_ms = stamp_ms;
    imu_raw_msg.status = (uint8_t)status;
    const rcl_ret_t publish_result = rcl_publish(&imu_raw_pub, &imu_raw_msg, NULL);
    (void)publish_result;
}
