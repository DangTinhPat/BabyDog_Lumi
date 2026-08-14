#include "stand_sit_fsm.h"
#include "actuator_if.h"
#include <math.h>

#define RAMP_DURATION_MS   1200.0f /* cùng 1.2s như stand_sit_controller (phía ROS2) */
#define SETTLED_AFTER_MS   1800U   /* elapsed_ms >= 1.5 * RAMP_DURATION_MS */

/* Cùng tư thế đứng/ngồi mặc định như controllers.yaml (phía ROS2) - xem ghi
 * chú ở đó về công thức hình học devq. */
static const float STAND_POS[12] = {
    0.0f, -0.748f, 1.495f,
    0.0f, -0.748f, 1.495f,
    0.0f, -0.748f, 1.495f,
    0.0f, -0.748f, 1.495f
};
static const float SIT_POS[12] = {
    0.0f, -1.231f, 2.462f,
    0.0f, -1.231f, 2.462f,
    0.0f, -1.231f, 2.462f,
    0.0f, -1.231f, 2.462f
};
#define STAND_KP 30.0f
#define STAND_KD 1.5f
#define SIT_KP   30.0f
#define SIT_KD   1.5f

static FsmState_t g_state = FSM_PASSIVE;
static float g_start_pos[12];
static uint32_t g_transition_start_ms = 0U;

static uint32_t g_last_cmd_rx_ms = 0U;
static int g_ever_received_cmd = 0;
static uint8_t g_last_seq = 0U;
static uint8_t g_fault_flags = 0U;
static int g_settled = 1;

static void enter_passive(void)
{
    g_state = FSM_PASSIVE;
    g_settled = 1;
    Actuator_Disable();
}

static void enter_transition(FsmState_t target_state, uint32_t now_ms)
{
    Actuator_GetLastTarget(g_start_pos);
    g_transition_start_ms = now_ms;
    g_state = target_state;
    g_settled = 0;
}

void FSM_Init(void)
{
    Actuator_Init();
    enter_passive();
}

void FSM_OnCommandReceived(uint8_t command, uint8_t seq, uint32_t now_ms)
{
    g_last_cmd_rx_ms = now_ms;
    g_ever_received_cmd = 1;
    g_last_seq = seq;

    /* ESTOP có hiệu lực ngay lập tức, kể cả đang nội suy. */
    if (command == CMD_ESTOP)
    {
        g_fault_flags |= FAULT_ESTOP_ACTIVE;
        enter_passive();
        return;
    }
    g_fault_flags &= (uint8_t)~(FAULT_ESTOP_ACTIVE | FAULT_LINK_TIMEOUT_RECOVERED);

    if (g_state != FSM_PASSIVE && !g_settled)
    {
        return; /* đang nội suy dở dang - chờ ổn định trước khi đổi hướng */
    }

    if (command == CMD_STAND && g_state != FSM_STAND)
    {
        enter_transition(FSM_STAND, now_ms);
    }
    else if (command == CMD_SIT && g_state != FSM_SIT)
    {
        enter_transition(FSM_SIT, now_ms);
    }
}

void FSM_Update(uint32_t now_ms)
{
    /* Mất kết nối RDK X5 <-> STM32: an toàn là tự ngắt lực, không giữ tư thế
     * mà không có ai giám sát (xem hợp đồng an toàn trong protocol.h). */
    if (g_ever_received_cmd && (now_ms - g_last_cmd_rx_ms) > LINK_TIMEOUT_MS)
    {
        if (g_state != FSM_PASSIVE)
        {
            g_fault_flags |= FAULT_LINK_TIMEOUT_RECOVERED;
            enter_passive();
        }
        return;
    }

    if (g_state == FSM_PASSIVE)
    {
        return;
    }

    const float *target_pos = (g_state == FSM_STAND) ? STAND_POS : SIT_POS;
    const float kp = (g_state == FSM_STAND) ? STAND_KP : SIT_KP;
    const float kd = (g_state == FSM_STAND) ? STAND_KD : SIT_KD;

    const uint32_t elapsed = now_ms - g_transition_start_ms;
    if (elapsed >= SETTLED_AFTER_MS)
    {
        g_settled = 1;
    }
    const float phase = tanhf((float)elapsed / RAMP_DURATION_MS);

    float target[12];
    for (int i = 0; i < 12; i++)
    {
        target[i] = phase * target_pos[i] + (1.0f - phase) * g_start_pos[i];
    }
    Actuator_SetTarget(target, kp, kd);
}

FsmState_t FSM_GetState(void)
{
    return g_state;
}

int FSM_IsSettled(void)
{
    return g_settled;
}

uint8_t FSM_GetFaultFlags(void)
{
    return g_fault_flags;
}

uint8_t FSM_GetLastSeq(void)
{
    return g_last_seq;
}
