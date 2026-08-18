#include "actuator_if.h"
#include "motor_topology.h"
#include "motor_calib.h"
#include "baby_alpha2_protocol.h"
#include "can.h"
#include "tick.h"
#include <string.h>
#include <stdbool.h>

#define BA2_PI_F        3.14159265358979323846f
#define BA2_DEG2RAD(d)  ((d) * (BA2_PI_F / 180.0f))
#define BA2_RAD2DEG(r)  ((r) * (180.0f / BA2_PI_F))

/* Vị trí ngồi/crouch làm giá trị mặc định TRƯỚC KHI Actuator_Init() hiệu
 * chuẩn xong (hoặc cho khớp hiệu chuẩn thất bại) - khớp với sit_pos trong
 * main_bot/config/controllers.yaml (phía ROS2). KHÔNG GIAN LOGIC (xem
 * motor_calib.h header). */
static float g_last_target[JOINT_COUNT] = {
    0.0f, -1.231f, 2.462f,
    0.0f, -1.231f, 2.462f,
    0.0f, -1.231f, 2.462f,
    0.0f, -1.231f, 2.462f
};
static float g_measured_angle[JOINT_COUNT];      /* khong gian LOGIC */
static float g_measured_velocity[JOINT_COUNT];   /* khong gian LOGIC */
static uint32_t g_last_telemetry_ms[JOINT_COUNT];
static int g_has_feedback[JOINT_COUNT];

/* true = khớp này hiệu chuẩn thành công lúc Actuator_Init() (PING+đọc HOME
 * đều OK) - Actuator_SetTarget() BỎ QUA hoàn toàn khớp hiệu chuẩn thất bại,
 * không gửi lệnh gì (an toàn hơn là gửi lệnh với giới hạn không đáng tin). */
static int g_joint_ok[JOINT_COUNT];

/* Giới hạn hành trình TUYỆT ĐỐI (rad, KHÔNG GIAN LOGIC), tính từ HOME đo
 * được lúc boot + độ lệch tương đối cố định (motor_calib.h) - dùng làm lớp
 * kẹp AN TOÀN THỨ HAI trong Actuator_SetTarget(), độc lập với SETUP_LIMITS
 * đã gửi cho driver (oneLeg/HW.md mục 8 "lớp an toàn": 2 lớp bảo vệ nhau). */
static float g_limit_max_rad[JOINT_COUNT];
static float g_limit_min_rad[JOINT_COUNT];

/* Offset hieu chuan ADAPTIVE (rad, khong gian LOGIC) - bu vao moi lan chuyen
 * doi RAW<->LOGIC de "diem 0 tuyet doi" cua encoder driver (TROI moi lan mat
 * dien - da xac nhan qua oneLeg/main_adaptive_confirmed_v2.c.bak) khop voi
 * gia dinh ASSUMED_REST_LOGICAL_RAD duoi day. Tinh lai MOI LAN boot trong
 * Actuator_Init() tu chinh HOME do duoc phien nay - KHONG dung bang hang so
 * co dinh (offset thuc te doi tung phien cap dien). Mac dinh 0 (memset) cho
 * khop chua/khong hieu chuan duoc. */
static float g_home_offset_rad[JOINT_COUNT];

/* Gia tri LOGIC gia dinh dung cho tu the HOME (nam xap luc cap nguon, xem
 * motor_calib.h) - chi so theo JointType_t (1=Hang,2=Dui,3=Goi), [0] khong
 * dung. Dat 0.0 cho ca 3 loai: URDF (babydog.xacro) HIEN DANG o nguyen ban
 * goc (chua hieu chinh origin/limit rieng cho tu the home那 - xem NOTE.md
 * muc 2.8, phan URDF con dang dang do), nen "0" o day chi co nghia "coi tu
 * the vat ly luc cap nguon la moc logic 0" - CHUA chac chan khop hinh dang
 * "nam xap" tren RViz cho toi khi URDF cung duoc hieu chinh tuong ung. Neu
 * sau nay do duoc gia tri khac dung hon, chi sua mang nay. */
static const float ASSUMED_REST_LOGICAL_RAD[4] = {0.0f, 0.0f, 0.0f, 0.0f};

/* ===== Chuyen doi khong gian LOGIC <-> RAW (xem motor_calib.h header) =====
 * Diem DUY NHAT trong toan bo firmware biet ve MOTOR_JOINT_SIGN - moi noi
 * khac (gioi han, HOME, g_last_target/g_measured_*) deu lam viec thuan tuy
 * trong khong gian LOGIC (DA bao gom offset hieu chuan, xem g_home_offset_rad
 * o tren). */
static float LogicalToRaw(JointIndex_t joint, float logical_rad)
{
    const int sign = MOTOR_JOINT_SIGN[Motor_LegGroupForJoint(joint)][Motor_JointTypeForJoint(joint)];
    return (float)sign * (logical_rad - g_home_offset_rad[joint]);
}

static float RawToLogical(JointIndex_t joint, float raw_rad)
{
    /* sign = +-1 nen dao nguoc dung la nhan lai chinh no (sign*sign=1). */
    const int sign = MOTOR_JOINT_SIGN[Motor_LegGroupForJoint(joint)][Motor_JointTypeForJoint(joint)];
    return (float)sign * raw_rad + g_home_offset_rad[joint];
}

/* Torque khong co offset: chi doi dau theo cung mapping lap guong cua goc.
 * Neu lenh vi tri logic duong can torque logic duong de ho tro no, ca hai
 * phai toi motor voi cung phep doi dau. */
static float LogicalTorqueToRaw(JointIndex_t joint, float logical_nm)
{
    const int sign = MOTOR_JOINT_SIGN[Motor_LegGroupForJoint(joint)][Motor_JointTypeForJoint(joint)];
    return (float)sign * logical_nm;
}

/* Velocity la dao ham cua position: cung doi dau lap guong, khong bao gio
 * cong home offset. Tach ham de khong vo tinh dung LogicalToRaw() (ham do co
 * offset vi tri) cho v_des. */
static float LogicalVelocityToRaw(JointIndex_t joint, float logical_rad_s)
{
    const int sign = MOTOR_JOINT_SIGN[Motor_LegGroupForJoint(joint)][Motor_JointTypeForJoint(joint)];
    return (float)sign * logical_rad_s;
}

static bool can_tx(uint32_t instance, const CAN_Frame *f)
{
    for (uint32_t i = 0U; i < 200U; i++)
    {
        if (CAN_Transmit(instance, f)) { return true; }
    }
    return false;
}

typedef bool (*MatchFn)(const CAN_Frame *rx, uint32_t id);
static bool match_ping(const CAN_Frame *rx, uint32_t id)  { return rx->data_len >= 1U && rx->data[0] == BA2_REPLY_PING(id); }
static bool match_hs(const CAN_Frame *rx, uint32_t id)    { return rx->data_len >= 1U && rx->data[0] == BA2_REPLY_HS(id); }
static bool match_setup(const CAN_Frame *rx, uint32_t id) { return rx->data_len >= 1U && rx->data[0] == BA2_REPLY_SETUP(id); }
static bool match_page0(const CAN_Frame *rx, uint32_t id) { return rx->data_len >= 12U && rx->data[0] == BA2_REPLY_PAGE0(id); }

/* Gửi rồi chờ đúng phản hồi mong đợi (retry nếu timeout) - MỌI phản hồi
 * BabyAlpha2 về CAN ID=0 (xem motor_topology.h header), phân biệt bằng
 * data[0] qua tham số m/id. CHỈ dùng trong Actuator_Init() (trước khi
 * main.c vào vòng lặp chính) - đây là nơi DUY NHẤT khác main.c tự gọi
 * CAN_Receive(), an toàn vì không có ai khác đọc FIFO cùng lúc lúc boot. */
static bool send_wait(uint32_t instance, const CAN_Frame *f, MatchFn m, uint32_t id,
                       uint32_t timeout_ms, uint32_t retries)
{
    for (uint32_t a = 0U; a <= retries; a++)
    {
        (void)can_tx(instance, f);
        const uint32_t t0 = Tick_GetMs();
        while ((Tick_GetMs() - t0) < timeout_ms)
        {
            CAN_Frame rx;
            while (CAN_IsRxPending(instance) && CAN_Receive(instance, &rx))
            {
                if (rx.id == 0U && m(&rx, id)) { return true; }
            }
        }
    }
    return false;
}

/* Nhắc MOTOR_ENABLE cho MỌI khớp đã PING thành công TRỪ khớp đang xử lý -
 * dùng giữa các bước hiệu chuẩn TUẦN TỰ để không khớp nào bị "bỏ đói" quá
 * lâu trong lúc khớp khác đang hiệu chuẩn (driver tự rơi Standby nếu thiếu
 * nhắc định kỳ) - 12 khớp nên chuỗi hiệu chuẩn dài hơn nhiều so với 1 chân
 * (oneLeg/HW.md mục 8). */
static void keepalive_others(JointIndex_t skip_joint)
{
    for (int j = 0; j < (int)JOINT_COUNT; j++)
    {
        if ((JointIndex_t)j == skip_joint || !g_joint_ok[j]) { continue; }
        const JointIndex_t joint = (JointIndex_t)j;
        const CAN_Frame en = BA2_BuildSystemFrame(Motor_IdForJoint(joint), BA2_OPCODE_MOTOR_ENABLE);
        (void)can_tx(Motor_BusForJoint(joint), &en);
    }
}

/* Nhac lai MOTOR_ENABLE cho MOI khop da PING OK - PHAI goi dinh ky TRONG SUOT
 * luc chay (khong chi luc boot). Truoc day chi goi keepalive_others() ben
 * trong Actuator_Init() (chi luc hieu chuan), main.c's vong lap chinh KHONG
 * bao gio goi lai - driver BabyAlpha2 tu roi Standby (mat luc, bo qua lenh
 * PD) sau 1 khoang khong duoc nhac, dung sau khi Init() xong la het nhac -
 * gay hien tuong dong co "keu lach cach" (driver co gang xu ly frame nhung
 * khong o trang thai Enable de sinh momen) du PD van gui deu qua /joint_cmd.
 * Xac nhan qua oneLeg/main_12joint_hold_proven.c.bak (da chay that): ho nhac
 * lai moi RE_ENABLE_PERIOD_MS=500ms trong main loop, khong chi luc Init().
 * main.c goi ham nay voi cung chu ky 500ms. */
void Actuator_ReenableAll(void)
{
    for (int j = 0; j < (int)JOINT_COUNT; j++)
    {
        if (!g_joint_ok[j]) { continue; }
        const JointIndex_t joint = (JointIndex_t)j;
        const CAN_Frame en = BA2_BuildSystemFrame(Motor_IdForJoint(joint), BA2_OPCODE_MOTOR_ENABLE);
        (void)can_tx(Motor_BusForJoint(joint), &en);
    }
}

/* PING+HANDSHAKE+MOTOR_ENABLE - CHƯA gửi SETUP_LIMITS (chưa biết HOME) -
 * an toàn vì chưa gửi Kp/Kd khác 0 (oneLeg/HW.md mục 5). KHÔNG gửi
 * CALIB_ENCODER (0xF4, xem baby_alpha2_protocol.h) - tài liệu chính thức
 * liệt kê bước này trong chuỗi khởi tạo 5 bước, nhưng công thức tính offset
 * (Byte 1-2) chưa được xác nhận rõ ràng trên phần cứng của dự án này; thay
 * vào đó dùng cách hiệu chuẩn thích ứng (đọc HOME + giới hạn tương đối) đã
 * kiểm chứng qua oneLeg, không cần hiểu offset đó là gì. */
static bool ping_handshake_enable(JointIndex_t joint)
{
    const uint32_t instance = Motor_BusForJoint(joint);
    const uint32_t id = Motor_IdForJoint(joint);

    CAN_Frame f = BA2_BuildSystemFrame(id, BA2_OPCODE_PING);
    const bool ok = send_wait(instance, &f, match_ping, id, 300U, 4U);

    f = BA2_BuildHandshakeFrame(id);
    (void)send_wait(instance, &f, match_hs, id, 300U, 4U);

    f = BA2_BuildSystemFrame(id, BA2_OPCODE_MOTOR_ENABLE);
    (void)can_tx(instance, &f);
    return ok;
}

/* Đọc vị trí thực bằng khung PD Kp=Kd=tau=0 (động cơ KHÔNG sinh lực trong
 * lúc đọc) - dùng làm HOME của phiên khởi động này (khong gian RAW, chuyen
 * sang LOGIC ngay truoc khi tra ve). */
static bool read_home_logical(JointIndex_t joint, float *logical_rad)
{
    const uint32_t instance = Motor_BusForJoint(joint);
    const uint32_t id = Motor_IdForJoint(joint);
    for (uint32_t a = 0U; a < 20U; a++)
    {
        const CAN_Frame probe = BA2_BuildPdFrame(
            id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, MOTOR_TAU_ABS_LIMIT_NM);
        (void)can_tx(instance, &probe);
        const uint32_t t0 = Tick_GetMs();
        while ((Tick_GetMs() - t0) < 20U)
        {
            CAN_Frame rx;
            while (CAN_IsRxPending(instance) && CAN_Receive(instance, &rx))
            {
                if (rx.id == 0U && match_page0(&rx, id))
                {
                    const float raw_rad = BA2_DecodePosAct(((uint16_t)rx.data[1] << 8) | rx.data[2]);
                    *logical_rad = RawToLogical(joint, raw_rad);
                    return true;
                }
            }
        }
    }
    return false;
}

/* limit_max/min_logical: khong gian LOGIC (g_limit_max_rad/min_rad). Chuyen
 * sang RAW qua LogicalToRaw() truoc khi ma hoa - neu dau la -1, phep nhan
 * dao ca thu tu max/min nen phai tu sap xep lai (khong duoc gia dinh
 * raw_max > raw_min). */
static bool send_setup_limits(JointIndex_t joint, float limit_max_logical, float limit_min_logical)
{
    const uint32_t instance = Motor_BusForJoint(joint);
    const uint32_t id = Motor_IdForJoint(joint);

    const float raw_a = LogicalToRaw(joint, limit_max_logical);
    const float raw_b = LogicalToRaw(joint, limit_min_logical);
    const float raw_max = (raw_a > raw_b) ? raw_a : raw_b;
    const float raw_min = (raw_a > raw_b) ? raw_b : raw_a;

    const uint16_t max_raw = BA2_EncodePosAct(raw_max);
    const uint16_t min_raw = BA2_EncodePosAct(raw_min);
    const CAN_Frame f = BA2_BuildSetupLimitsFrame(id, max_raw, min_raw, MOTOR_VMAX_RAW);
    return send_wait(instance, &f, match_setup, id, 300U, 4U);
}

void Actuator_Init(void)
{
    memset(g_measured_angle, 0, sizeof(g_measured_angle));
    memset(g_measured_velocity, 0, sizeof(g_measured_velocity));
    memset(g_last_telemetry_ms, 0, sizeof(g_last_telemetry_ms));
    memset(g_has_feedback, 0, sizeof(g_has_feedback));
    memset(g_joint_ok, 0, sizeof(g_joint_ok));
    memset(g_limit_max_rad, 0, sizeof(g_limit_max_rad));
    memset(g_limit_min_rad, 0, sizeof(g_limit_min_rad));
    memset(g_home_offset_rad, 0, sizeof(g_home_offset_rad));

    /* Buoc 1: PING+HANDSHAKE+ENABLE tuan tu 12 khop (keepalive giua cac
     * buoc de khong khop nao bi bo doi qua lau). */
    for (int j = 0; j < (int)JOINT_COUNT; j++)
    {
        g_joint_ok[j] = ping_handshake_enable((JointIndex_t)j) ? 1 : 0;
        keepalive_others((JointIndex_t)j);
    }

    /* Buoc 2: doc HOME (Kp=Kd=tau=0, khong gian LOGIC) cho tung khop da PING
     * OK - GIA DINH robot dang nam xap (tu the nghi tu nhien) luc cap nguon,
     * nguoi van hanh PHAI dam bao dieu nay truoc khi bat nguon (xem
     * motor_calib.h). Tinh gioi han LOGIC tuyet doi = HOME + do lech tuong
     * doi (motor_calib.h, KHONG can mirror rieng - da chuyen het sang
     * LogicalToRaw() luc gui SETUP_LIMITS), roi moi gui SETUP_LIMITS. Khop
     * nao khong doc duoc HOME -> danh dau that bai, Actuator_SetTarget() se
     * bo qua hoan toan khop do. */
    for (int j = 0; j < (int)JOINT_COUNT; j++)
    {
        if (!g_joint_ok[j]) { continue; }
        const JointIndex_t joint = (JointIndex_t)j;

        float home_logical_rad = 0.0f;
        const bool got_home = read_home_logical(joint, &home_logical_rad);
        keepalive_others(joint);
        if (!got_home)
        {
            g_joint_ok[j] = 0;
            continue;
        }

        const JointType_t jt = Motor_JointTypeForJoint(joint);

        /* Hieu chuan ADAPTIVE: home_logical_rad o day la gia tri CHUA co
         * offset (g_home_offset_rad[j] van = 0 tu memset, xem RawToLogical())
         * - tinh offset SAO CHO sau khi cong vao, HOME phien nay tro thanh
         * dung ASSUMED_REST_LOGICAL_RAD[jt]. Moi lan doc/gui RAW<->LOGIC SAU
         * DIEM NAY (limit, target, telemetry) deu tu dong di qua offset nay
         * qua LogicalToRaw()/RawToLogical(). */
        const float assumed_rest = ASSUMED_REST_LOGICAL_RAD[jt];
        g_home_offset_rad[j] = assumed_rest - home_logical_rad;

        const float home_deg = BA2_RAD2DEG(assumed_rest);   /* = HOME DA hieu chuan, hang so */
        g_limit_max_rad[j] = BA2_DEG2RAD(home_deg + MOTOR_LIMIT_MAX_REL_DEG[jt]);
        g_limit_min_rad[j] = BA2_DEG2RAD(home_deg + MOTOR_LIMIT_MIN_REL_DEG[jt]);

        /* Diem bat dau noi suy = chinh HOME vua do (DA hieu chuan = assumed_rest)
         * - lenh giu dau tien khong giat (sai so ban dau = 0, oneLeg/HW.md muc 8). */
        g_last_target[j] = assumed_rest;
        g_measured_angle[j] = assumed_rest;
        g_has_feedback[j] = 1;
        g_last_telemetry_ms[j] = Tick_GetMs();

        (void)send_setup_limits(joint, g_limit_max_rad[j], g_limit_min_rad[j]);
        keepalive_others(joint);
    }
}

void Actuator_OnBabyAlpha2Frame(uint32_t instance, const CAN_Frame *frame)
{
    /* MOI phan hoi BabyAlpha2 (PING/HANDSHAKE/SETUP/telemetry Page0) ve
     * chung CAN ID=0 - o day chi quan tam telemetry Page0 (data[0]=1..6,
     * du dai >=12 byte); cac reply he thong khac da duoc send_wait() trong
     * Actuator_Init() xu ly rieng (khong can main.c dispatch toi day). */
    if (frame->id != 0U || frame->data_len < 12U) { return; }
    const uint8_t b0 = frame->data[0];
    if (b0 < 1U || b0 > 6U) { return; }

    const uint32_t bus_index = (instance == CAN_INSTANCE_2) ? 1U : 0U;
    const uint32_t slot = (bus_index * 6U) + (uint32_t)(b0 - 1U);
    if (slot >= JOINT_COUNT) { return; }
    const JointIndex_t joint = (JointIndex_t)slot;

    const float raw_rad = BA2_DecodePosAct(((uint16_t)frame->data[1] << 8) | frame->data[2]);
    const float angle_logical_rad = RawToLogical(joint, raw_rad);
    const uint32_t now_ms = Tick_GetMs();

    /* BabyAlpha2's telemetry Page0 khong bao gom van toc (chi position, xem
     * baby_alpha2_protocol.h) - tu uoc luong bang sai phan luot (backward
     * difference) qua 2 lan telemetry lien tiep TRONG khong gian LOGIC (da
     * chuyen doi dau o tren, nen hieu 2 lan doc lien tiep tu dong dung dau,
     * khong can xu ly rieng) - KHONG phai gia tri driver bao ve, du chinh
     * xac cho muc dich bao cao state len ROS2 (Actuator_GetMeasured()),
     * khong dung de dieu khien PD (PD chay hoan toan tren driver). */
    if (g_has_feedback[slot])
    {
        const uint32_t dt_ms = now_ms - g_last_telemetry_ms[slot];
        if (dt_ms > 0U)
        {
            g_measured_velocity[slot] = (angle_logical_rad - g_measured_angle[slot]) / ((float)dt_ms / 1000.0f);
        }
    }
    g_measured_angle[slot] = angle_logical_rad;
    g_last_telemetry_ms[slot] = now_ms;
    g_has_feedback[slot] = 1;
}

void Actuator_SetTarget(const float angles_rad[12], const float velocities_rad_s[12],
                        const float kp[12],
                        const float kd[12], const float tau_ff_nm[12])
{
    /* Kep velocity/KP/KD/Tff LOP 2 - doc lap voi phia ROS2/EC (xem
     * StandSitController.h). BA2_EncodeGainX100() chi kep san = 0, chua co
     * tran - kep o day truoc, phong gia tri loi (YAML sai don vi, hoac
     * kp_x100/kd_x100 tu /joint_cmd bi loi) truyen thang xuong dong co that. */
    for (int j = 0; j < (int)JOINT_COUNT; j++)
    {
        /* Khop hieu chuan that bai luc Actuator_Init() -> KHONG gui lenh gi
         * ca (an toan hon la gui voi gioi han khong dang tin). */
        if (!g_joint_ok[j]) { continue; }
        const JointIndex_t joint = (JointIndex_t)j;

        float joint_kp = kp[j];
        float joint_kd = kd[j];
        float joint_velocity = velocities_rad_s[j];
        float joint_tau_ff = tau_ff_nm[j];
        if (joint_velocity > MOTOR_VELOCITY_ABS_LIMIT_RAD_S[j])
        {
            joint_velocity = MOTOR_VELOCITY_ABS_LIMIT_RAD_S[j];
        }
        if (joint_velocity < -MOTOR_VELOCITY_ABS_LIMIT_RAD_S[j])
        {
            joint_velocity = -MOTOR_VELOCITY_ABS_LIMIT_RAD_S[j];
        }
        if (joint_kp < 0.0f) { joint_kp = 0.0f; }
        if (joint_kp > MOTOR_KP_ABS_LIMIT) { joint_kp = MOTOR_KP_ABS_LIMIT; }
        if (joint_kd < 0.0f) { joint_kd = 0.0f; }
        if (joint_kd > MOTOR_KD_ABS_LIMIT) { joint_kd = MOTOR_KD_ABS_LIMIT; }
        if (joint_tau_ff > MOTOR_TAU_ABS_LIMIT_NM) { joint_tau_ff = MOTOR_TAU_ABS_LIMIT_NM; }
        if (joint_tau_ff < -MOTOR_TAU_ABS_LIMIT_NM) { joint_tau_ff = -MOTOR_TAU_ABS_LIMIT_NM; }

        /* Kep gioi han hanh trinh LOP 2 TRONG KHONG GIAN LOGIC - doc lap voi
         * SETUP_LIMITS da gui driver (oneLeg/HW.md muc 8, 2 lop bao ve
         * nhau). */
        float target_logical = angles_rad[j];
        if (target_logical > g_limit_max_rad[j]) { target_logical = g_limit_max_rad[j]; }
        if (target_logical < g_limit_min_rad[j]) { target_logical = g_limit_min_rad[j]; }
        /* Neu target dang o bien, khong cho D-term tiep tuc yeu cau van toc
         * huong RA NGOAI hanh trinh. Van toc huong vao trong van duoc giu de
         * khop co the roi khoi bien mot cach binh thuong. */
        if ((target_logical >= g_limit_max_rad[j] && joint_velocity > 0.0f) ||
            (target_logical <= g_limit_min_rad[j] && joint_velocity < 0.0f))
        {
            joint_velocity = 0.0f;
        }
        g_last_target[j] = target_logical;

        /* Chuyen position/velocity/torque sang khong gian RAW NGAY TRUOC KHI
         * ma hoa. Position co home offset; velocity va torque chi doi dau,
         * tuyet doi khong duoc cong offset goc vao hai dai luong dao ham. */
        const float target_raw = LogicalToRaw(joint, target_logical);
        const float velocity_raw = LogicalVelocityToRaw(joint, joint_velocity);
        const float tau_ff_raw = LogicalTorqueToRaw(joint, joint_tau_ff);
        const CAN_Frame frame = BA2_BuildPdFrame(
            Motor_IdForJoint(joint), target_raw, velocity_raw, joint_kp, joint_kd,
            tau_ff_raw, MOTOR_TAU_ABS_LIMIT_NM);
        (void)CAN_Transmit(Motor_BusForJoint(joint), &frame);
    }
}

void Actuator_Disable(void)
{
    /* kp=0/kd nhỏ -> board driver mỗi khớp tự thả lỏng (giống StatePassive
     * phía ROS2: kp=0, kd=1.0) - target góc không còn ý nghĩa khi kp=0, dùng
     * lại vị trí đo được gần nhất cho gọn. */
    float target[JOINT_COUNT];
    float velocity[JOINT_COUNT];
    float kp[JOINT_COUNT];
    float kd[JOINT_COUNT];
    float tau_ff[JOINT_COUNT];
    Actuator_GetLastTarget(target);
    for (int j = 0; j < (int)JOINT_COUNT; ++j)
    {
        velocity[j] = 0.0f;
        kp[j] = 0.0f;
        kd[j] = 1.0f;
        tau_ff[j] = 0.0f;
    }
    Actuator_SetTarget(target, velocity, kp, kd, tau_ff);
}

void Actuator_GetLastTarget(float angles_rad[12])
{
    for (int j = 0; j < (int)JOINT_COUNT; j++)
    {
        angles_rad[j] = g_has_feedback[j] ? g_measured_angle[j] : g_last_target[j];
    }
}

void Actuator_GetMeasured(float angles_rad[12], float velocities_rad_s[12])
{
    for (int j = 0; j < (int)JOINT_COUNT; j++)
    {
        angles_rad[j] = g_has_feedback[j] ? g_measured_angle[j] : 0.0f;
        velocities_rad_s[j] = g_has_feedback[j] ? g_measured_velocity[j] : 0.0f;
    }
}

int Actuator_IsJointOk(int joint)
{
    if (joint < 0 || joint >= (int)JOINT_COUNT) { return 0; }
    return g_joint_ok[joint];
}
