# NOTE.md — Nhật ký debug phần cứng thật (babyDog)

Ghi lại các bug đã gặp trên **phần cứng thật** (không phải sim), nguyên nhân gốc, cách fix, và các phát hiện về hệ thống trong quá trình bring-up. Cập nhật dần mỗi lần có phiên debug mới.

---

## 1. Kiến trúc hệ thống (tóm tắt)

```
PC/RDK X5 (ROS2 Jazzy)
   │  micro-ROS qua UART1 (/dev/ttyUSB0, 921600 baud, PA9=TX/PA10=RX trên MCU)
   │  topic: /joint_cmd (xuống), /joint_fb (lên)
   ▼
STM32H7 (FK743M5-XIH6, Cortex-M7 480MHz)
   │  CAN-FD 12 byte, protocol BabyAlpha2 (baby_alpha2_protocol.h)
   │  2 bus độc lập:
   ├─ CAN_INSTANCE_1 (FDCAN1, PA11=RX/PA12=TX, AF9) → 6 driver CHÂN TRƯỚC (ID 1-6)
   └─ CAN_INSTANCE_2 (FDCAN2, PB5=RX/PB6=TX, AF9)  → 6 driver CHÂN SAU  (ID 1-6)
   ▼
12× board driver "BabyAlpha2" (PD cục bộ, tự chạy kp/kd do MCU gửi xuống)
```

**Ánh xạ joint → bus/ID** (`motor_protocol.h`, đã đối chiếu khớp tài liệu hãng https://github.com/DungTranBK/BabyAlpha2_Docs):

| JointIndex | Tên | Bus | CAN ID |
|---|---|---|---|
| 0,1,2 | FRONT_RIGHT abad/hip/knee | CAN_INSTANCE_1 | 1,2,3 |
| 3,4,5 | FRONT_LEFT abad/hip/knee | CAN_INSTANCE_1 | 4,5,6 |
| 6,7,8 | HIND_RIGHT abad/hip/knee | CAN_INSTANCE_2 | 1,2,3 |
| 9,10,11 | HIND_LEFT abad/hip/knee | CAN_INSTANCE_2 | 4,5,6 |

**URDF (`babydog.xacro`) — góc khớp = 0 nghĩa là gì**: `hip_joint`/`knee_joint`/`foot_joint` có origin nối tiếp thẳng hàng theo trục -Z, KHÔNG có góc bù nào trong `<origin rpy>` — nghĩa là khi `hip=0` và `knee=0`, cả đùi-ống-chân xếp thẳng 1 đường = tư thế **"duỗi thẳng đơ"** theo đúng nghĩa hình học. Ngoài ra `knee_joint` có `<limit lower="0.436">` — tức `knee=0` còn NẰM NGOÀI vùng chuyển động cơ khí thật (RViz vẫn vẽ vì không kiểm tra `<limit>` lúc hiển thị). Vậy tư thế mặc định lúc chưa có `/joint_states` (hoặc lúc góc logic đúng bằng 0) LUÔN trông "duỗi" — không phải bug, là hệ quả tất yếu của cách đặt gốc tọa độ khớp. Xem thêm mục 2.6.

Công thức: `bus = joint<6 ? CAN_INSTANCE_1 : CAN_INSTANCE_2`, `id = (joint%6)+1`.

**Driver BabyAlpha2** (theo tài liệu hãng):
- Cần nguồn ổn định **24V–28V** để vào trạng thái sẵn sàng nhận PING.
- CAN ID **cố định theo firmware driver**: lệnh gửi tới ID 0x001–0x006, MỌI phản hồi (PING/HANDSHAKE/SETUP/telemetry) đều về chung ID 0x000, phân biệt bằng byte `data[0]`.
- Không có DIP switch/cấu hình ID nào khác.

**Package/thư mục liên quan**:
- `firmware/stm32h7/` — firmware MCU (Makefile riêng, `make` / `make flash`).
- `src/main_bot_hardware/` — `RealSystem` (ros2_control hardware_interface, cầu nối `/joint_fb`↔state interfaces, `/joint_cmd`↔command interfaces).
- `src/controller/` (trước đây `stand_sit_controller`) — FSM Passive/HoldPose (Stand/Sit), controller ros2_control.
- `src/main_bot/launch/real_ros2_control.launch.py` — launch chính cho phần cứng thật (ép `ROS_DOMAIN_ID=0`).
- `/home/dvt/OUT_SAVE/babyDog_test/oneLeg/` — project tham khảo (đã test thật, có snapshot `leg1_proven_2026-08-08/` và nhiều bản `main_4leg_*.bak`/`main_12joint_hold_proven.c.bak` đã test cả 4 chân) — dùng để đối chiếu mỗi khi nghi ngờ firmware.

---

## 2. Bug đã tìm và fix (firmware MCU)

### 2.1. `CAN_Start()` không xác nhận thật sự thoát khỏi Init mode
- **Triệu chứng**: CAN không bao giờ gửi được frame nào, không lỗi rõ ràng.
- **Nguyên nhân**: chỉ ghi `CCCR.INIT=0` một lần rồi tin là xong, không đọc lại xác nhận. Trên phần cứng thật, `CCCR` đọc lại đôi khi vẫn còn `INIT=1`.
- **Fix**: `CAN_Start()` đổi từ `void`→`bool`, clear `CCCR_INIT_Msk|CCCR_CCE_Msk` cùng lúc, poll xác nhận với timeout trước khi trả `true`.
- File: `firmware/stm32h7/lib/src/can.c`.

### 2.2. Sai lệch bit-timing pha Data (DBTP)
- **Triệu chứng**: PING không có phản hồi dù CAN_Start() đã fix.
- **Nguyên nhân**: đặt data-phase = nominal (1Mbit/s) với giả định "BRS=false nên DBTP không ảnh hưởng" — SAI theo Bosch M_CAN (cơ chế fixed-stuff-bit của CRC FD phụ thuộc cấu hình data-phase kể cả khi BRS=0 từng frame).
- **Fix**: đổi `data_tseg1=3, data_tseg2=1, data_sjw=1` (5Mbit/s), khớp chính xác oneLeg đã test thật.
- File: `firmware/stm32h7/main.c` (`CAN_BUS_CONFIG`).

### 2.3. Tự gây regression: treo `while(1)` khi `CAN_Start()` thất bại
- Thêm bẫy treo (giống `CAN_Init`) khiến TOÀN BỘ micro-ROS không bao giờ khởi động nếu 1 bus CAN lỗi — làm mất khả năng debug từ xa qua `/joint_fb`.
- **Fix**: gọi `(void)CAN_Start(...)` không chặn, để hệ thống tiếp tục boot dù CAN lỗi (an toàn: khớp lỗi tự báo TIMEOUT qua ROS2, không khớp nào bị gửi lệnh sai).

### 2.4. Mất ổn định kết nối micro-ROS ("chập chờn", session cứ re-establish)
- **Triệu chứng**: `/stm32_joint_node` không bao giờ xuất hiện đầy đủ dù agent log báo "session established" rồi lại "session re-established" lặp lại mỗi ~10s.
- **Chẩn đoán**: đọc source `wait_session_status()` (Micro-XRCE-DDS-Client) + `SessionManager::establish_session()` (micro_ros_agent) — xác nhận "re-established" nghĩa là CLIENT (MCU) tự gửi lại yêu cầu tạo session vì KHÔNG nhận được ACK của agent trong thời gian chờ, không phải do teardown/reconnect vật lý.
- **Nguyên nhân**: `Transport_Write()` dùng TX ring buffer (bất đồng bộ qua ngắt TXE) — thư viện XRCE gọi `write()` rồi LẬP TỨC bắt đầu đếm giờ chờ phản hồi, KHÔNG đợi xác nhận byte đã thực sự rời khỏi UART.
- **Fix tạm thời (đang áp dụng)**: `microros_transport.c` có `#define TX_BLOCKING_POLLING_TEST 1` — chuyển `Transport_Write()` về blocking polling (`USART_SendByte()` từng byte) thay vì ring buffer. Đã xác nhận: session established ổn định, `/stm32_joint_node` xuất hiện đầy đủ, TF cập nhật đúng, 0 lần re-establish qua nhiều lần test 30-40s.
- **Việc còn dang dở**: `tx_ring_enqueue()` (code ring buffer TX gốc) vẫn còn trong file, hiện không dùng (cảnh báo `-Wunused-function`). Cần quyết định: giữ blocking polling vĩnh viễn (đơn giản, đã proven) hay quay lại tối ưu ring buffer với cơ chế đợi xác nhận đúng cách.
- File: `firmware/stm32h7/app/src/microros_transport.c`.
- **Đã LOẠI TRỪ**: tầng thanh ghi/GPIO USART1 (test polling thuần, 80.000 byte, 0 lỗi) và tầng ISR+ring buffer RX (test qua đúng `Transport_Open()`/`Transport_Read()` production, 80.000 byte, 0 lỗi) — cả 2 đều hoàn toàn sạch, không phải nguyên nhân.

### 2.5. `make flash` không tự reset chip sau khi ghi
- **Triệu chứng**: nhiều lần flash firmware mới nhưng board vẫn chạy hành vi CŨ — gây nhầm lẫn "code sửa rồi mà vẫn lỗi y hệt" trong nhiều giờ debug.
- **Nguyên nhân**: board không nối chân NRST tới ST-Link, `st-flash write` (không có cờ `--reset`) không tự khởi động lại core sau khi ghi flash — CPU có thể bị treo/crash (do flash bị ghi đè ngay dưới chân đang chạy) cho tới khi có reset thủ công.
- **Fix**: `Makefile`'s target `flash` đổi thành `st-flash --reset write ...`.
- File: `firmware/stm32h7/Makefile`.

### ⭐ 2.6+2.8. [ĐÃ FIX VĨNH VIỄN - ĐIỀU CỐ HỮU QUAN TRỌNG NHẤT] Hiệu chuẩn "góc 0 = tư thế nằm xấp" — CẦN CẢ 2 LỚP (firmware + URDF), thiếu 1 trong 2 là sai

**Đây là điểm quan trọng nhất cần nhớ khi debug bất kỳ vấn đề hiển thị/tư thế nào sau này** - đã từng bị revert đi revert lại nhiều lần trong đêm nay vì tưởng chỉ cần 1 lớp, KHÔNG PHẢI:

| | Trước khi fix | Sau khi fix (hiện tại, đang chạy) |
|---|---|---|
| **Firmware** (`actuator_if.c`) | `RawToLogical`/`LogicalToRaw` chỉ đổi dấu (±1), KHÔNG bù offset - giá trị báo về = encoder thô, TRÔI ngẫu nhiên mỗi lần cấp điện | Có `g_home_offset_rad[12]` + `ASSUMED_REST_LOGICAL_RAD=0.0` - mỗi lần boot tự bù để giá trị báo về LUÔN đúng **0.000** bất kể encoder trôi tới đâu |
| **URDF** (`babydog.xacro`) | `origin rpy="0 0 0"` (nguyên bản) - góc khớp = 0 → hình học vẽ ra tư thế **"duỗi thẳng đơ"** (do các link xếp thẳng hàng khi origin=0) | `origin rpy` đã xoay lại (abad=±0.360, hip=1.238, knee=-2.705, `<limit>` dịch tương ứng) - góc khớp = 0 → vẽ ra đúng **tư thế nằm xấp thật** |
| **Kết quả kết hợp** | fw báo 0 (hoặc số ngẫu nhiên) + URDF hiểu "0=duỗi" → RViz **luôn hiện duỗi**, sai | fw LUÔN báo đúng 0 lúc boot + URDF hiểu "0=nằm xấp" → RViz **luôn hiện đúng nằm xấp** lúc kết nối, ổn định mọi phiên cấp điện |

**Tại sao cần CẢ 2, thiếu 1 là sai**:
- Chỉ sửa firmware (fw báo đúng 0 ổn định) mà KHÔNG sửa URDF → 0 vẫn bị URDF vẽ thành "duỗi" (đã xảy ra thật, gây hoang mang tưởng có bug mới).
- Chỉ sửa URDF (0=nằm xấp) mà KHÔNG sửa firmware → giá trị báo về vẫn trôi ngẫu nhiên mỗi phiên, "0" thực tế hiếm khi đúng là 0, tư thế vẽ ra vẫn sai hầu hết các lần.
- **Phải sửa cả 2 cùng lúc** thì "góc 0" mới vừa ỔN ĐỊNH (nhờ fw) vừa CÓ Ý NGHĨA ĐÚNG (nhờ URDF).

**Trạng thái hiện tại (2026-08-13, đang chạy trên board thật + trong xacro)**: CẢ 2 lớp đều ĐANG BẬT. Nếu sau này lại thấy "duỗi" quay lại, kiểm tra NGAY 2 chỗ này trước tiên (có thể 1 trong 2 bị revert nhầm):
1. `firmware/stm32h7/app/src/actuator_if.c`: `g_home_offset_rad`/`ASSUMED_REST_LOGICAL_RAD` còn tồn tại không, `LogicalToRaw`/`RawToLogical` có trừ/cộng offset không.
2. `src/main_bot/description/babydog.xacro`: `origin rpy` của 12 khớp revolute có đang là `±0.360`/`1.238`/`-2.705` (không phải `0 0 0`) không.

---

### 2.6. [phần chi tiết] Thiếu hiệu chuẩn điểm 0 (home offset) giữa encoder driver và URDF
- **Triệu chứng**: cầm tay xoay chân thật, RViz xoay ĐÚNG TỈ LỆ theo (chuyển động tương đối chính xác — xác nhận TF sống thật), nhưng tư thế TUYỆT ĐỐI không khớp tư thế thật của robot (vẫn "duỗi" dù robot đang ở tư thế khác).
- **Nguyên nhân**: `RawToLogical()`/`LogicalToRaw()` (`actuator_if.c`) trước đây CHỈ đổi dấu (±1 theo `MOTOR_JOINT_SIGN`), không hề bù độ lệch nào — ngầm giả định "điểm 0 tuyệt đối của encoder driver = đúng góc 0 URDF vẽ", một giả định về LẮP RÁP CƠ KHÍ, không được hiệu chỉnh bằng phần mềm.
- **Không thể dùng bảng offset cố định**: đọc `oneLeg/main_adaptive_confirmed_v2.c.bak` xác nhận (do người dùng xác nhận trực tiếp trên phần cứng) **điểm 0 của encoder driver TRÔI MỖI LẦN MẤT ĐIỆN** — offset đúng cho phiên cấp điện này sẽ SAI ở phiên sau. Đây là lý do oneLeg phải làm "adaptive": đo lại HOME MỖI LẦN BOOT.
- **Fix**: thêm `g_home_offset_rad[JOINT_COUNT]`, tính lại mỗi lần boot trong `Actuator_Init()` bước đọc HOME: `offset = ASSUMED_REST_LOGICAL_RAD[loai_khop] - home_do_duoc`. `RawToLogical()`/`LogicalToRaw()` cộng/trừ offset này một cách trong suốt — mọi nơi khác (giới hạn, target, telemetry) tự động đi qua offset mà không cần sửa gì thêm. `ASSUMED_REST_LOGICAL_RAD` hiện đặt = 0.0 cho cả 3 loại khớp (khớp với việc URDF có góc=0 là tư thế "duỗi thẳng" — xem mục 6 — đúng tư thế nằm xấp giả định lúc cấp nguồn).
- **Lưu ý xác minh còn lại**: `ASSUMED_REST_LOGICAL_RAD=0.0` là giá trị GIẢ ĐỊNH dựa trên suy luận hình học URDF, CHƯA đo đạc xác nhận trực tiếp trên robot thật xem "nằm xấp" có đúng khớp góc=0 cho MỌI loại khớp (Hang/Dui/Goi) hay không — nếu sau khi test vẫn còn lệch, chỉ cần sửa 3 giá trị trong mảng này (không cần sửa logic).
- File: `firmware/stm32h7/app/src/actuator_if.c`.
- **Lịch sử**: triển khai lần 1 (2026-08-12) → bị revert theo yêu cầu người dùng → **triển khai lại lần 2 (2026-08-13), ĐANG BẬT** sau khi xác nhận cần kết hợp với mục 2.8 (URDF) mới đủ. Xem tóm tắt ⭐ ở đầu mục 2.6 để biết trạng thái CUỐI CÙNG.

### 2.7. [CHƯA FIX] Không giám sát sức khỏe CAN lúc runtime
- **Phát hiện khi**: CAN_INSTANCE_1 (chân trước) mất phản hồi hoàn toàn 1 khoảng thời gian dài (xem mục 3), sau đó tự phục hồi khi có người chạm/lắc dây — nghi ngờ đầu nối lỏng.
- **Vấn đề**: `g_joint_ok[j]` (cờ "khớp OK") chỉ được set **1 lần lúc boot** trong `Actuator_Init()`, không bao giờ được đánh giá lại. `CAN_IsBusOff()` (đã có hàm sẵn trong `can.c`) **không được gọi ở đâu cả** trong vòng lặp chính.
- **Rủi ro**: nếu CAN rơi vào Bus-Off giữa lúc đang vận hành thật, firmware sẽ không tự phát hiện, không tự phục hồi (`CAN_Start()` lại), và không báo động — kẹt cho tới khi reset board thủ công.
- **Đề xuất fix (chưa làm)**: thêm kiểm tra định kỳ (vd mỗi 1s) trong vòng lặp chính của `main.c`: gọi `CAN_IsBusOff(instance)` cho cả 2 bus, nếu true thì tự gọi lại `CAN_Start(instance)` để phục hồi; cân nhắc thêm cờ trạng thái bus báo lên `/joint_fb` hoặc topic riêng để EC biết bus nào đang lỗi.

### 2.8. [phần chi tiết] Hiệu chỉnh home ngay tại định nghĩa URDF (thay vì offset runtime)
- **Hướng tiếp cận khác mục 2.6** (mục 2.6 đã revert): thay vì bù offset trong firmware lúc runtime, sửa THẲNG `<origin rpy>`/`<limit>` của từng khớp trong `babydog.xacro` để góc logic=0 tự nó đã đúng nghĩa "home" — sạch hơn vì mọi công cụ dùng chung URDF (RViz, sim, FK/IK...) tự động hiểu đúng, không cần patch riêng ở firmware.
- **Đo home bằng sim thay vì đoán**: thả robot rơi tự do dưới trọng lực trong Gazebo (controller mặc định ở Passive/torque=0 khi mới launch `sim.launch.py`), đợi ổn định (velocity→0), đọc `/joint_states`. Xác nhận thân robot nằm phẳng sát đất (`z=0.0538m`, orientation gần như phẳng tuyệt đối `w≈0.9999999`) — đúng nghĩa nằm xấp thật, không phải treo lơ lửng.
- **Nghi ngờ ban đầu (SAI, đã tự sửa)**: lần đo đầu (chưa có damping khớp) ra `abad=±0.363, hip=-1.236, knee=2.704` — khớp gối dừng sát giới hạn cơ khí trên. Người dùng quan sát thấy vô lý ("gối nhô lên như cào cào", không giống nằm xấp thật có cẳng chân áp đất) → nghi ngờ do khớp `dynamics damping="0.0"` khiến robot rơi có quán tính, va vào giới hạn SAI thay vì dừng ở điểm cân bằng thật. Đã thử thêm damping tạm (`damping=2.0 friction=0.5`) để kiểm chứng.
- **Kết quả sau khi thêm damping**: RA GẦN NHƯ Y HỆT lần đầu (`abad≈0, hip=-1.213, knee=2.705`, vẫn dừng sát giới hạn trên) — nghi vấn quán tính/damping bị loại 1 phần.
- **Test thứ 3 — nghi ngờ do ma sát bàn chân cao (`mu=6.0`) làm chân "dính" tại điểm chạm đất đầu tiên**: giảm toàn bộ `mu1/mu2` (thân + bàn chân) xuống `0.02` (gần như trượt tự do), đo lại → **`abad=±0.360, hip=-1.238, knee=2.705`**, vận tốc ~1e-11 (đứng yên tuyệt đối) — GẦN NHƯ Y HỆT lần đo đầu tiên (không phải bản có damping). Đã khôi phục `mu1/mu2` về nguyên bản (0.2 thân, 6.0 bàn chân) sau khi đo xong.
- **Kết luận sau 3 lần đo độc lập** (không damping/có damping/ít ma sát) đều hội tụ cùng vùng giá trị: đây CHẮC CHẮN là điểm cân bằng vật lý thật của bộ `<inertial>` hiện có trong URDF, không phải artifact của damping hay ma sát. Người dùng xác nhận đây **CHÍNH LÀ** tư thế nằm xấp cần tìm (ban đầu tưởng "cào cào" là sai, thực ra là đúng). Giá trị CUỐI CÙNG dùng để hiệu chỉnh (từ lần đo ma sát thấp, đáng tin nhất vì gần với điều kiện gốc nhất): **`abad=0.360, hip=-1.238, knee=2.705`**.
- **Công thức bù** (đã verify bằng toán ma trận xoay, sai số ~1e-16): với khớp `axis="1 0 0"` (abad) → `origin rpy = (home, 0, 0)`; với khớp `axis="0 -1 0"` (hip/knee, trục âm) → `origin rpy = (0, -home, 0)`. Đồng thời **PHẢI dịch `<limit lower/upper>` theo cùng lượng** (`new = old - home`) vì vùng giới hạn hợp lệ vẫn neo theo số cũ, không tự dịch theo origin — quên bước này khiến khớp gối bị kẹp cứng vào biên sai (đã tự phát hiện và sửa lại trong lúc làm). Với `knee=2.705` trùng khít giới hạn trên cũ, limit mới của knee là `[-2.269, 0.0]`.
- **Lỗi phụ đã tự bắt được lúc làm**: script sửa origin/limit ban đầu lỡ khớp luôn `foot_joint` (khớp `fixed`, không nên đổi) vì trùng `xyz="0 0 -0.15"` với `knee_joint` — đã phát hiện qua kiểm tra lại và sửa về `rpy="0 0 0"` cho cả 4 chân.
- **Cách verify đáng tin cậy**: KHÔNG dùng "thả rơi lại xem có về đúng 0 không" (test này không đáng tin — hệ nhiều khớp có giới hạn, thả lại từ điểm khác có thể hội tụ về điểm cân bằng khác, không phản ánh đúng/sai của phép ánh xạ động học). Verify đúng cách: tính tay ma trận biến đổi (rotation) của "URDF mới, góc=0" và so với "URDF cũ, góc=HOME đo được" — phải khớp tuyệt đối (đã làm, sai số ~1e-16 chỉ do làm tròn số). Damping tạm thêm lúc điều tra đã được XÓA lại `0.0/0.0` sau khi xong.
- **Lịch sử**: áp dụng lần 1 → revert (đo lại qua sim, nghi ngờ giá trị) → thử nhiều biến thể độ cao spawn/damping/ma sát (mục 3 dòng thời gian) → **áp dụng lại lần cuối (2026-08-13), ĐANG BẬT trong `babydog.xacro`** cùng lúc với mục 2.6 (firmware). Xem tóm tắt ⭐ ở đầu mục 2.6.
- **Còn thiếu để khép kín hoàn toàn trên phần cứng thật**: fix này CHỈ đúng cho tầng URDF/sim/RViz. Trên robot thật, firmware vẫn báo giá trị RAW không hiệu chỉnh (mục 2.6 đã revert) — nghĩa là để RViz hiển thị đúng khi dùng robot thật, vẫn cần làm lại lớp offset adaptive runtime (mục 2.6) sau này, dùng lại đúng `ASSUMED_REST_LOGICAL_RAD=0.0` vì giờ URDF đã định nghĩa đúng "0 = home".
- File: `src/main_bot/description/babydog.xacro`.

---

## 3. Sự cố CAN_INSTANCE_1 (6 driver chân trước) mất phản hồi — vật lý, không phải firmware

**Triệu chứng**: `/joint_fb` báo đúng 0.000 tuyệt đối (giá trị fallback khi chưa từng có feedback) cho cả 6 khớp chân trước, lặp lại y hệt qua nhiều lần đọc — trong khi 6 khớp chân sau (CAN_INSTANCE_2) đọc ra giá trị thật, thay đổi theo chuyển động tay.

**Quá trình loại trừ firmware** (theo thứ tự, mỗi bước đều cho kết quả "không phải do code"):
1. Diff GPIO config (PA11/12 AF9), toàn bộ `can.c` (CAN_Init/CAN_Start/TX FIFO/CCE-INIT fix), và ánh xạ `Motor_BusForJoint`/`Motor_IdForJoint` với bản oneLeg đã test thật cả 4 chân → **giống 100% về logic** (chỉ khác comment).
2. Đọc thanh ghi trạng thái CAN_INSTANCE_1 trực tiếp qua `/joint_fb` (mượn tạm field velocity của khớp 0/1/2 vì luôn =0):
   - `PSR`: `LEC=Ack Error`, `ACT=Transmitter`, `EP=1 (Error Passive)`, `BO=0 (chưa bus-off)`.
   - `ECR`: `TEC=128` (đúng ngưỡng Error Passive).
   - `CCCR`: `FDOE=1,BRSE=1,INIT=0,MON=0,TEST=0` — cấu hình đúng, đang chạy Normal thật.
   - **Ý nghĩa**: Ack Error là lỗi phần cứng CAN controller tự phát hiện ở khe ACK cuối khung — khung đã đi qua ĐÚNG toàn bộ arbitration/CRC, chỉ riêng không ai ACK lại. Đây không phải lỗi firmware có thể gây ra (nếu sai ID/data/format sẽ ra Form/CRC/Stuff/Bit Error, không phải Ack Error).
3. Build và flash **thẳng bản gốc oneLeg** (`main_12joint_hold_proven.c.bak`, không sửa 1 dòng logic CAN, chỉ thêm in kết quả qua USART1 + ép kp=kd=0 an toàn) lên đúng board thật → **kết quả giống hệt**: 0/6 chân trước, 6/6 chân sau.
4. Test loopback nội bộ CAN_INSTANCE_1 (`CAN_MODE_LOOPBACK_INTERNAL` — TX nối tắt về RX ngay trong chip, không chạm chân/transceiver) → **`tx_ok=1, nhận_được_khung=1`** — chứng minh toàn bộ peripheral FDCAN1 (digital logic, bit timing, Message RAM, FIFO) hoạt động hoàn hảo. Lỗi chắc chắn nằm ở tầng analog phía sau chip (transceiver hoặc dây).

**Kết luận**: lỗi vật lý ở CAN_INSTANCE_1, khoanh vùng còn lại là **chip transceiver CAN** (cần nguồn logic riêng 3.3V/5V, KHÁC với nguồn 24-28V cấp cho driver động cơ — dễ bị bỏ sót khi chỉ đo nguồn driver) hoặc **dây/đầu nối CAN_H/CAN_L** phía sau transceiver, hoặc **điện trở đầu cuối (termination)**.

**Diễn biến cuối**: sau nhiều lần thao tác/cầm nắm robot, kết nối **tự phục hồi** — mạnh mẽ gợi ý **đầu nối lỏng** (không phải hỏng hẳn linh kiện). Xem mục 2.6 — đây là lý do cần thêm giám sát runtime, vì kiểu lỗi "lỏng, chập chờn" này có thể tái diễn bất cứ lúc nào.

**Việc CHƯA làm**: đo trực tiếp bằng đồng hồ/máy hiện sóng điện áp vi sai CAN_H/CAN_L và nguồn logic transceiver để xác nhận độc lập (hiện tại toàn bộ chẩn đoán dựa trên chính MCU tự báo cáo, chưa có xác nhận từ thiết bị thứ ba).

---

## 4. Vấn đề vận hành / process hygiene (không phải bug code)

### 4.1. Chạy trùng nhiều tiến trình `real_ros2_control.launch.py`/`rz_real.launch.py`
- **Triệu chứng đã gặp**: `TF_OLD_DATA ignoring data from the past` (2 nguồn cùng publish `/tf` với mốc thời gian xen kẽ không đồng bộ); spawner controller báo `FATAL Failed loading controller ...` (do bị load trùng tên vào cùng controller_manager); **2 `micro_ros_agent` cùng lúc `lsof` giữ chung `/dev/ttyUSB0`** — rủi ro an toàn thật (2 tiến trình cùng gửi lệnh PD xuống động cơ thật qua chung 1 dây).
- **Quy tắc**: chỉ chạy **ĐÚNG 1** tiến trình `real_ros2_control.launch.py` tại một thời điểm. Muốn xem RViz riêng thì chạy `rviz2 -d ...` độc lập (không kèm launch full stack) — an toàn, không trùng.
- **Cách kiểm tra nhanh trước khi launch**: `lsof /dev/ttyUSB0` và `ps aux | grep -E "micro_ros_agent|ros2_control_node|robot_state_publisher"`.
- **Lưu ý quan trọng**: `pkill -f "ros2 launch main_bot"` / `pkill -f micro_ros_agent` **đôi khi ÂM THẦM KHÔNG giết được** tiến trình dù pattern khớp đúng (đã gặp nhiều lần) — LUÔN kiểm tra lại bằng `ps aux`/`lsof` SAU khi pkill, nếu còn sót thì `kill -9 <PID>` trực tiếp theo từng PID cụ thể.

### 4.2. `ROS_DOMAIN_ID` lệch giữa terminal và launch file
- `real_ros2_control.launch.py` ép `ROS_DOMAIN_ID=0` cho MỌI tiến trình nó sinh ra.
- `~/.bashrc` của máy đặt mặc định `ROS_DOMAIN_ID=7` cho terminal thường.
- **Hệ quả nếu quên**: mở terminal mới gõ `ros2 topic list`/mở RViz sẽ **KHÔNG thấy gì cả** — không phải lỗi kết nối, chỉ là lệch domain.
- **Luôn nhớ**: `export ROS_DOMAIN_ID=0` trước khi dùng `ros2 ...` CLI hoặc mở RViz để xem robot thật.

### 4.3. Cần source đủ 3 workspace
```bash
source /opt/ros/jazzy/setup.bash
source /home/dvt/mros/mros_ws/install/setup.bash   # <-- de tim duoc micro_ros_agent, hay bi quen
source /home/dvt/babyDog/install/setup.bash
```

---

## 5. Lệnh tham khảo nhanh

```bash
# Build + flash firmware MCU (co tu-reset sau khi ghi)
cd /home/dvt/babyDog/firmware/stm32h7
make
make flash

# Kiem tra khong co tien trinh trung truoc khi test that
lsof /dev/ttyUSB0
ps aux | grep -E "micro_ros_agent|ros2_control_node|robot_state_publisher"

# Chay full stack that (khong rviz)
export ROS_DOMAIN_ID=0
ros2 launch main_bot real_ros2_control.launch.py serial_dev:=/dev/ttyUSB0 rviz:=false

# Xem RViz rieng (sau khi full stack da chay)
export ROS_DOMAIN_ID=0
rviz2 -d /home/dvt/babyDog/install/main_bot/share/main_bot/rviz/babydog_real.rviz

# Doc du lieu khop that
export ROS_DOMAIN_ID=0
ros2 topic echo /joint_fb --once
ros2 control list_controllers
```
