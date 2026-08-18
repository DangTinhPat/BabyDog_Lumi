# Firmware STM32H7 - relay lệnh khớp

Board thật: **FANKE FK743M5-XIH6** (STM32H743XIH6, xem [`BOARD_FK743M5-XIH6.md`](BOARD_FK743M5-XIH6.md)). Dùng thư viện thanh ghi tự viết (không HAL/CMSIS) từ [`~/OUT_SAVE/babyDog_fwSTM/`](../../../OUT_SAVE/babyDog_fwSTM) - project firmware gốc, đã test một phần trên board thật (xem "Đã kiểm tra" bên dưới) - copy nguyên `lib/` + `startup_stm32h743xx.c` + `stm32h743xihx_flash.ld` vào đây, cộng thêm `app/` là code ứng dụng riêng cho giai đoạn đứng/ngồi này. `~/OUT_SAVE/babyDog_fwSTM/` vẫn giữ nguyên làm project tham khảo/hồi quy độc lập (main.c gốc có bài test loopback CAN qua 7 test case) - đã chuyển từ `~/babyDog_fwSTM/` sang `~/OUT_SAVE/` lúc dọn dẹp trước khi kết nối robot thật (xem OUT_SAVE/README nếu có).

## Cấu trúc

```
lib/inc, lib/src/     Thư viện dùng chung (copy từ ~/OUT_SAVE/babyDog_fwSTM/, KHÔNG sửa):
                        rcc      - clock (HSE 25MHz -> PLL1 480MHz, xem "SystemClock" bên dưới)
                        gpio     - cấu hình chân
                        can      - driver FDCAN (Classic + FD), ĐÃ TEST loopback thật trên board
                        tim      - timer đa năng (timebase + PWM, PWM chưa dùng ở đây)
                        systick  - delay CHẶN (blocking) - KHÔNG dùng cho đo elapsed time, xem tick.c
                        usart, dma, cache, mem_attr, stm32h743_regs - còn lại, chưa dùng trong app này

app/inc, app/src/     Code ứng dụng riêng cho giai đoạn đứng lên/ngồi xuống:
                        motor_topology.h    Giao thức CAN-FD với 12 board driver khớp
                                            (tự định nghĩa)
                        tick.h/.c           Bộ đếm mili-giây tự do (TIM2), thay lib/systick.h
                                            (blocking) cho vòng lặp chính không-chặn
                        actuator_if.h/.c    Đóng gói vị trí/vận tốc/kp/kd/Tff riêng từng khớp + feedback qua CAN
                        app_i2c.h/.c         I2C1 app-local, PB8/PB7, timeout hữu hạn
                        mpu6050.h/.c         WHO_AM_I + DLPF + đọc accel/gyro 100 Hz

main.c                Vòng lặp chính, nối tất cả lại với nhau
startup_stm32h743xx.c, stm32h743xihx_flash.ld   (copy từ ~/OUT_SAVE/babyDog_fwSTM/, không đổi)
```

## Kiến trúc giao tiếp

- **RDK X5/laptop <-> STM32H7: micro-ROS qua UART1** - subscriber `/joint_cmd`, publishers `/joint_fb` + `/imu/raw`. FSM/FK/IK và Kalman IMU đều chạy trên EC; firmware không có tư thế Stand/Sit hay bộ cân bằng.
- **MPU6050 -> STM32H7: I2C1** - J4 `PB8=SCL`, `PB7=SDA`, địa chỉ `0x68`; ±2g, ±250°/s, DLPF và 100 Hz. Lỗi/mất sensor chỉ tạo status lỗi, không chặn đường joint/CAN.
- **STM32H7 <-> 12 board driver khớp: CAN-FD** (`fd_format=true`) - `motor_topology.h` định tuyến bus/ID, `baby_alpha2_protocol.c` mã hóa khung PD 12 byte. Mỗi driver chạy PD cục bộ và cộng `tau_ff`; STM32 gửi position/v_des/Kp/Kd/Tff riêng từng khớp. 6 khớp trên mỗi CAN instance.

## Build

Sau khi sửa `JointCmd.msg`, `JointFb.msg` hoặc `ImuRaw.msg`, phải regenerate type-support/lib micro-ROS trước; nếu không EC và MCU có thể dùng hai layout DDS khác nhau:

```bash
make microros-lib     # chạy từ repo root, cần ~/mros/mros_ws
```

```bash
cd firmware/stm32h7
make                # ra build/firmware.elf/.bin/.hex + in size
make flash          # nạp qua st-flash (cần stlink-tools + ST-Link thật cắm vào board)
make clean
```

Cần `arm-none-eabi-gcc` trong PATH. Makefile tự tìm thư mục header `newlib/` tương đối theo vị trí toolchain (workaround cho lỗi đóng gói phổ biến của `gcc-arm-none-eabi` từ apt/.deb: header newlib bị tách khỏi đường tìm mặc định, gây lỗi `string.h: No such file or directory` dù toolchain cài đúng) - áp dụng cho cả Makefile ở đây và ở `~/OUT_SAVE/babyDog_fwSTM/`.

Build sạch (không lỗi/warning ngoài các warning `_close`/`_read`/`_write`/`_lseek not implemented` vô hại từ `--specs=nosys.specs`) với `arm-none-eabi-gcc 13.2.1`.

## ĐÃ KIỂM TRA vs CHƯA KIỂM TRA

**Đã kiểm tra THẬT trên board FK743M5-XIH6** (không phải chỉ build-test):
- Bản tham chiếu `~/OUT_SAVE/testSTM` đã đọc MPU6050 thật trên chính PB8/PB7 của board này và publish micro-ROS ổn định khoảng 41.6 Hz với `sensor_msgs/Imu`. Production tái sử dụng transaction repeated-START đã sửa từ lần bring-up đó, nhưng đổi sang bản tin compact và Kalman trên EC.
- `lib/can.c`: 7 test case đa dạng (Classic/FD, ID chuẩn/mở rộng, 0-64 byte, có/không BRS) qua **loopback nội bộ thật** (`CAN_MODE_LOOPBACK_INTERNAL`) - gửi+nhận khớp tuyệt đối 100%, ổn định qua nhiều lần reset. Quá trình test tìm và sửa 4 lỗi thật (xem comment đầu `lib/inc/can.h`): GFC.ANFS/ANFE routing sai FIFO, vị trí bit DLC, Message RAM chỉ chấp nhận truy cập 32-bit, mask get_index thiếu bit.
- `lib/rcc.c`: HSE 25MHz lên đúng (HSERDY=1) trên board thật. Bước nâng VOS0/Overdrive để chạy 480MHz **luôn timeout** (nguyên nhân chưa rõ, xem comment trong `rcc.c`) - firmware **không treo máy** nhờ timeout an toàn, tự động tiếp tục chạy ở HSI 64MHz mặc định.
- `app/`, `main.c` (code viết cho giai đoạn đứng/ngồi này): build sạch, đã build-test lại trên máy này với toolchain thật của bạn - **CHƯA nạp/chạy trên board thật ở mức toàn hệ thống**.

**CHƯA kiểm tra:**
- Đường production `/imu/raw` 100 Hz chạy đồng thời với `/joint_cmd` + `/joint_fb` trên robot; cần đo rate/stale diagnostics sau khi flash. Mapping trục vật lý MPU6050 -> `imu_link` vẫn phải qua checklist trong `GUIDE.md`; active balance đang tắt.
- Bit-timing FDCAN 1Mbit/s (tính từ HSE 25MHz, xem công thức trong `main.c`) qua bus CAN thật + transceiver + board driver khớp - `can.c` mới test loopback nội bộ (không qua đường truyền vật lý nên không phản ánh đúng tốc độ thật). **Bắt buộc đo bằng dao động ký/logic analyzer trên CANH/CANL** trước khi nối vào bus có board động cơ khác.
- `CAN_INSTANCE_2` (FDCAN2) - `lib/can.c` mới test `CAN_INSTANCE_1`.
- **Giao thức CAN với board driver động cơ (`motor_topology.h`) là TỰ ĐỊNH NGHĨA** (chưa có giao thức có sẵn từ board driver, theo xác nhận) - board driver mỗi khớp (bạn tự làm/đang làm) PHẢI cài đặt encode/decode khớp y hệt, hoặc sửa `motor_topology.h` + `actuator_if.c` cho khớp với thứ board driver thật sự nói.
- Chưa test vòng lặp đầy đủ RDK X5 -> STM32 -> board driver khớp -> động cơ thật, vì chưa có board driver khớp thật để nối vào.
- Điện trở kết cuối CAN 120Ω - chưa thấy rõ trên schematic board breakout động cơ (chỉ có R1/R2/R7/R8 10Ω, giống lọc EMI hơn là kết cuối chuẩn) - kiểm tra khi bring-up thật.
- Mapping khớp -> connector (`Motor_BusForJoint()` trong `motor_topology.h`) là giả định theo schematic - sửa lại nếu bạn đấu dây thật khác.

## Hợp đồng lệnh

- `/joint_cmd` qua micro-ROS: `target_angle_mrad[12]`, `target_velocity_mrad_s[12]`, `kp_x100[12]`, `kd_x100[12]`, `tau_ff_mnm[12]`, `seq`.
- STM32 đổi từng phần tử sang SI, clamp lại bằng `MOTOR_KP_ABS_LIMIT`, `MOTOR_KD_ABS_LIMIT`, `MOTOR_TAU_ABS_LIMIT_NM`, đổi LOGIC→RAW rồi tạo BabyAlpha2 PD frame 12 byte. EC cũng kẹp Tff độc lập ở ±10 N·m trước khi serialize; MCU là lớp chặn cuối thứ hai.
- Position RAW dùng dấu + home offset. Velocity/Tff RAW dùng cùng dấu khớp nhưng không bao giờ cộng home offset.
- `/joint_fb` vẫn chỉ có góc/vận tốc ước lượng; chưa có torque/current feedback.
- `/imu/raw`: accel `m/s² ×1000`, gyro `rad/s ×1000`, `stamp_ms` dùng tính delta-time và status `OK/INIT_FAILED/READ_FAILED`. Đây là transport raw fixed-size; `/imu/data` chuẩn và Kalman nằm trên EC.

## SystemClock

`main.c` gọi `RCC_SystemClock_Config_HSE_480MHz()` (best-effort - xem "Đã kiểm tra" ở trên, hiện luôn timeout ở bước VOS0 và giữ nguyên HSI 64MHz, KHÔNG treo máy). Firmware này không phụ thuộc việc PLL có lên hay không:
- CAN dùng HSE trực tiếp làm kernel clock (mặc định phần cứng sau reset, xem `lib/can.c`), không qua PLL/SystemCoreClock.
- I2C1 chọn HSI 64 MHz làm kernel clock một cách tường minh, nên timing 100 kHz không đổi giữa chế độ fallback và PLL.
- `Tick_Init()` (app/) tính prescaler TIM2 dựa trên `SystemCoreClock` hiện tại - đúng ở trạng thái thật hiện nay (HSI 64MHz, không chia domain nào vì bước PLL fail sớm) nhưng **sẽ cần sửa lại** nếu sau này bạn sửa được lỗi VOS0 và PLL 480MHz chạy thành công (APB1 lúc đó có prescaler /2 + nhân đôi timer clock = 240MHz, khác `SystemCoreClock`=480MHz) - xem ghi chú trong `app/inc/tick.h`.
