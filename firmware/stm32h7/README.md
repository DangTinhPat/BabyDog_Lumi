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
                        actuator_if.h/.c    Đóng gói lệnh vị trí/kp/kd + feedback cho 12 khớp qua CAN

main.c                Vòng lặp chính, nối tất cả lại với nhau
startup_stm32h743xx.c, stm32h743xihx_flash.ld   (copy từ ~/OUT_SAVE/babyDog_fwSTM/, không đổi)
```

## Kiến trúc giao tiếp

- **RDK X5/laptop <-> STM32H7: micro-ROS qua UART1** - `/joint_cmd` và `/joint_fb`. FSM/FK/IK chạy ở ROS2 controller; firmware không có tư thế Stand/Sit đặt sẵn.
- **STM32H7 <-> 12 board driver khớp: CAN-FD** (`fd_format=false`) - `motor_topology.h`. Mỗi khớp có 1 board driver CAN riêng, tự làm PWM + đọc encoder 6 dây (2 dây nguồn động cơ + 4 dây encoder VCC/GND/A/B) tại chỗ, tự chạy **thuật toán PD cục bộ** (`pwm = kp*(target - đo_được) + kd*(0 - vận_tốc_đo_được)`) bằng kp/kd STM32H7 gửi xuống theo `/joint_cmd`. 6 khớp trên `CAN_INSTANCE_1` (connector P1/P2/P3/P7/P9/P11), 6 khớp trên `CAN_INSTANCE_2` (PB5/PB6, connector P4/P5/P6/P8/P10/P12).

## Build

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
- `lib/can.c`: 7 test case đa dạng (Classic/FD, ID chuẩn/mở rộng, 0-64 byte, có/không BRS) qua **loopback nội bộ thật** (`CAN_MODE_LOOPBACK_INTERNAL`) - gửi+nhận khớp tuyệt đối 100%, ổn định qua nhiều lần reset. Quá trình test tìm và sửa 4 lỗi thật (xem comment đầu `lib/inc/can.h`): GFC.ANFS/ANFE routing sai FIFO, vị trí bit DLC, Message RAM chỉ chấp nhận truy cập 32-bit, mask get_index thiếu bit.
- `lib/rcc.c`: HSE 25MHz lên đúng (HSERDY=1) trên board thật. Bước nâng VOS0/Overdrive để chạy 480MHz **luôn timeout** (nguyên nhân chưa rõ, xem comment trong `rcc.c`) - firmware **không treo máy** nhờ timeout an toàn, tự động tiếp tục chạy ở HSI 64MHz mặc định.
- `app/`, `main.c` (code viết cho giai đoạn đứng/ngồi này): build sạch, đã build-test lại trên máy này với toolchain thật của bạn - **CHƯA nạp/chạy trên board thật ở mức toàn hệ thống**.

**CHƯA kiểm tra:**
- Bit-timing FDCAN 1Mbit/s (tính từ HSE 25MHz, xem công thức trong `main.c`) qua bus CAN thật + transceiver + board driver khớp - `can.c` mới test loopback nội bộ (không qua đường truyền vật lý nên không phản ánh đúng tốc độ thật). **Bắt buộc đo bằng dao động ký/logic analyzer trên CANH/CANL** trước khi nối vào bus có board động cơ khác.
- `CAN_INSTANCE_2` (FDCAN2) - `lib/can.c` mới test `CAN_INSTANCE_1`.
- **Giao thức CAN với board driver động cơ (`motor_topology.h`) là TỰ ĐỊNH NGHĨA** (chưa có giao thức có sẵn từ board driver, theo xác nhận) - board driver mỗi khớp (bạn tự làm/đang làm) PHẢI cài đặt encode/decode khớp y hệt, hoặc sửa `motor_topology.h` + `actuator_if.c` cho khớp với thứ board driver thật sự nói.
- Chưa test vòng lặp đầy đủ RDK X5 -> STM32 -> board driver khớp -> động cơ thật, vì chưa có board driver khớp thật để nối vào.
- Điện trở kết cuối CAN 120Ω - chưa thấy rõ trên schematic board breakout động cơ (chỉ có R1/R2/R7/R8 10Ω, giống lọc EMI hơn là kết cuối chuẩn) - kiểm tra khi bring-up thật.
- Mapping khớp -> connector (`Motor_BusForJoint()` trong `motor_topology.h`) là giả định theo schematic - sửa lại nếu bạn đấu dây thật khác.

## Giao thức với board driver mỗi khớp - CAN-FD

- `MOTOR_CMD_BASE_ID + joint` (0x200-0x20B), STM32 -> board driver, mỗi khi nhận `/joint_cmd`: `target_angle_mrad` (int16, rad×1000) + `kp_x100`/`kd_x100` (uint16) + `seq`.
- `MOTOR_FB_BASE_ID + joint` (0x210-0x21B), board driver -> STM32: `measured_angle_mrad` + `measured_velocity_mrad_s` (int16) + `fault_flags` + `last_seq`. STM32 dùng vị trí đo được này làm điểm bắt đầu nội suy lần chuyển trạng thái tiếp theo (`Actuator_GetLastTarget()`).

## SystemClock

`main.c` gọi `RCC_SystemClock_Config_HSE_480MHz()` (best-effort - xem "Đã kiểm tra" ở trên, hiện luôn timeout ở bước VOS0 và giữ nguyên HSI 64MHz, KHÔNG treo máy). Firmware này không phụ thuộc việc PLL có lên hay không:
- CAN dùng HSE trực tiếp làm kernel clock (mặc định phần cứng sau reset, xem `lib/can.c`), không qua PLL/SystemCoreClock.
- `Tick_Init()` (app/) tính prescaler TIM2 dựa trên `SystemCoreClock` hiện tại - đúng ở trạng thái thật hiện nay (HSI 64MHz, không chia domain nào vì bước PLL fail sớm) nhưng **sẽ cần sửa lại** nếu sau này bạn sửa được lỗi VOS0 và PLL 480MHz chạy thành công (APB1 lúc đó có prescaler /2 + nhân đôi timer clock = 240MHz, khác `SystemCoreClock`=480MHz) - xem ghi chú trong `app/inc/tick.h`.
