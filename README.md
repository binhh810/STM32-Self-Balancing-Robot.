# STM32 Self-Balancing Robot

Dự án phát triển phần mềm điều khiển Robot tự cân bằng hai bánh, sử dụng vi điều khiển STM32F103.

##  Các tính năng chính và thuật toán đã áp dụng
* **Bộ lọc Kalman (Kalman Filter):** Tích hợp thư viện `mpu6050.c` đọc dữ liệu I2C từ cảm biến MPU6050, sử dụng thuật toán Kalman để dung hợp dữ liệu gia tốc (Accelerometer) và con quay hồi chuyển (Gyroscope), triệt tiêu nhiễu và hiện tượng trôi góc (Drift).
* **Điều khiển PID & Anti-windup:** Thuật toán PID vòng lặp 4ms được tối ưu hóa. Tích hợp khâu giới hạn Anti-windup cho thành phần Tích phân (I) để tránh hiện tượng bão hòa khi robot khởi động hoặc bị cản trở.
* **Giao tiếp Bluetooth (BLE) không đồng bộ:** Sử dụng ngắt `HAL_UART_RxCpltCallback` để nhận lệnh điều hướng (F, B, L, R) từ HC-05 mà không làm treo hay ảnh hưởng đến tần số lấy mẫu (250Hz) của vòng lặp cân bằng.
* **Bù trừ phi tuyến động cơ bước:** Ánh xạ phi tuyến (Non-linear Mapping) để chuyển đổi từ tín hiệu lực (Output PID) sang chu kỳ xung điều khiển driver A4988.
* **Điều khiển động cơ tốc độ cao:** Sử dụng trực tiếp thanh ghi `BSRR` trong ngắt Timer (`HAL_TIM_PeriodElapsedCallback`) để tạo xung STEP, tối ưu tốc độ thực thi so với hàm HAL thông thường.

##  Cấu trúc thư mục
* `main.c` / `main.h`: Chứa vòng lặp cấu hình hệ thống, thuật toán PID, ngắt UART và ngắt Timer tạo xung động cơ.
* `mpu6050.c` / `mpu6050.h`: Thư viện giao tiếp I2C và thuật toán toán học cho bộ lọc Kalman.
