# PHELAN BUTTERFLY - Hệ Thống Điều Khiển Bướm Cơ Học Bionic RC

[![C++17](https://img.shields.io/badge/Ng%C3%B4n%20ng%E1%BB%AF-C%2B%2B17-blue.svg)](https://en.cppreference.com/w/cpp/17)
[![Hardware](https://img.shields.io/badge/Vi%20%C4%91i%E1%BB%81u%20khi%E1%BB%83n-ESP32--C3%20%7C%20ESP32--C6-green.svg)](https://www.espressif.com/)
[![License: Proprietary](https://img.shields.io/badge/Gi%E1%BA%A5y%20ph%C3%A9p-B%E1%BA%A3n%20quy%E1%BB%81n%20%C4%91%C3%B3ng-red.svg)](LICENSE)
[![Tác giả](https://img.shields.io/badge/T%C3%A1c%20gi%E1%BA%A3-PhelanDev-red.svg)](https://github.com/PhelanDev)

**PHELAN BUTTERFLY** là hệ thống điều khiển bay cho bướm cơ học vỗ cánh sinh học (Bionic RC Ornithopter) chạy trên vi điều khiển **ESP32-C3** và **ESP32-C6**. Dự án tích hợp các thuật toán khí động học cánh vỗ tiên tiến, bù biên độ động theo tay ga, và hỗ trợ đa dạng phương thức điều khiển: từ tay cầm RC truyền thống cho đến buồng lái Web Ground Control Station (GCS) trên điện thoại thông minh.

---

## Tính Năng Nổi Bật

- **Biên độ vỗ động theo tay ga (`dynamicAmp`)**: Tự động co biên độ quét cánh từ **$55.0^\circ$** (ở mức ga thấp $1050\text{ µs}$) về **$35.0^\circ$** (ở mức ga tối đa $2000\text{ µs}$). Giúp bướm ôm gió sinh lực nâng cất cánh ở ga thấp, đồng thời triệt tiêu rung giật quán tính và chống cháy nhông servo khi vỗ nhanh ở ga cao.
- **Vòng lặp điều khiển servo 300Hz**: Chu kỳ điều khiển bay siêu mịn $3333\text{ µs}$ kết hợp bộ lọc làm mượt EMA, đảm bảo hai cánh vỗ đồng pha tuyệt đối.
- **Tích hợp 3 trong 1 (3-in-1 Protocols)**:
  - **CRSF (ExpressLRS)**: Tự động giải mã tín hiệu tốc độ cao 420.000 baud.
  - **SBUS**: Tự động nhận diện tín hiệu 100.000 baud.
  - **Wi-Fi Web GCS**: Gói tin nhị phân WebSocket điều khiển thời gian thực ở tần số 50Hz.
- **Buồng lái Web Pure Cockpit siêu nhẹ (~43.7 KB)**: Nhúng trực tiếp vào bộ nhớ Flash của ESP32, vào trang web là bay ngay lập tức:
  - Cần ga bên trái mặc định ở đáy **0%** an toàn, tích hợp cơ chế giữ ga hành trình (**Sticky Throttle Hold**).
  - Khóa an toàn cất cánh (**Arming Interlock**): Bắt buộc ga ở đúng 0% mới cho phép nhấn `ARM`.
  - Nhúng sẵn ảnh logo Phelan thư pháp dạng Base64 độc lập, bay ngoài trời không cần internet.
- **Chế độ bay Bionic**:
  - **Vỗ cánh linh hoạt**: Dạng sóng tam giác (Triangle) lực đẩy cao hoặc sóng sin (Sine) êm ái.
  - **Tư thế đậu nghỉ (Perch Mode)**: Gập cánh chữ V $+30^\circ$ để bướm đậu bám cành hoặc lượn lướt gió.
  - **Tự động kích hoạt Failsafe (+15°)**: Khi mất sóng Wi-Fi quá $500\text{ ms}$, bướm tự gập cánh góc chữ V $15^\circ$ để lướt hạ cánh an toàn, không bị bay mất.
- **Bảo vệ chống sụt áp Brownout**: Constructor cấp thấp (`disable_bod_early`) vô hiệu hóa ngắt BOD, chống reset chip khi servo ăn dòng giật lúc cất cánh.

---

## Bảng Phân Loại 12 Firmware Trong Dự Án

Toàn bộ dự án có **12 bản firmware** chia làm 6 nhánh tính năng, mỗi nhánh đều có đủ **bản cấu trúc** (chia module) và **bản đơn** (1 file duy nhất để nạp nhanh bằng Arduino IDE):

| # | Nhánh Firmware | Vi điều khiển | Phương thức điều khiển | Bản cấu trúc (Modular) | Bản đơn (Single All-in-One) |
| :-: | :--- | :--- | :--- | :--- | :--- |
| **1** | **Sóng Sin (Thuần RC)** | ESP32-C3 | Cắm bộ thu CRSF / ELRS hoặc SBUS | [`butterfly_esp32c3mini_Phelan_sin`](butterfly_esp32c3mini_Phelan_sin/) | [`butterfly_esp32c3mini_Phelan_single_sin`](butterfly_esp32c3mini_Phelan_single_sin/) |
| **2** | **Sóng Tam giác (Thuần RC)** | ESP32-C3 | Cắm bộ thu CRSF / ELRS hoặc SBUS | [`butterfly_esp32c3mini_Phelan_tria`](butterfly_esp32c3mini_Phelan_tria/) | [`butterfly_esp32c3mini_Phelan_single_tria`](butterfly_esp32c3mini_Phelan_single_tria/) |
| **3** | **C3 3-in-1 (V1 - SoftAP)** | ESP32-C3 | Bướm tự phát Wi-Fi (`192.168.4.1`) + CRSF + SBUS | [`code_3in1/c3mini3in1`](code_3in1/c3mini3in1/) | [`single_code_3in1/c3mini3in1`](code_3in1/single_code_3in1/c3mini3in1/) |
| **4** | **C3 3-in-1 (V2 - Station STA)** | ESP32-C3 | Bắt Wi-Fi Router/Hotspot (STA) + Bridge + CRSF + SBUS | [`code_3in1/c3mini3in1_2`](code_3in1/c3mini3in1_2/) | [`single_code_3in1/c3mini3in1_2`](code_3in1/single_code_3in1/c3mini3in1_2/) |
| **5** | **C6 3-in-1** | ESP32-C6 | Tự phát Wi-Fi AP + CRSF + SBUS cho chip C6 | [`code_3in1/c6mini`](code_3in1/c6mini/) | [`single_code_3in1/c6mini`](code_3in1/single_code_3in1/c6mini/) |
| **6** | **C3 Wi-Fi STA (Thuần Wi-Fi)** | ESP32-C3 | **Chuyên dụng Wi-Fi STA qua Web** *(Lược bỏ code tay cầm RC)* | [`c3wifi_sta/c3wifi_sta`](c3wifi_sta/c3wifi_sta/) | [`c3wifi_sta/single_c3wifi_sta`](c3wifi_sta/single_c3wifi_sta/) |

---

## Sơ Đồ Chân Nối Phần Cứng (Pinout)

| Chân ESP32-C3 | Kết nối | Chức năng |
| :--- | :--- | :--- |
| **GPIO 5** | Tín hiệu Servo Cánh Trái | Xuất xung PWM 300Hz cho servo trái |
| **GPIO 6** | Tín hiệu Servo Cánh Phải | Xuất xung PWM 300Hz cho servo phải |
| **GPIO 3** | Tín hiệu Bộ Thu RC (RX) | Cắm chân TX của RX ELRS / CRSF hoặc SBUS *(Bản `c3wifi_sta` không cần dùng)* |
| **GPIO 8** | Đèn LED Trạng Thái | Nhấp nháy báo trạng thái Wi-Fi và ARM |
| **VIN / 5V** | Nguồn Cấp (+) | Mạch nguồn BEC 5V (hạ áp từ pin LiPo 2S 7.4V - 8.4V) |
| **GND** | Mass (-) | Cực âm chung cho Pin 2S, ESP32 và 2 Servo |

> **Yêu cầu Nguồn Pin (Power Supply)**:
> - **Loại pin**: Sử dụng pin LiPo **2S (7.4V - 8.4V)** để đảm bảo công suất, mô-men xoắn và lực nâng cho 2 servo cánh vỗ ở tần số 300Hz.
> - **Cấp nguồn Vi điều khiển**: Dùng mạch hạ áp DC-DC hoặc BEC 5V từ pin 2S cấp vào chân `VIN / 5V` của ESP32 (không cấp trực tiếp 8.4V vào chân 5V của ESP32 để tránh làm cháy chip).
> - **Cấp nguồn Servo**: Nối dây dương servo vào nguồn cấp phù hợp (pin 2S cho HV servo hoặc đường BEC riêng) và **bắt buộc nối chung cực âm GND** với ESP32.
> - Đối với board ESP32-C6 (`c6mini`), đèn LED trạng thái được nối vào **GPIO 14**.

---

## Hướng Dẫn Cài Đặt & Nạp Code

### 1. Cấu hình Arduino IDE
1. Tải và cài đặt **Arduino IDE** (khuyên dùng bản 2.x).
2. Thêm gói board ESP32 trong *Preferences* $\rightarrow$ *Additional boards manager URLs*:
   `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
3. Cài đặt các thư viện cần thiết trong *Library Manager*:
   - `ESP32Servo`
   - `ESPAsyncWebServer` & `AsyncTCP` *(nếu nạp bản cấu trúc chia module; các bản single-file biên dịch trực tiếp không cần thư viện ngoài)*.
4. Cấu hình nạp trong menu **Tools**:
   - **Board**: `ESP32C3 Dev Module` (hoặc `ESP32C6 Dev Module`).
   - **Flash Frequency**: `80MHz`.
   - **Flash Size**: `4MB`.
   - **USB CDC On Boot**: `Enabled`.

### 2. Chọn bản nạp phù hợp
- **Nếu muốn bướm tự phát Wi-Fi, cầm điện thoại ra sân bay ngay**:
  - Mở và nạp: `code_3in1/single_code_3in1/c3mini3in1/c3mini3in1.ino`.
  - Kết nối Wi-Fi: `PHELAN BUTTERFLY` (Mật khẩu: `12345678`).
  - Mở trình duyệt gõ: `http://192.168.4.1`.
- **Nếu muốn bay xa qua Router ngoài sân hoặc qua Hotspot 4G/5G điện thoại**:
  - Mở và nạp: `c3wifi_sta/single_c3wifi_sta/single_c3wifi_sta.ino`.
  - Sửa `ROUTER_SSID` và `ROUTER_PASS` khớp với mạng Wi-Fi hoặc Hotspot của bạn.
  - Mở trình duyệt gõ: `http://phelan-102.local` (hoặc IP do Router cấp).
- **Nếu muốn bay bằng tay cầm điều khiển RC truyền thống (ELRS/SBUS)**:
  - Nạp bản sóng tam giác `butterfly_esp32c3mini_Phelan_single_tria.ino` hoặc bản đa năng `code_3in1/single_code_3in1/c3mini3in1.ino`.
  - Cắm chân tín hiệu RX vào **GPIO 3**, hệ thống tự động nhận diện giao thức.

---

## Tầm Xa Bay Khả Hữu

| Phương thức điều khiển | Thiết bị phát sóng | Tầm bay khả hữu thực tế |
| :--- | :--- | :---: |
| **Wi-Fi SoftAP Trực Tiếp** | Ăng-ten chip gốm onboard của ESP32 | **30m – 50m** |
| **Wi-Fi Station (Hotspot)** | Điểm phát Hotspot từ điện thoại di động | **40m – 60m** |
| **Wi-Fi Station (Router Sân)** | Router Wi-Fi công suất cao đặt giữa bãi bay | **80m – 150m** |
| **Tay cầm RC ExpressLRS (ELRS)** | Bộ thu ELRS 2.4GHz cắm vào GPIO 3 | **500m – 2+ km** |

---

## Giấy Phép Bản Quyền (License)

Dự án thuộc bản quyền đóng (**Proprietary License**), được bảo lưu toàn bộ quyền sở hữu trí tuệ bởi **PhelanDev**. Mọi hành vi sao chép, chỉnh sửa, dịch ngược, phân phối lại hoặc sử dụng cho mục đích thương mại khi chưa có sự cho phép bằng văn bản đều bị nghiêm cấm. Xem chi tiết tại tệp [LICENSE](LICENSE).

Tác giả: **[PhelanDev](https://github.com/PhelanDev)**.
