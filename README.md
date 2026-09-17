CHƯƠNG 1. Cở sở lý thuyết
	1. SPI
- SPI là viết tắt của Serial Peripheral Interface. Đây là một giao thức truyền thông nối tiếp đồng bộ. Nó được sử dụng để giao tiếp giữa các thiết bị ngoại vi, tức là thiết bị đầu vào, đầu ra và vi điều khiển. Nó được phép truyền dữ liệu tốc độ cao. Nó được ưa chuộng trong các ứng dụng truyền thông số và hệ thống nhúng. SPI có thể truyền dữ liệu và nhận dữ liệu từ một thiết bị sang thiết bị khác tại một thời điểm. 
- Giao diện ngoại vi nối tiếp nối tiếp (SPI) là quá trình giao thức truyền thông nối tiếp đồng bộ. Nó chủ yếu được sử dụng để kết nối vi điều khiển với các thiết bị ngoại vi như cảm biến, màn hình và chip nhớ. Nó tạo điều kiện cho giao tiếp nối tiếp đồng bộ full-duplex giữa một hoặc nhiều thiết bị slave và vi điều khiển. 
- Các thành phần của SPI:
    • Thiết bị chủ: Thiết bị chính không có gì ngoài việc kiểm soát quá trình chuyển đổi dữ liệu trên bus SPI. 
    • Thiết bị Slave: Thiết bị Slave là các thiết bị ngoại vi được kết nối với bus SPI và được điều khiển bởi các thiết bị chủ. Mỗi thiết bị slave có một dòng chọn slave (SS) khác nhau, cho phép master chọn thiết bị mà nó muốn giao tiếp.
    • SPI Bus: SPI bus là một kết nối vật lý qua việc truyền dữ liệu giữa các thiết bị slave và thiết bị master. Nó có bốn đường tín hiệu như dưới đây:
        ◦ Slave Select (SS): Trong Slave Select, mỗi thiết bị slave có một chân SS chuyên dụng. Nếu chủ nhân sẽ giao tiếp với slave cụ thể. Nhiều thiết bị slave có thể được chia sẻ với các đường giống như MOSI, MISO và SCK nhưng phải có các đường SS riêng biệt.
        ◦ Master Out Slave In (MOSI): Trong Master Out Slave In, MOSI có thể chia sẻ dữ liệu hoặc thông tin từ master sang các thiết bị slave khác.
        ◦ Master In Slave Out (MISO): Trong Master In Slave Out, MISO có thể chia sẻ dữ liệu hoặc thông tin từ thiết bị slave với master.
        ◦ Xung nhịp nối tiếp (SCK): Trong Đồng hồ nối tiếp, tín hiệu đồng hồ này được sử dụng bởi thiết bị chính và thiết bị phụ để phối hợp thời gian truyền dữ liệu.
- Tốc độ dữ liệu: Bus SPI có thể hỗ trợ các tốc độ truyền dữ liệu khác nhau tùy thuộc vào khả năng master của thiết bị slave và chiều dài đường truyền. Tốc độ dữ liệu được xác định bằng bit trên megahertz (MHz) hoặc giây (bps).
	2. I2C
- I2C là viết tắt của Inter-Integrated Circuit. Đây là một giao thức kết nối giao diện bus được tích hợp vào các thiết bị để truyền thông nối tiếp.
- Nó chỉ sử dụng 2 đường thoát nước hở hai chiều cho truyền thông dữ liệu gọi là SDA và SCL. Cả hai đường này đều được kéo cao. 
- Các bước truyền dữ liệu I2C
    • Điều kiện khởi động: Thiết bị chính gửi điều kiện bắt đầu bằng cách kéo đường SDA thấp khi đường SCL ở mức cao. Điều này báo hiệu rằng một đợt truyền tin sắp bắt đầu.
    • Địa chỉ slave: Master gửi địa chỉ 7-bit của thiết bị slave mà nó muốn giao tiếp, tiếp theo là một bit đọc/ghi. Bit đọc/ghi cho biết nó muốn đọc từ hay ghi vào slave.
    • Bit xác nhận (ACK): Thiết bị slave được địa chỉ phản hồi bằng cách kéo đường SDA xuống thấp trong xung nhịp tiếp theo (SCL). Điều này xác nhận rằng nô lệ đã sẵn sàng giao tiếp.
    • Truyền dữ liệu: Bộ chủ hoặc phụ (tùy thuộc vào thao tác đọc/ghi) gửi dữ liệu thành các đoạn 8-bit. Sau mỗi byte, một ACK được gửi để xác nhận rằng dữ liệu đã được nhận thành công.
    • Điều kiện dừng: Khi quá trình truyền hoàn tất, master gửi điều kiện dừng bằng cách thả đường SDA lên cao trong khi đường SCL ở vị trí cao. Điều này báo hiệu rằng phiên giao tiếp đã kết thúc.
	3. DMA
- Trong các hệ thống máy tính hiện đại, việc truyền dữ liệu giữa các thiết bị nhập/xuất (I/O) và bộ nhớ có thể làm chậm hiệu suất nếu CPU xử lý mọi bước. Để khắc phục điều này, một Bộ điều khiển Truy cập Bộ nhớ Trực tiếp (DMA) được sử dụng. Nó cho phép các thiết bị I/O truyền dữ liệu trực tiếp đến hoặc từ bộ nhớ mà không cần CPU sử dụng nhiều, cải thiện tốc độ và hiệu quả. 
- Vai trò:
    • Giảm tải cho CPU: CPU không cần phải thực hiện các lệnh sao chép dữ liệu thủ công
    • Tăng tốc độ truyền dữ liệu: DMA hoạt động trực tiếp trên bus hệ thống với tốc độ cao, không bị gián đoạn bởi các tác vụ khác
    • Tiết kiệm năng lượng: CPU hoạt động ít hơn
    • Cho phép xử lý tác vụ xử lý song song: Trong khi DMA đang vận chuyển dữ liệu, CPU có thể tập chung xử lý các thuật toán phức tạp khác, tối ưu được thời gian chạy chương trình tổng thể				
- Thiết bị ngoại vi gửi yêu cầu đến bộ điều khiển DMA để bắt đầu truyền dữ liệu.
- Bộ điều khiển DMA kiểm soát bus bộ nhớ của hệ thống và truy cập bộ nhớ trực tiếp, hoặc đọc dữ liệu từ đó hoặc ghi dữ liệu vào đó.
- Sau khi quá trình truyền hoàn tất, bộ điều khiển DMA sẽ báo hiệu cho CPU rằng tác vụ đã hoàn thành, và CPU có thể tiếp tục các tác vụ khác.
- Các loại DMA:

  +DMA Chế độ Burst 	    • Ở chế độ này, bộ điều khiển DMA kiểm soát bus bộ nhớ và truyền một khối dữ liệu chỉ trong một lần.
                          • CPU tạm thời bị khóa truy cập bộ nhớ trong khi bộ điều khiển DMA hoàn tất quá trình truyền dữ liệu.
  +Cycle Stealing DMA 	  • Ở chế độ này, bộ điều khiển DMA truyền từng mục dữ liệu một lần nhưng cho phép CPU truy cập bộ nhớ giữa mỗi lần truyền.
                          • Điều này cho phép CPU và bộ điều khiển DMA chia sẻ bus bộ nhớ và làm việc hợp tác hơn.
  +Block Mode DMA 	        • Bộ điều khiển DMA truyền một khối dữ liệu mà không bị gián đoạn, nhưng sử dụng phương pháp tổ chức hơn so với chế độ bùng nổ, cho phép truyền tải hiệu quả hơn.
                          • CPU bị khóa truy cập bộ nhớ trong quá trình truyền.


  +Demand Mode DMA 	      • Ở chế độ này, bộ điều khiển DMA chỉ truyền dữ liệu khi CPU không sử dụng bus bộ nhớ, về cơ bản là chờ thời gian rảnh để thực hiện chuyển dữ liệu.
	4. State Machine
- State Machine là một cách để mô tả một hệ thống chỉ có thể ở một trong số các tình huống (trạng thái) tại một thời điểm, và nó chỉ chuyển sang tình huống khác khi có một điều kiện cụ thể xảy ra. 
- Cần có ba thành phần cơ bản để xây dựng một máy trạng thái:
    • Actions: Là các sự kiện hoặc thay đổi xảy ra trong suốt quá trình thực thi của một máy trạng thái. 
    • Transitions: Là cách thức để di chuyển giữa các trạng thái khác nhau. 
    • States:  Là các bước, nhiệm vụ hoặc chế độ hoạt động khác nhau để phân biệt các giai đoạn của quy trình. 
CHƯƠNG 2. Lập trình theo datasheet
- ICM20948 là cảm biến chuyển động 9 trục (9-DOF) tích hợp
    • Accelerometre (Gia tốc kế): 3 trục ( ± 2g, ±4g, ±8g, ±16g).
    • Gyroscope (Con quay hồi chuyển): 3 trục (±250, ±500, ±1000, ±2000 dps).
    • Magnetometer(Từ trường kế): Dải độ rộng lên tới ±4900 uT.
- Xử lý chuyển động: tích hợp bộ xử lý chuyển động kỹ thuật số Onboard Digital Motion Processor (DMP).
- Hệ điều hành: Android.
- Giao tiếp phụ: Giao diện giao tiếp I2C dùng để mở rộng kết nối với các cảm biến bên ngoài.
- Chuyển đổi tín hiệu & bộ lọc: tích hợp ADC 16-bit On-chip và các bộ lọc lập trình.
- Chuẩn giao tiếp: SPI tốc độ cao lên tới 7 Mhz hoặc I2C tốc độ cao lên tới 400kHz.
- Điện áp hoạt động(VDD): Trong khoảng từ 1.71V đến 3.6V
CHƯƠNG 4. KẾT LUẬN
	1. Những kiến thức đạt được
- Về mặt giao thức truyền thống:
    • Hiểu được nguyên lý hoạt động của SPI, bao gồm các tín hiệu cơ bản: SCK, MOSI, MISO, SS và cơ chế hoạt động của full-deplex
    • Nắm thêm được kiến thức về chuẩn giao tiếp I2C, đặc biệt về cách thức truyền dữ liệu với 2 đường SDA và SCL, điều kiện Start/Stop, cũng như cơ chế ACK trong từng khung dữ liệu
- Về mặt tối ưu hiệu năng hệ thống
    • Hiểu được vai trò quan trọng của DMA trong việc giảm tải cho CPU khi xử lý các tác vụ truyền dữ liệu lặp đi lặp lại. DMA giúp hệ thống vận hành mượt mà hơn, cho phép CPU tập chung xử lý các thuật toán phức tạp trong khi dữ liệu đang được truyền ngầm ở phía sau
- Về thiết kế hệ thống Nhúng
    • Nắm thêm được kiến thức về mô hình State Machine và cách áp dụng vào bài toàn điều khiển, giúp logic chương trình trở nên rõ ràng, có cấu trúc và dễ dàng mở rộng, bảo trì sau này
    • Nắm ở mức ổn kiến thức tổng quan về ICM20948, biết được cấu trúc tổng quan gồm gia tốc kế, con quay hồi chuyển và từ trường kế, cũng như cách hiệu chỉnh sai số tùy thuộc vào môi trường sử dụng
	2. Những khó khăn gặp phải  
- Về lý thuyết
    • Các cơ chế hoạt động của DMA còn khó trừu tượng, cần thời gian để khám phá sâu vào cách CPU tương tác với bộ điều khiển DMA cũng như hình dung được luồng dữ liệu
    • Việc đọc datasheet của ICM20948 gặp nhiều khó khăn do tài liệu dài, nhiều thuật ngữ chuyên ngành và thông số chi tiết
CHƯƠNG 5. NGUỒN TÀI LIỆU THAM KHẢO
	1. SPI
Link: What is Serial Peripheral Interface (SPI)? – GeeksforGeeks
	2. I2C
Link: I2C Communication Protocol – GeeksforGeeks
	3. DMA
Link: Direct Memory Access (DMA) Controller – GeeksforGeeks
	4. STATE MACHINE
Link: State Machine Diagrams | Unified Modeling Language (UML) - GeeksforGeeks 
	5. THIẾT KẾ ICM20948
Link: Lập Trình STM32 #15 | Giao tiếp cảm biến IMU bằng SPI - Phần 1
Link: Lập Trình STM32 #16 | Giao tiếp cảm biến IMU bằng SPI - Phần 2
Link: Lập Trình STM32 #17 | Đọc dữ liệu IMU non-blocking với DMA + State Machine - Phần 3
Link: Lập Trình STM32 #17.1 | Hiệu Chỉnh Gyroscope ICM-20948 (Bias Calibration) - Phần 4
	6. DATASHEET
Link: ICM-20948-datasheet.pdf


