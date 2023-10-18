mô tả bộ dữ liệu:
Deport nằm ở gốc toạ độ (0,0)
Các giá trị đã được đổi về đơn vị thống nhất: 
    đơn vị khoảng cách tính theo m
    đơn vị thời gian tính theo s
Thông số của xe tải:
    Nằm trong file Truck_config.json 
    Giờ làm việc tính từ thời điểm 0, các khung giờ và hệ số trơn nằm trong mảng T
Thông số của drone:
    drone_linear_config.json: 
        Khi sử dụng công thức tiêu hao năng lượng tuyến tính sử dụng các tham số này
        Có 4 trường hợp theo speed và range
    drone_nonelinearconfig.json: 
        Khi sử dụng công thức tiêu hao năng lượng phi tuyến sử dụng các giá trị này
        Có 4 trường hợp Có 4 trường hợp theo speed và range
    drone_endurance_model.json
        Khi năng lượng của drone bị giới hạn theo 1 trong hai giá trị: 
            FixedTime (Khi drone đợi sẽ tính vào tiêu hao năng lượng)
            FixedDistance (coi như chỉ bị giới hạn bởi khoảng cách bay)

