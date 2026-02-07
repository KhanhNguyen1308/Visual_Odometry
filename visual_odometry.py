import numpy as np
import cv2

class VisualOdometry:
    def __init__(self, camera_matrix):
        """
        Khởi tạo hệ thống VO
        :param camera_matrix: Ma trận nội tham camera (Intrinsic Matrix) 3x3
        """
        self.k = camera_matrix
        self.focal = self.k[0, 0]
        self.pp = (self.k[0, 2], self.k[1, 2])
        
        # Sử dụng FAST detector cho tốc độ cao trên Jetson Nano
        self.detector = cv2.FAST_create(threshold=20, nonmaxSuppression=True)
        
        # Tham số cho Optical Flow (Lucas-Kanade)
        self.lk_params = dict(winSize=(21, 21), 
                              criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))

        self.prev_frame = None
        self.prev_pts = None
        
        # Vị trí hiện tại của Robot (Khởi tạo tại 0,0,0)
        self.cur_R = np.eye(3)
        self.cur_t = np.zeros((3, 1))
        
        # Ngưỡng số lượng điểm đặc trưng tối thiểu để tìm lại
        self.min_features = 1000

    def process_frame(self, frame):
        # 1. Chuyển sang ảnh xám
        if frame.ndim > 2:
            frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        else:
            frame_gray = frame

        # 2. Khởi tạo frame đầu tiên
        if self.prev_frame is None:
            self.prev_frame = frame_gray
            # Tìm các điểm đặc trưng (Keypoints)
            kps = self.detector.detect(frame_gray, None)
            self.prev_pts = np.array([x.pt for x in kps], dtype=np.float32).reshape(-1, 1, 2)
            return self.cur_t

        # 3. Theo dõi điểm ảnh bằng Optical Flow
        # Tính toán vị trí các điểm cũ trong frame mới
        curr_pts, status, err = cv2.calcOpticalFlowPyrLK(self.prev_frame, frame_gray, self.prev_pts, None, **self.lk_params)

        # Lọc bỏ các điểm không theo dõi được
        status = status.reshape(-1)
        good_prev = self.prev_pts[status == 1]
        good_curr = curr_pts[status == 1]

        # 4. Tính toán chuyển động (Essential Matrix)
        if len(good_prev) < 5:
            return self.cur_t # Không đủ điểm để tính toán

        E, mask = cv2.findEssentialMat(good_curr, good_prev, self.focal, self.pp, cv2.RANSAC, 0.999, 1.0)
        
        if E is not None:
            # Khôi phục tư thế (Pose Recovery) -> Trả về R (Xoay) và t (Dịch chuyển)
            _, R, t, mask = cv2.recoverPose(E, good_curr, good_prev, focal=self.focal, pp=self.pp)
            
            # --- QUAN TRỌNG: XỬ LÝ TỈ LỆ (SCALE) ---
            # Với Monocular, 't' chỉ là hướng (độ lớn = 1). 
            # Bạn CẦN lấy tốc độ thật từ Encoder bánh xe.
            # Ở đây tôi giả định tốc độ cố định (Scale = 1) để demo.
            absolute_scale = 1.0 
            
            # Kiểm tra điều kiện để tránh nhiễu khi đứng yên
            if (absolute_scale > 0.1):
                self.cur_t = self.cur_t + absolute_scale * self.cur_R.dot(t)
                self.cur_R = self.cur_R.dot(R)

        # 5. Cập nhật cho vòng lặp sau
        # Nếu số lượng điểm theo dõi xuống thấp, tìm lại điểm mới
        if len(good_prev) < self.min_features:
            kps = self.detector.detect(frame_gray, None)
            new_pts = np.array([x.pt for x in kps], dtype=np.float32).reshape(-1, 1, 2)
            self.prev_pts = np.concatenate((good_curr.reshape(-1, 1, 2), new_pts), axis=0)
        else:
            self.prev_pts = good_curr.reshape(-1, 1, 2)
            
        self.prev_frame = frame_gray
        
        return self.cur_t

# --- CHƯƠNG TRÌNH CHÍNH ---
if __name__ == "__main__":
    # 1. CẤU HÌNH CAMERA (QUAN TRỌNG NHẤT)
    # Bạn phải thay số này bằng thông số camera của bạn (Calibrate Camera)
    # Ví dụ dưới đây là thông số giả định
    focal_length = 718.8560
    center_x = 607.1928
    center_y = 185.2157
    
    camera_matrix = np.array([[focal_length, 0, center_x],
                              [0, focal_length, center_y],
                              [0, 0, 1]])

    vo = VisualOdometry(camera_matrix)

    # 2. MỞ CAMERA HOẶC VIDEO
    # Nếu dùng Camera USB: cap = cv2.VideoCapture(0)
    # Nếu dùng Camera CSI (Jetson): Cần chuỗi GStreamer (phức tạp hơn)
    # Ở đây test bằng video file cho dễ:
    cap = cv2.VideoCapture('test_video.mp4') 
    
    # Tạo map trống để vẽ đường đi
    traj = np.zeros((600, 600, 3), dtype=np.uint8)

    while(cap.isOpened()):
        ret, frame = cap.read()
        if not ret: break

        # Gọi hàm xử lý
        t = vo.process_frame(frame)
        
        # Vẽ điểm hiện tại lên bản đồ (Trajectory)
        # Cộng thêm 300 để ra giữa hình
        x, y = int(t[0]) + 300, int(t[2]) + 100 
        cv2.circle(traj, (x, y), 1, (0, 255, 0), 1)

        # Hiển thị
        cv2.imshow('Camera View', frame)
        cv2.imshow('Trajectory (Map)', traj)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()