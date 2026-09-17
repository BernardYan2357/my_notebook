import cv2
import numpy as np
import json
import os

class CameraCalibrationViewer:
    def __init__(self, calibration_file, save_dir=None):
        """
        初始化相机校准查看器
        Args:
            calibration_file: 保存相机参数的JSON文件路径
            save_dir: 保存图像的目录路径,如果为None则不保存
        """
        self.calibration_file = calibration_file
        self.save_dir = save_dir
        self.camera_matrix = None
        self.dist_coeffs = None
        self.cap = None
        # 加载相机参数
        self.load_calibration_parameters()
        
    def load_calibration_parameters(self):
        """从JSON文件加载相机标定参数"""
        try:
            with open(self.calibration_file, 'r') as f:
                params = json.load(f)
            self.camera_matrix = np.array(params['camera_matrix'])
            self.dist_coeffs = np.array(params['distortion_coefficients'])
            print("相机参数加载成功!")
            print("内参矩阵:")
            print(self.camera_matrix)
            print("畸变系数:")
            print(self.dist_coeffs.ravel())
        except Exception as e:
            print(f"加载相机参数失败: {e}")
            raise
    
    def initialize_camera(self, camera_id=0):
        """
        初始化摄像头
        Args:
            camera_id: 摄像头ID (默认0)
        """
        self.cap = cv2.VideoCapture(camera_id)
        if not self.cap.isOpened():
            raise RuntimeError(f"无法打开摄像头 {camera_id}")
        # 获取实际分辨率
        width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = self.cap.get(cv2.CAP_PROP_FPS)
        print(f"摄像头初始化成功: {width}x{height}, {fps} FPS")
        return width, height
    
    def undistort_frame(self, frame):
        """
        对帧进行去畸变处理
        Args:
            frame: 原始帧
        Returns:
            undistorted_frame: 去畸变后的帧
        """
        h, w = frame.shape[:2]
        # 获取最优新相机矩阵和ROI
        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
            self.camera_matrix, self.dist_coeffs, (w, h), 1, (w, h))
        # 去畸变
        undistorted = cv2.undistort(frame, self.camera_matrix, 
                                  self.dist_coeffs, None, new_camera_matrix)
        # 裁剪图像到有效区域
        x, y, w, h = roi
        undistorted = undistorted[y:y+h, x:x+w]
        return undistorted
    
    def run(self, camera_id=0):
        """
        运行摄像头显示
        Args:
            camera_id: 摄像头ID
        """
        try:
            # 初始化摄像头
            width, height = self.initialize_camera(camera_id)
            print("开始显示摄像头画面...")
            print("按 'q' 键退出")
            print("按 's' 键保存当前帧")
            save_count = 0
            while True:
                # 读取帧
                ret, frame = self.cap.read()
                if not ret:
                    print("无法读取摄像头帧")
                    break
                # 去畸变处理
                undistorted_frame = self.undistort_frame(frame)
                # 调整去畸变帧的大小以匹配原始帧
                undistorted_resized = cv2.resize(undistorted_frame, (width, height))
                # 添加文字说明
                cv2.putText(frame, "Original (Distorted)", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(undistorted_resized, "Undistorted (Corrected)", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                # 水平拼接显示
                combined = np.hstack((frame, undistorted_resized))
                # 显示结果
                cv2.imshow("Camera Calibration - Original (Left) vs Undistorted (Right)", combined)
                # 键盘输入处理
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('s'):
                    # 保存当前帧
                    save_count += 1
                    if self.save_dir:
                        # 确保保存目录存在
                        os.makedirs(self.save_dir, exist_ok=True)
                        original_path = os.path.join(self.save_dir, f"original_frame_{save_count}.jpg")
                        undistorted_path = os.path.join(self.save_dir, f"undistorted_frame_{save_count}.jpg")
                        cv2.imwrite(original_path, frame)
                        cv2.imwrite(undistorted_path, undistorted_resized)
                        print(f"帧已保存到 {self.save_dir}: original_frame_{save_count}.jpg, undistorted_frame_{save_count}.jpg")
                    else:
                        # 如果没有指定保存目录，保存到当前目录
                        cv2.imwrite(f"original_frame_{save_count}.jpg", frame)
                        cv2.imwrite(f"undistorted_frame_{save_count}.jpg", undistorted_resized)
                        print(f"帧已保存: original_frame_{save_count}.jpg, undistorted_frame_{save_count}.jpg")
        except Exception as e:
            print(f"运行过程中出错: {e}")
        finally:
            # 释放资源
            if self.cap:
                self.cap.release()
            cv2.destroyAllWindows()

def main():
    """主函数"""
    # 参数设置
    CAMERA_ID = 1
    CALIBRATION_FILE = "./camera_calib/camera_parameters.json"  # 相机参数文件路径
    SAVE_DIR = "./camera_calib/saved_img"  # 保存图像的目录
    # 检查参数文件是否存在
    if not os.path.exists(CALIBRATION_FILE):
        print(f"错误: 找不到相机参数文件 {CALIBRATION_FILE}")
        print("请先运行相机标定程序生成参数文件")
        return
    # 创建并运行查看器
    try:
        viewer = CameraCalibrationViewer(CALIBRATION_FILE, save_dir=SAVE_DIR)
        viewer.run(camera_id=CAMERA_ID)
    except Exception as e:
        print(f"程序运行失败: {e}")

if __name__ == "__main__":
    main()
