import os
import numpy as np
import cv2
import glob
import json
from typing import Tuple, Dict, List

class CameraCalibrator:
    def __init__(self, chessboard_size: Tuple[int, int], square_size: float):
        """
        初始化相机标定器
        Args:
            chessboard_size: 棋盘格内部角点数量 (width, height)
            square_size: 每个棋盘格方格的物理尺寸(米)
        """
        self.chessboard_size = chessboard_size
        self.square_size = square_size
        
    def prepare_object_points(self) -> np.ndarray:
        """
        准备世界坐标系中的角点坐标
        Returns:
            世界坐标系中的角点坐标数组
        """
        w, h = self.chessboard_size
        # 创建角点的世界坐标 (z=0)
        objp = np.zeros((w * h, 3), np.float32)
        # np.mgrid[0:w, 0:h]返回一个2xwxh的数组，然后通过.T转置为hxwx2，再reshape(-1,2)变成(w*h, 2)的数组。这样，每一行就是一个角点的(x, y)坐标，其中x从0到w-1，y从0到h-1
        objp[:, :2] = np.mgrid[0:w, 0:h].T.reshape(-1, 2)
        # 将网格坐标转换为物理坐标
        return objp * self.square_size
    
    def find_chessboard_corners(self, image_dir: str, image_type: str = "jpg", 
                                show_corners: bool = True, save_dir: str = None) -> Tuple[List, List]:
        """
        在图像中查找棋盘格角点
        Args:
            image_dir: 图像文件夹路径
            image_type: 图像文件类型
            show_corners: 是否显示角点检测结果
            save_dir: 保存角点检测结果的文件夹
        Returns:
            obj_points: 世界坐标系点列表
            img_points: 图像坐标系点列表
        """
        # 准备世界坐标系点
        objp = self.prepare_object_points()
        # 初始化存储列表
        obj_points = []  # 3D点在世界坐标系中
        img_points = []  # 2D点在图像平面中
        # 获取所有图像文件
        images = glob.glob(os.path.join(image_dir, f"*.{image_type}"))
        if not images:
            raise ValueError(f"No images found in {image_dir} with type {image_type}")
        print(f"Found {len(images)} images for calibration")
        # 遍历图像文件
        for fname in images:
            # 读取图像
            img = cv2.imread(fname)
            if img is None:
                print(f"Warning: Could not read {fname}")
                continue
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            # 查找棋盘格角点
            ret, corners = cv2.findChessboardCorners(gray, self.chessboard_size, None)
            if ret: # 找到角点
                # 提高角点检测精度
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                # 存储点
                obj_points.append(objp)
                img_points.append(corners_refined)
                # 显示和保存角点检测结果
                img_with_corners = img.copy()
                cv2.drawChessboardCorners(img_with_corners, self.chessboard_size, corners_refined, ret)
                # 显示角点检测结果
                if show_corners:
                    cv2.imshow('Chessboard Corners', img_with_corners)
                    cv2.waitKey(1000)  # 显示1秒
                # 保存角点检测结果
                if save_dir:
                    os.makedirs(save_dir, exist_ok=True)
                    output_path = os.path.join(save_dir, f"corners_{os.path.basename(fname)}")
                    cv2.imwrite(output_path, img_with_corners)
            else:
                print(f"Warning: Chessboard corners not found in {fname}")
        if show_corners:
            cv2.destroyAllWindows()
        if len(obj_points) < 5:
            raise ValueError(f"Only found {len(obj_points)} valid images. Need at least 5 for calibration.")
        print(f"Successfully detected corners in {len(obj_points)} images")
        return obj_points, img_points
    
    def calibrate_camera(self, image_dir: str, image_type: str = "jpg", save_dir: str = None) -> Dict:
        """
        执行相机标定，计算内外参数
        Args:
            image_dir: 图像文件夹路径
            image_type: 图像文件类型
            save_dir: 保存角点检测结果的文件夹
        Returns:
            包含相机参数的字典
        """
        # 获取角点
        obj_points, img_points = self.find_chessboard_corners(image_dir, image_type, save_dir=save_dir)
        # 获取图像尺寸
        sample_image = cv2.imread(glob.glob(os.path.join(image_dir, f"*.{image_type}"))[0])
        image_size = (sample_image.shape[1], sample_image.shape[0])  # (width, height)
        # 相机标定
        print("Performing camera calibration...")
        ret, self.camera_matrix, self.dist_coeffs, self.rvecs, self.tvecs = cv2.calibrateCamera(
            obj_points, img_points, image_size, None, None)
        if not ret:
            raise RuntimeError("Camera calibration failed")
        # 计算重投影误差
        self.calibration_error = self._compute_reprojection_error(obj_points, img_points)
        print("Camera calibration completed successfully!")
        print(f"Calibration error: {self.calibration_error:.6f}")
        # 返回相机参数
        return self._get_camera_parameters()
    
    def _compute_reprojection_error(self, obj_points: List, img_points: List) -> float:
        """计算平均重投影误差"""
        total_error = 0
        for i in range(len(obj_points)):
            img_points_repro, _ = cv2.projectPoints(
                obj_points[i], self.rvecs[i], self.tvecs[i], 
                self.camera_matrix, self.dist_coeffs)
            error = cv2.norm(img_points[i], img_points_repro, cv2.NORM_L2) / len(img_points_repro)
            total_error += error
        return total_error / len(obj_points)
    
    def _get_camera_parameters(self) -> Dict:
        """获取相机参数字典"""
        return {
            'camera_matrix': self.camera_matrix.tolist(), # 相机内参矩阵
            'distortion_coefficients': self.dist_coeffs.tolist(), # 畸变系数 [k1, k2, p1, p2, k3]
            # 'rotation_vectors': [rvec.tolist() for rvec in self.rvecs], # 棋盘格图像的旋转向量
            # 'translation_vectors': [tvec.tolist() for tvec in self.tvecs],  # 棋盘格图像的平移向量
            'reprojection_error': self.calibration_error, # 重投影误差
            'chessboard_size': self.chessboard_size, # 标定板内部角点的行列数
            'square_size': self.square_size # 每个棋盘格方格的物理尺寸
        }
    
    def save_parameters(self, file_path: str):
        """保存相机参数到JSON文件"""
        if self.camera_matrix is None:
            raise ValueError("Camera not calibrated yet. Call calibrate_camera() first.")
        # 获取参数字典
        parameters = self._get_camera_parameters()
        with open(file_path, 'w') as f:
            json.dump(parameters, f, indent=4)
        print(f"Camera parameters saved to {file_path}")

def main():
    """主函数"""
    # 参数设置
    CHESSBOARD_SIZE = (8,5) # 棋盘格内部角点数量 (width, height)
    SQUARE_SIZE = 0.027 # 每个方格的物理尺寸(米)
    CALIB_IMAGE_DIR = "./camera_calib/calib" # 标定图像文件夹
    CORNER_IMAGE_DIR = None # 角点保存路径
    OUTPUT_DIR = "./camera_calib" # 输出文件夹
    # 创建输出文件夹
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    try:
        # 相机标定
        calibrator = CameraCalibrator(CHESSBOARD_SIZE, SQUARE_SIZE)
        # 执行标定
        camera_params = calibrator.calibrate_camera(CALIB_IMAGE_DIR, "jpg", CORNER_IMAGE_DIR)
        # 保存相机参数
        calibrator.save_parameters(os.path.join(OUTPUT_DIR, "camera_parameters.json"))
        # 打印相机内参
        print("\n=== Camera Intrinsic Parameters ===")
        print("Camera Matrix:")
        print(calibrator.camera_matrix)
        print("\nDistortion Coefficients:")
        print(calibrator.dist_coeffs.ravel())
        print(f"\nFinal Results:")
        print(f"Calibration Error: {calibrator.calibration_error:.6f}")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()