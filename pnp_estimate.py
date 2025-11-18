import cv2
import numpy as np

def get_aruco_world_coordinates(marker_size=0.096):
    """
    返回ArUco标记四个角点的世界坐标(以标记中心为原点)
    Args:
        marker_size: 标记边长
    Returns:
        world_points: 世界坐标系下的四个角点坐标，形状为(4,3)的numpy数组
                      顺序为：左上、右上、右下、左下
    """
    half_size = marker_size / 2
    # 世界坐标（右手系，Z轴垂直标记平面向外）
    world_points = np.array([
        [-half_size, half_size, 0],   # 左上
        [half_size, half_size, 0],    # 右上  
        [half_size, -half_size, 0],   # 右下
        [-half_size, -half_size, 0]   # 左下
    ], dtype=np.float32)
    return world_points

def detect_aruco_corners(frame, aruco_dict=cv2.aruco.DICT_4X4_50):
    """
    检测图像中的ArUco标记四个角点
    Args:
        frame: 输入图像帧
        aruco_dict: ArUco字典类型, 默认为4x4_50
    Returns:
        corners: 四个角点的像素坐标，形状为(1,4,2)的numpy数组
                 顺序为：左上、右上、右下、左下
                 如果未检测到则返回None
    """
    # 获取ArUco字典
    aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict)
    # 创建ArUco检测参数
    parameters = cv2.aruco.DetectorParameters()
    # 创建检测器
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
    # 检测ArUco标记
    corners, ids, _ = detector.detectMarkers(frame)
    # 如果检测到标记，返回第一个标记的四个角点
    if len(corners) > 0:
        return corners[0]
    return None

def solve_pnp_pose(corners, camera_matrix, dist_coeffs, marker_size=0.096):
    """
    使用PnP算法求解相机相对于标记的位姿
    Args:
        corners: 标记四个角点的像素坐标
        camera_matrix: 相机内参矩阵
        dist_coeffs: 畸变系数
        marker_size: 标记边长
    Returns:
        rvec: 旋转向量
        tvec: 平移向量
    """
    # 获取世界坐标点
    world_points = get_aruco_world_coordinates(marker_size)
    # 使用PnP求解位姿
    success, rvec, tvec = cv2.solvePnP(
        world_points, 
        corners, 
        camera_matrix, 
        dist_coeffs, 
        flags=cv2.SOLVEPNP_IPPE_SQUARE
    )
    if success:
        return rvec, tvec
    else:
        return None, None

def draw_pose_axes(frame, camera_matrix, dist_coeffs, rvec, tvec, length=0.07):
    """
    在图像上绘制坐标系轴，显示相机的位姿
    Args:
        frame: 输入图像帧
        camera_matrix: 相机内参矩阵
        dist_coeffs: 畸变系数
        rvec: 旋转向量
        tvec: 平移向量
        length: 坐标轴长度
    """
    # 定义坐标系轴的点（世界坐标系）
    axis_points = np.float32([
        [0,      0,           0],
        [length, 0,           0],
        [0,      length,      0],
        [0,      0,      length]
    ])
    # 将世界坐标系中的点投影到图像平面
    img_points, _ = cv2.projectPoints(axis_points, rvec, tvec, camera_matrix, dist_coeffs)
    img_points = img_points.reshape(-1, 2)
    # 绘制坐标轴
    origin = tuple(img_points[0].astype(int))
    x_axis = tuple(img_points[1].astype(int))
    y_axis = tuple(img_points[2].astype(int))
    z_axis = tuple(img_points[3].astype(int))
    # 绘制X轴（红色）
    cv2.arrowedLine(frame, origin, x_axis, (0, 0, 255), 3)
    # 绘制Y轴（绿色）
    cv2.arrowedLine(frame, origin, y_axis, (0, 255, 0), 3)
    # 绘制Z轴（蓝色）
    cv2.arrowedLine(frame, origin, z_axis, (255, 204, 0), 3)
    # 计算相机在标记坐标系中的位置（tvec的逆变换）
    R, _ = cv2.Rodrigues(rvec)
    camera_pos_in_marker = -R.T @ tvec
    # 在图像左上角显示相机位置信息
    cv2.putText(frame, f"Camera Coordinates:", (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.putText(frame, f"X: {camera_pos_in_marker[0][0]:.3f}m", (10, 60), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    cv2.putText(frame, f"Y: {camera_pos_in_marker[1][0]:.3f}m", (10, 90), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.putText(frame, f"Z: {camera_pos_in_marker[2][0]:.3f}m", (10, 120), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 204, 0), 2)

def main():
    is_save=False
    save_path="./images/pnp.mp4"
    # 打开摄像头
    cap = cv2.VideoCapture(0)
    # 获取视频的帧率和尺寸
    fps = int(cap.get(cv2.CAP_PROP_FPS))
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    # 相机内参矩阵
    camera_matrix=np.array([
        [609.139547919527,0.0,              336.0685341407001 ],
        [0.0,             608.5126119093808,194.25692564931205],
        [0.0,             0.0,              1.0               ]
    ],dtype=np.float32)
    # 畸变系数（需要根据实际相机标定结果修改）
    dist_coeffs=np.array([
            -0.5004858125592277,
            0.412601310630979,
            0.0012856558248768365,
            -0.0011703164301384594,
            -0.39732492871531927
        ],dtype=np.float32)
    # 创建 VideoWriter 对象，保存处理后的视频
    if is_save:
        fourcc = cv2.VideoWriter_fourcc(*'H264')
        out = cv2.VideoWriter(save_path, fourcc, fps, (width, height))
    # 主循环
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        # 检测ArUco标记角点
        corners = detect_aruco_corners(frame)
        # 如果检测到角点，计算位姿并绘制
        if corners is not None:
            # 绘制检测到的标记
            cv2.aruco.drawDetectedMarkers(frame, [corners])
            # 使用PnP求解位姿
            rvec, tvec = solve_pnp_pose(corners, camera_matrix, dist_coeffs)
            # 如果成功求解位姿，绘制坐标系
            if rvec is not None and tvec is not None:
                draw_pose_axes(frame, camera_matrix, dist_coeffs, rvec, tvec)
        # 保存
        if is_save :
            out.write(frame)
        # 显示结果
        cv2.imshow('ArUco Marker Pose Estimation', frame)
        # 按'q'退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    # 释放资源
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()