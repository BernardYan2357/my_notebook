import cv2
import numpy as np
import time

def track_aruco(frame, aruco_dict):
    aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
    corners, ids, _ = detector.detectMarkers(frame)
    centers = []
    if corners is None or len(corners) == 0:
        return None, None, None
    for c in corners:
        pts = np.asarray(c).reshape(-1, 2)
        cx = int(np.mean(pts[:, 0]))
        cy = int(np.mean(pts[:, 1]))
        centers.append((cx, cy))
    return corners, ids, centers

def predict_aruco(centers, ids, kalman, missed_count):
    # 返回 (prediction, missed_count, corrected_flag)
    prediction = kalman.predict()
    # prediction 形状通常为 (4,1) 或可索引为 prediction[0,0], prediction[1,0]
    if ids is not None and centers:
        measurement = np.array([[np.float32(centers[0][0])], [np.float32(centers[0][1])]])
        corrected = kalman.correct(measurement)
        missed_count = 0
        return corrected, missed_count, True
    else:
        missed_count += 1
        return prediction, missed_count, False

# 绘制检测框，预测框和中心点
def draw_aruco(frame, corners, centers, prediction, ids, latest_detected_box):
    if ids is not None and corners is not None:
        for i, corner in enumerate(corners):
            corner_points = corner[0].astype(np.int32)
            x, y, w, h = cv2.boundingRect(corner_points)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            if centers and i < len(centers):
                cv2.circle(frame, (centers[i][0], centers[i][1]), 5, (255, 0, 0), -1)
            latest_detected_box = (w, h)
    # prediction 可以是 predict() 或 correct() 的返回
    if prediction is not None:
        pred_x = int(prediction[0, 0])
        pred_y = int(prediction[1, 0])
        cv2.circle(frame, (pred_x, pred_y), 5, (0, 0, 255), -1)
        if latest_detected_box is not None:
            w, h = latest_detected_box
            cv2.rectangle(frame, (pred_x - w//2, pred_y - h//2), (pred_x + w//2, pred_y + h//2), (255, 0, 255), 2)
    return frame, latest_detected_box

def main():
    # 初始化Aruco标记字典
    aruco_dict=cv2.aruco.DICT_5X5_100
    # 初始化卡尔曼滤波器
    kalman=cv2.KalmanFilter(4,2)
    kalman.transitionMatrix = np.array([
        [1, 0, 1, 0], 
        [0, 1, 0, 1], 
        [0, 0, 1, 0], 
        [0, 0, 0, 1]], np.float32)
    kalman.measurementMatrix = np.array([
        [1, 0, 0, 0], 
        [0, 1, 0, 0]], np.float32)
    # 设置噪声协方差矩阵，给速度赋予更大的不确定性
    kalman.processNoiseCov = np.diag([1e-4, 1e-4, 5e-4, 5e-4]).astype(np.float32)
    # 设置测量噪声协方差矩阵
    kalman.measurementNoiseCov = np.eye(2, dtype=np.float32)*1e-2
    # 设置后验误差协方差矩阵，使滤波器更容易根据测量调整状态（特别是速度）
    kalman.errorCovPost = np.eye(4, dtype=np.float32) * 1.0
    # 初始化变量
    initialized = False
    missed_count = 0
    latest_detected_box = None
    is_save = False
    save_path = "./images/kalman.mp4"
    # 视频帧处理
    cap=cv2.VideoCapture(1)
    # 保存视频设置
    if is_save:
        fps = int(cap.get(cv2.CAP_PROP_FPS))
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        out = cv2.VideoWriter(save_path, fourcc, fps, (width, height))
    if not cap.isOpened():
        print("Error: Could not open camera.")
        exit()
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        # 追踪Aruco标记
        corners, ids, centers = track_aruco(frame, aruco_dict)
        # 初始化卡尔曼滤波器状态
        if not initialized and ids is not None:
            cx, cy = centers[0]
            kalman.statePost = np.array([[np.float32(cx)], [np.float32(cy)], [0.0], [0.0]], dtype=np.float32)
            initialized = True
        # 进行预测与校正
        prediction, missed_count, corrected_flag = predict_aruco(centers, ids, kalman, missed_count)
        frame, latest_detected_box = draw_aruco(frame, corners, centers, prediction, ids, latest_detected_box)
        if missed_count > 20:
            cv2.putText(frame, "Marker lost", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.imshow('camera1', frame)
        # 保存
        if is_save :
            out.write(frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

if __name__=="__main__":
    main()