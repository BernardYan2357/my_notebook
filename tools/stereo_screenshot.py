import pathlib
import sys
import cv2


if __package__ in (None, ""):
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from my_function import resize_for_display


def Stereo_Screenshot(save_dir, camera_num=0, rotate=False, concat_display=True,
                      width=None, height=None, display_max_size=1000):
    """
    按 s 保存一对图像，按 q 退出
    rotate: 是否旋转180度
    concat_display: 是否左右拼接显示
    width、height: 完整双目流的宽高, None 表示保留默认值
    display_max_size: 每个预览窗口图像的最长边上限，不影响保存分辨率
    """
    if type(display_max_size) is not int or display_max_size <= 0:
        raise ValueError("display_max_size 必须是正整数")
    for name, value in (("width", width), ("height", height)):
        if value is not None and (type(value) is not int or value <= 0):
            raise ValueError(f"{name} 必须是正整数或 None")
    if width is not None and width % 2 != 0:
        raise ValueError("双目流宽度必须为偶数，以便均分左右图像")
    save_dir = pathlib.Path(save_dir)
    left_dir = save_dir / "left"
    right_dir = save_dir / "right"
    left_dir.mkdir(parents=True, exist_ok=True)
    right_dir.mkdir(parents=True, exist_ok=True)

    # 从已有编号之后继续保存，避免覆盖之前的截图
    existing_indices = [
        int(path.stem)
        for directory in (left_dir, right_dir)
        for path in directory.glob("*.jpg")
        if path.stem.isdecimal()
    ]
    i = max(existing_indices, default=-1) + 1
    cap = cv2.VideoCapture(camera_num)
    try:
        if not cap.isOpened():
            print("Error: Could not open stereo camera.")
            return

        if width is not None:
            if not cap.set(cv2.CAP_PROP_FRAME_WIDTH, width):
                print(f"摄像头未接受宽度设置: {width}")
        if height is not None:
            if not cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height):
                print(f"摄像头未接受高度设置: {height}")

        print("按 s 保存左右图像，按 q 退出。")
        resolution_reported = False
        while True:
            if rotate:
                ret, frame = cap.read()
                if ret:
                    frame = cv2.rotate(frame, cv2.ROTATE_180)
            else:
                ret, frame = cap.read()
            if not ret:
                print("读取双目摄像头图像失败。")
                break

            actual_height, actual_width = frame.shape[:2]
            if actual_width < 2 or actual_width % 2 != 0:
                print(f"图像宽度 {actual_width} 无法均分为左右图像。")
                break
            midpoint = actual_width // 2
            left_frame = frame[:, :midpoint]
            right_frame = frame[:, midpoint:]
            if not resolution_reported:
                print(f"摄像头流实际分辨率: {actual_width} x {actual_height}")
                print(f"左右图像分辨率: {midpoint} x {actual_height}")
                if ((width is not None and actual_width != width)
                        or (height is not None and actual_height != height)):
                    print("实际分辨率与请求值不同，摄像头可能不支持该设置。")
                resolution_reported = True

            if concat_display:
                cv2.imshow("stereo", resize_for_display(frame, display_max_size))
            else:
                cv2.imshow("left", resize_for_display(left_frame, display_max_size))
                cv2.imshow("right", resize_for_display(right_frame, display_max_size))
            key = cv2.waitKey(1) & 0xFF
            if key == ord("s"):
                left_path = left_dir / f"{i}.jpg"
                right_path = right_dir / f"{i}.jpg"
                left_saved = cv2.imwrite(str(left_path), left_frame)
                right_saved = cv2.imwrite(str(right_path), right_frame)
                if left_saved and right_saved:
                    print(f"Saved {left_path} and {right_path}")
                    i += 1
                else:
                    # 清理本次未完整保存的图像对
                    left_path.unlink(missing_ok=True)
                    right_path.unlink(missing_ok=True)
                    print("左右图像保存失败，请检查保存目录。")
            elif key == ord("q"):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    camera_num = 1
    rotate = True
    concat_display = True
    display_max_size = 3840//2 # 预览图像最长边上限，可按显示器大小调整
    # 完整双目流分辨率；例如 3840 x 1080 对应每侧 1920 x 1080
    width = 3840
    height = 1080
    save_dir = pathlib.Path("./tools/image")
    Stereo_Screenshot(save_dir, camera_num, rotate, concat_display=concat_display,
                      width=width, height=height, display_max_size=display_max_size)
