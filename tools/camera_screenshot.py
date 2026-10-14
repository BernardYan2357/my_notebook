import cv2
import pathlib

def Camera_Screenshot(save_dir, camera_num=0, width=None, height=None):
    for name, value in (("width", width), ("height", height)):
        if value is not None and (not isinstance(value, int) or value <= 0):
            raise ValueError(f"{name} 必须是正整数或 None")
    
    if not save_dir.exists():
        save_dir.mkdir(parents=True, exist_ok=True)
    i=0

    cap=cv2.VideoCapture(camera_num)
    if not cap.isOpened():
        print("Error: Could not open video.")
        cap.release()
        return
    
    current_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    current_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"当前分辨率: {current_width} x {current_height}")

    if width is not None:
        if not cap.set(cv2.CAP_PROP_FRAME_WIDTH, width):
            print(f"摄像头未接受宽度设置: {width}")
    if height is not None:
        if not cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height):
            print(f"摄像头未接受高度设置: {height}")
    resolution_reported = False

    while True:
        ret,frame=cap.read()
        if not ret:
            break
        if not resolution_reported:
            actual_height, actual_width = frame.shape[:2]
            print(f"分辨率: {actual_width} x {actual_height}")
            if ((width is not None and actual_width != width)
                    or (height is not None and actual_height != height)):
                print("实际分辨率与请求值不同，摄像头可能不支持该分辨率。")
            resolution_reported = True
        cv2.imshow("video",frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):
            filename = save_dir / f"{i}.jpg"
            cv2.imwrite(str(filename), frame) 
            print(f"Saved {filename}")
            i += 1
        if key == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    camera_num=0
    width = None
    height = None
    save_dir = pathlib.Path("./")
    Camera_Screenshot(save_dir, camera_num, width=width, height=height)