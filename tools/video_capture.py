import cv2
import pathlib

def Video_Capture(save_dir, camera_num=0):
    if not save_dir.exists():
        save_dir.mkdir(parents=True, exist_ok=True)

    cap=cv2.VideoCapture(camera_num)

    fps = int(cap.get(cv2.CAP_PROP_FPS))
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fourcc = cv2.VideoWriter.fourcc(*'mp4v')
    out = cv2.VideoWriter(str(save_dir / 'output.mp4'), fourcc, fps, (width, height))
    
    if not cap.isOpened():
        print("Error: Could not open video.")
        exit()

    flag: bool = False

    while True:
        ret,frame=cap.read()
        if not ret:
            break
        cv2.imshow("video",frame)

        if flag:
            out.write(frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):
            flag = not flag
            if flag:
                print("Recording started.")
            else:
                print("Recording stopped.")
        if key == ord('q'):
            break

    cap.release()
    out.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    camera_num=0
    save_dir = pathlib.Path("./")
    Video_Capture(save_dir, camera_num)