import cv2
import pathlib

def Camera_Screenshot(save_dir, camera_num=0):
    if not save_dir.exists():
        save_dir.mkdir(parents=True, exist_ok=True)
    i=0
    cap=cv2.VideoCapture(camera_num)
    if not cap.isOpened():
        print("Error: Could not open video.")
        exit()
    while True:
        ret,frame=cap.read()
        if not ret:
            break
        cv2.imshow("video",frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):
            filename = save_dir / f"{i}.jpg"
            cv2.imwrite(str(filename), frame) 
            print(f"Saved {filename}")
            i += 1
        if cv2.waitKey(1) & key == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    camera_num=1
    save_dir = pathlib.Path("./camera_calib/")
    Camera_Screenshot(save_dir, camera_num)