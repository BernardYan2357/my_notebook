import cv2
import numpy as np

def imshow(**images):
    """
    使用字典传入窗口名和图像
    自动将宽高大于500px的图像按比例缩放至宽度为500px
    使用示例: imshow(name1=image1, name2=image2)
    """
    for name, image in images.items():
        h, w = image.shape[:2]
        if max(h, w) > 500:
            scale = 500.0 / max(h, w)
            new_width = int(w * scale)
            new_height = int(h * scale)
            resized_image = cv2.resize(image, (new_width, new_height))
            cv2.imshow(name, resized_image)
        else:
            cv2.imshow(name, image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()