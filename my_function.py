import cv2
import numpy as np

def resize_for_display(image, max_size=500):
    """将图像最长边限制在 max_size 像素内，保持比例，不放大小图"""
    if type(max_size) is not int or max_size <= 0:
        raise ValueError("max_size 必须是正整数")
    h, w = image.shape[:2]
    if max(h, w) <= max_size:
        return image
    scale = max_size / max(h, w)
    size = (max(1, int(w * scale)), max(1, int(h * scale)))
    return cv2.resize(image, size, interpolation=cv2.INTER_AREA)


def imshow(max_size=500, **images):
    """
    使用字典传入窗口名和图像
    自动将最长边大于 max_size 的图像按比例缩放至最长边为 max_size
    使用示例: imshow(name1=image1, name2=image2)
    """
    for name, image in images.items():
        cv2.imshow(name, resize_for_display(image, max_size))
    cv2.waitKey(0)
    cv2.destroyAllWindows()
