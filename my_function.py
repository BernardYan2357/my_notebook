import cv2
import numpy as np
import matplotlib.pyplot as plt

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

def generate_gaussian_noise(img,std=25):
    """
    生成高斯噪声图像
    传入参数img为灰度图像,std为噪声标准差,默认值为25
    返还值为添加高斯噪声后的图像
    """
    gauss_noise=np.random.normal(0,std,size=img.shape)
    img_G_noise=np.clip(img+gauss_noise,0,255).astype(np.uint8)
    return img_G_noise

def generate_salt_and_pepper_noise(img,threshold=0.05):
    """
    生成椒盐噪声图像
    传入参数img为灰度图像,threshold为噪声比例,默认值为0.05
    返还值为添加椒盐噪声后的图像
    """
    x,y=img.shape
    salt_pepper_noise=np.random.rand(x,y)
    img[salt_pepper_noise<threshold]=0
    img[salt_pepper_noise>(1-threshold)]=255
    img_S_noise=img.copy()
    return img_S_noise

def show_gray_hist(gray):
    """
    显示灰度图像的直方图
    传入参数gray为灰度图像
    """
    hist=cv2.calcHist(gray,[0],None,[256],[0,256])
    plt.figure(figsize=[4,3])
    plt.plot(hist, color='black')
    plt.title('Grayscale Histogram')
    plt.xlabel('Pixel Value')
    plt.ylabel('Frequency')
    plt.xlim([0, 256])
    plt.grid(True)
    plt.show()