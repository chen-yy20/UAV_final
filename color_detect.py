import numpy as np
import cv2

# 读入图片
def find_door(img):
    # img = cv2.imread('./red3.jpg')

    # 将图片转换为HSV颜色空间
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # 设定红色的HSV阈值范围
    lower_red = np.array([0, 50, 50])
    upper_red = np.array([10, 255, 255])
    mask1 = cv2.inRange(hsv, lower_red, upper_red)

    lower_red = np.array([170, 50, 50])
    upper_red = np.array([180, 255, 255])
    mask2 = cv2.inRange(hsv, lower_red, upper_red)

    # 将两个mask相加
    mask = mask1 + mask2

    # 对mask进行形态学操作，去除噪点
    kernel = np.ones((5,5),np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # 找到圆形轮廓
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 遍历所有轮廓，找到最大的圆形轮廓
    max_contour = None
    max_area = 0
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > max_area:
            max_area = area
            max_contour = contour

    # 找到最大圆形轮廓的最小外接圆
    (x,y),radius = cv2.minEnclosingCircle(max_contour)
    center = (int(x),int(y))
    radius = int(radius)

    print("此圆半径：{}".format(radius))
    # 在原图上画出圆心位置
    cv2.circle(img,center,radius,(0,255,0),2)

    # 显示图片
    cv2.imshow('image',img)
    cv2.waitKey()
    cv2.destroyAllWindows()

    # 将图片中心设为坐标系原点
    img_center = (img.shape[1]//2, img.shape[0]//2)
    if  radius <= 30:
        # 如果没有找到圆心，输出None
        return ("None")
    else:
        # 计算圆心相对于图片中心的位置
        center_relative = (center[0] - img_center[0], img_center[1] - center[1])
        # 判断圆心在哪个象限
        if center_relative[0] >= 0 and center_relative[1] >= 0:
            return (1)
        elif center_relative[0] < 0 and center_relative[1] >= 0:
            return (0)
        elif center_relative[0] < 0 and center_relative[1] < 0:
            return (2)
        else:
            return (3)
    

    # # 输出圆心坐标
    # print(center)
for i in range(4):
    name = './pictures/left'+str(i)+'.png'  
    img = cv2.imread(name)
    print(find_door(img))
name = './pictures/leftn.jpg'  
img = cv2.imread(name)
print(find_door(img))