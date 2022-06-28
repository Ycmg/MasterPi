#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/MasterPi/')
import cv2
import time
import math
import signal
import threading
import numpy as np
import yaml_handle
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
from HiwonderSDK.PID import PID
import HiwonderSDK.Misc as Misc
import HiwonderSDK.Board as Board


AK = ArmIK()

color_num = 0
line_width = 0
line_centerx = -1
line_centery = -1
line_color = 'black'
color_exist = False
detect_color = 'None'
detect_step = 'color'
__isRunning = False
block_clamp = False
LINE_CENTER_X = 320
target_color_list = ['red','green','blue']

line_pid = PID(P=0.20, I=0.15, D=0.10)

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255)}

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

lab_data = None
def load_config():
    global lab_data
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)

# 初始位置
def initMove():
    MotorStop()
    Board.setPWMServoPulse(1, 1500, 800)
    AK.setPitchRangeMoving((0, 7.5, 8.5), -70, -90, 0, 1500)
     
# 初始化调用
def init():
    print("IntelligentSorting Init")
    initMove()
    load_config()

# 开始玩法调用
def start():
    global __isRunning
    __isRunning = True
    print("IntelligentSorting Start")

def setBuzzer(timer):
    Board.setBuzzer(1)
    time.sleep(timer)
    Board.setBuzzer(0)

def MotorStop():
    Board.setMotor(1, 0) 
    Board.setMotor(2, 0)
    Board.setMotor(3, 0)
    Board.setMotor(4, 0)

    
# 找出面积最大的轮廓
# 参数为要比较的轮廓的列表
def getAreaMaxContour(contours):
    contour_area_max = 0
    contour_area_temp = 0
    area_max_contour = None
    for c in contours:  # 历遍所有轮廓
        contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp >= 5:  # 只有在面积大于300时，最大面积的轮廓才是有效的，以过滤干扰
                area_max_contour = c

    return area_max_contour, contour_area_max  # 返回最大的轮廓

def move():
    global color_num
    global detect_step
    global block_clamp
    global color_exist
    global detect_color
    global line_centerx
    global target_color_list
    
    i = 0
    clamp_num = 0
    move_time = time.time()
    transversae_num = 0
    transversae_time = time.time()
    
    position = {'red':1, 'green':2, 'blue':3, 'None':-1}
    
    while __isRunning:
        if detect_step == 'line':  
            if line_centerx > 0 and line_centery > 0:
                num = (line_centerx - LINE_CENTER_X)
                if abs(num) <= 10:  # 偏差比较小，不进行处理
                    line_pid.SetPoint = num
                else:
                    line_pid.SetPoint = 0
                line_pid.update(num) 
                tmp = line_pid.output    # 获取PID输出值
                tmp = 100 if tmp > 100 else tmp   
                tmp = -100 if tmp < -100 else tmp
                base_speed = Misc.map(tmp, -100, 100, -50, 50)  # 速度进行映射
                Board.setMotor(1, int(42-base_speed))           # 设置马达速度
                Board.setMotor(2, int(42+base_speed))
                Board.setMotor(3, int(42-base_speed))
                Board.setMotor(4, int(42+base_speed))
                
                if line_width > 100 and block_clamp:
                    if (time.time()-transversae_time) > 0.8:
                        transversae_num += 1
                        transversae_time = time.time()
                        
                if transversae_num == position[detect_color]:
                    Board.setMotor(1, 35)
                    Board.setMotor(2, 35)
                    Board.setMotor(3, 35)
                    Board.setMotor(4, 35)
                    if transversae_num == 1:
                        time.sleep(0.5)
                        move_time = time.time() + 14
                    elif transversae_num == 2:
                        time.sleep(0.4)
                        move_time = time.time() + 13
                    elif transversae_num == 3:
                        time.sleep(0.3)
                        move_time = time.time() + 12
                        
                    MotorStop()
                    AK.setPitchRangeMoving((12, 0, 5), -90, -95, -65, 1000)
                    time.sleep(1)
                    AK.setPitchRangeMoving((12, 0, -2), -90, -95, -65, 1000)
                    time.sleep(1)
                    Board.setPWMServoPulse(1, 2000, 500)
                    time.sleep(0.5)
                    AK.setPitchRangeMoving((0, 7.5, 8.5), -70, -90, 0, 1500)
                    time.sleep(1.5)
                    Board.setPWMServoPulse(1, 1500, 500)
                    time.sleep(0.5)
                    clamp_num += 1
                    target_color_list.remove(detect_color)
                    if clamp_num == 3:
                        clamp_num = 0
                        target_color_list = ['red','green','blue']
                    color_num = 0
                    transversae_num = 0
                    detect_color = 'None'
                    block_clamp = False
                    color_exist = False
                
                if time.time() >= move_time and not block_clamp:
                    MotorStop()
                    transversae_num = 0
                    detect_step = 'color'   
                    
            else:
                MotorStop()
                time.sleep(0.01)
                
        elif detect_step == 'color':
            if detect_color != 'None' and color_exist:
                detect_step = 'line'
                Board.setPWMServoPulse(1, 2000, 500)
                setBuzzer(0.1)
                time.sleep(2)
                Board.setPWMServoPulse(1, 1500, 500)
                time.sleep(1.5)
                block_clamp = True
                
            else:
                time.sleep(0.01)
                

detect_num = 0
def ColorDetect(img,color):
    global detect_num
    global detect_color
    global color_exist
    
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间
    if color in lab_data:
        frame_mask = cv2.inRange(frame_lab,(lab_data[color]['min'][0],lab_data[color]['min'][1],lab_data[color]['min'][2]),
                                (lab_data[color]['max'][0],lab_data[color]['max'][1],lab_data[color]['max'][2])) #对原图像和掩模进行位运算
        opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算
        contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓
        areaMaxContour, area_max = getAreaMaxContour(contours)  # 找出最大轮廓
        if area_max > 2000:  # 有找到最大面积
            rect = cv2.minAreaRect(areaMaxContour)
            box = np.int0(cv2.boxPoints(rect))
            cv2.drawContours(img, [box], -1, range_rgb[color], 2)
            pt1_x, pt1_y = box[0, 0], box[0, 1]
            pt3_x, pt3_y = box[2, 0], box[2, 1]
            color_centerx, color_centery = (pt1_x+pt3_x)/2, (pt1_y+pt3_y)/2  #中心点
            cv2.putText(img, "Color: " + detect_color, (10, img.shape[0] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, range_rgb[color], 2) # 把检测到的颜色打印在画面上
            color_exist = True
            detect_num += 1
            if detect_num >= 3:
                detect_num = 0
                detect_color = color
        else:
            detect_num = 0
            color_exist = False
    
    return img


size = (640, 480)
roi = [ # [ROI, weight]
        (200, 240,  0, 640, 0.2), 
        (280, 320,  0, 640, 0.3), 
        (360, 400,  0, 640, 0.5)]
roi_h1 = roi[0][0]
roi_h2 = roi[1][0] - roi[0][0]
roi_h3 = roi[2][0] - roi[1][0]
roi_h_list = [roi_h1, roi_h2, roi_h3]

def LineDetect(img):
    global line_width
    global line_centerx
    global line_centery
    
    n = 0
    weight_sum = 0
    centroid_x_sum = 0
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)         
    #将图像分割成上中下三个部分，这样处理速度会更快，更精确
    for r in roi:
        roi_h = roi_h_list[n]
        n += 1
        blobs = frame_gb[r[0]:r[1], r[2]:r[3]]
        frame_lab = cv2.cvtColor(blobs, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间
        if line_color in lab_data:
            frame_mask = cv2.inRange(frame_lab,(lab_data[line_color]['min'][0],lab_data[line_color]['min'][1],lab_data[line_color]['min'][2]),
                                    (lab_data[line_color]['max'][0],lab_data[line_color]['max'][1],lab_data[line_color]['max'][2])) #对原图像和掩模进行位运算
            eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  #腐蚀
            dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))) #膨胀
        cnts = cv2.findContours(dilated , cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]#找出所有轮廓
        cnt_large, area = getAreaMaxContour(cnts)#找到最大面积的轮廓
        if area >= 300: 
            rect = cv2.minAreaRect(cnt_large)#最小外接矩形
            box = np.int0(cv2.boxPoints(rect))#最小外接矩形的四个顶点
            for i in range(4):
                box[i, 1] = box[i, 1] + (n - 1)*roi_h + roi[0][0]
                box[i, 1] = int(Misc.map(box[i, 1], 0, size[1], 0, img_h))
                box[i, 0] = int(Misc.map(box[i, 0], 0, size[0], 0, img_w))

            cv2.drawContours(img, [box], -1, (0,0,255,255), 2)#画出四个点组成的矩形
            #获取矩形的对角点
            pt1_x, pt1_y = box[0, 0], box[0, 1]
            pt3_x, pt3_y = box[2, 0], box[2, 1]
            line_width = int(abs(pt1_x - pt3_x))
            color_centerx, color_centery = (pt1_x + pt3_x) / 2, (pt1_y + pt3_y) / 2#中心点       
            cv2.circle(img, (int(color_centerx), int(color_centery)), 5, (0,0,255), -1)#画出中心点         
            #按权重不同对上中下三个中心点进行求和
            centroid_x_sum += color_centerx * r[4]
            weight_sum += r[4]
    if weight_sum is not 0:
        #求最终得到的中心点
        line_centery = int(color_centery)
        line_centerx = int(centroid_x_sum / weight_sum) 
        cv2.circle(img, (line_centerx, int(color_centery)), 10, (0,255,255), -1)#画出中心点 
    else:
        line_centerx = -1
        
    return img


def run(img):
    global color_num
    global block_clamp
    
    if detect_step == 'line':
        img = LineDetect(img)
                
    elif detect_step == 'color':
        num = len(target_color_list)
        color = target_color_list[color_num]
        img = ColorDetect(img, color)
        if not color_exist:
            color_num += 1
            color_num = 0 if color_num >= num else color_num
        
    return img

# 关闭检测函数
def stop(signum, frame):
    global __isRunning
    
    __isRunning = False
    time.sleep(0.2)
    MotorStop()
    print('关闭中...')
    
signal.signal(signal.SIGINT, stop)

if __name__ == '__main__':
    
    init()
    start()
    # 运行子线程
    th = threading.Thread(target=move, daemon=True)
    th.start()
    cap = cv2.VideoCapture('http://127.0.0.1:8080?action=stream')
    
    while __isRunning:
        ret,img = cap.read()
        if ret:
            frame = img.copy()
            Frame = run(frame)  
            frame_resize = cv2.resize(Frame, (320, 240))
            cv2.imshow('frame', frame_resize)
            key = cv2.waitKey(1)
            if key == 27:
                break
        else:
            time.sleep(0.01)
    
    cv2.destroyAllWindows()
