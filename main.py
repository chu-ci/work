import math
import cv2
import mediapipe as mp
import numpy as np
import pygame

class PoseDumbbell:
    def __init__(self):
        self.mpose = mp.solutions.pose
        #姿态对象
        self.pose = self.mpose.Pose()
        #画笔
        self.draw = mp.solutions.drawing_utils
    def findPose(self,img):
        #灰化
        imggray = cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
        self.rs = self.pose.process(imggray)
        if self.rs.pose_landmarks:
            self.draw.draw_landmarks(img,self.rs.pose_landmarks,self.mpose.POSE_CONNECTIONS)
        return img
    #定位
    def findPosition(self,img):
        self.lmlist = []
        if self.rs.pose_landmarks:
            for id,lm in enumerate(self.rs.pose_landmarks.landmark):
                h,w,c = img.shape
                #获取关节的坐标
                cx,cy = int(lm.x*w),int(lm.y*h)
                self.lmlist.append([id,cx,cy])
                cv2.circle(img,(cx,cy),5,(200,0,0),cv2.FILLED)
        return self.lmlist
    def computeAngle(self,img,p1,p2,p3):
        #肩
        x1,y1 = self.lmlist[p1][1:]
        #肘
        x2,y2 = self.lmlist[p2][1:]
        #手腕
        x3,y3 = self.lmlist[p3][1:]
        #手臂弯度
        angle = math.degrees(math.atan2(y3 - y2,x3 - x2) - math.atan2(y1 - y2,x1 - x2))
        if angle < 0:
            angle += 360
            cv2.putText(img,str(int(angle)),(x2 - 50,y2 + 50),cv2.FONT_HERSHEY_PLAIN,2,(0,0,222),2)
        return angle
detector = PoseDumbbell()
dir = 0
count = 0
cap = cv2.VideoCapture(0,cv2.CAP_DSHOW)
#cap = cv2.VideoCapture("放入视频路径")
while True:
    ok,frame = cap.read()
    pygame.init()
    pygame.mixer.init()
    bgsound = pygame.mixer.Sound("D:\桌面\python_work\smart/11323.wav")
    if ok:
        #设置分辨率
        img = cv2.resize(frame,(640,480))
        img = detector.findPose(img)
        lmlist = detector.findPosition(img)
        if len(lmlist) > 0:
            #此处计算的是右胳膊
            angle = detector.computeAngle(img,12,14,16)
            #线性插值
            per = np.interp(angle,(210,310),(0,100))
            if per == 100:
                if dir == 0:
                    count += 0.5
                    dir = 1
            if per == 0:
                if dir == 1:
                    count += 0.5
                    dir = 0
                    bgsound.play()
            cv2.putText(img,str(int(count)),(45,450),cv2.FONT_HERSHEY_PLAIN,5,(0,0,222),8)
    cv2.imshow("",img)
    cv2.waitKey(1)
cap.release()