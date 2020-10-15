import math
import time
from machine import Timer,PWM
from fpioa_manager import *
from Maix import GPIO

def Servo(servo,angle):
    if 180 >= angle >= 0:
        servo.duty((angle/90 + 0.5)*(100/20))

tim0 = Timer(Timer.TIMER0, Timer.CHANNEL0, mode=Timer.MODE_PWM)
tim1 = Timer(Timer.TIMER1, Timer.CHANNEL1, mode=Timer.MODE_PWM)
class gimbal():
    
    def __init__(self,pin1,pin2,tim0,tim1):

        self.tim0 = tim0
        self.tim1 = tim1
        self.pin1 = pin1
        self.pin2 = pin2
    
    def correction(self): #雷射點對與平面邊緣中心點校正
        servoPWM1 = PWM(self.tim0, freq=50, duty=0, pin=self.pin1)
        servoPWM2 = PWM(self.tim1, freq=50, duty=0, pin=self.pin2)
        Servo(servoPWM1,0) # 垂直指向水平
        Servo(servoPWM2,100)
        time.sleep(1)
        servoPWM1.deinit()
        servoPWM2.deinit()
    
    def angle(self,theta,phi):
        servoPWM1 = PWM(self.tim0, freq=50, duty=0, pin=self.pin1)
        servoPWM2 = PWM(self.tim1, freq=50, duty=0, pin=self.pin2)
        Servo(servoPWM1,theta)
        Servo(servoPWM2,phi)
        time.sleep(1)
        servoPWM1.deinit()
        servoPWM2.deinit()
        
    def coor2angle(self,x,y):
        z = -15-x/5
        y = y*( (8/5) - (1/(y+10e-6)))
        
        orig = (0,0,0) # 假設原點為 servo 交接處
        
        # 4.2 為垂直軸心與猜想圓心距離、13 為猜想圓心與地面距離
        d = 4.2*math.tan(math.pi/2 - math.atan((z - 4.2)/13)) # radius 
        sec_orig = (0,0,d)   # 更新原點
        coor     = (x,y,z)   # 目標點
        sec_coor = (x,y,z-d) # 目標點新座標

        # Cartesian coordinate system  =>  spherical coordinate system
        r     = math.sqrt(sec_coor[0]**2 + sec_coor[1]**2 + sec_coor[2]**2)
        theta = math.acos(-(sec_coor[0])/r)         # radius
        phi   = math.atan(sec_coor[1]/sec_coor[2])  # radius
        
        theta *= 180/math.pi # spherical degree
        phi   *= 180/math.pi # spherical degree
        
        theta  = theta-90 # hardware degree
        phi    = 100+phi  # hardware degree
        
        angle(theta,phi)
  
    
