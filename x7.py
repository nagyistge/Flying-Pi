# Client Program
import socket
import threading
import os
import sys
import select
from time import sleep
import serial
import re
from math import pi
from gy80 import GY80
from GpsController import GpsController
from Adafruit_PWM_Servo_Driver import PWM
from Adafruit_BMP085 import BMP085
#---------------------------------------------------------------------------------
os.system("sudo killall gpsd")
os.system("sudo /etc/init.d/gpsd start")
os.system("gpsd /dev/ttyAMA0 -F /var/run/gpsd.sock")
gpsc = GpsController()
gpsc.start()
#---------------------------------------------------------------------------------

ser = serial.Serial('/dev/ttyACM0', 9600, timeout=0.025)
ser.close()
ser.open()
#---------------------------------------------------------------------------------

#---------------------------------------------------------------------------------
#import L3G4200Dx as l3g
#---------------------------------------------------------------------------------
sys.path.insert(0, 'Adafruit-Raspberry-Pi-Python-Code/Adafruit_PWM_Servo_Driver')
sys.path.insert(0, 'Adafruit-Raspberry-Pi-Python-Code/Adafruit_BMP085')
pwm = PWM(0x40, debug=True) 
servoMin = 150
servoMax = 600
def setServoPulse(channel, pulse):
  pulseLength = 1000000                   # 1,000,000 us per second
  pulseLength /= 60                       # 60 Hz
  print ("%d us per period" % pulseLength)
  pulseLength /= 4096                     # 12 bits of resolution
  print ("%d us per bit" % pulseLength)
  pulse *= 1000
  pulse /= pulseLength
  pwm.setPWM(channel, 0, pulse)
pwm.setPWMFreq(60)                        # Set frequency to 60 Hz
#---------------------------------------------------------------------------------
x = 1
while x == 1:
        class Receive(threading.Thread):
                def run(self):
                        global s # connection
                        global threadRunning
			yaw = 410 #GLOBALIZED
			throttle=300 #GLOBALIZED
                        while(threadRunning):
                                ready = select.select( [s], [], [], 1 )
                                if(ready[0]):
                                        data = s.recv(1024).strip()
                                        if(len(data) == 0): # Connection closed by server
                                                threadRunning = False
                                        else:
#                                                print('Received:', data.decode("utf-8")) # Converts thhe receive$
                                                foxtrot=data.decode("utf-8")
                                                extract=foxtrot.split()
                                                keyboard=extract[0]
                                                xmouse=extract[1]
                                                ymouse=extract[2]
                                                scroll=extract[3]
                                                click=extract[4]
                                                yvr=extract[5]
						xvr=extract[6]
#                                               print ('yvr = {0}'.format(yvr))
#						print ('xvr = {0}'.format(xvr))
#						print(keyboard)
#---------------------------------------------------------------------------------
                                                if (keyboard == 'SPACE'):
                                                       throttle = 329
                                                       print('THROTTLE RESET')
                                                if (keyboard == 'S'):
                                                        throttle = throttle-5
                                                        if (throttle <= 330):
                                                                print('WARNING!!! MINIMUM THROTTLE')
                                                                throttle=330
                                                if (keyboard == 'W'):
                                                        throttle = throttle+5
                                                        if (throttle >=500):
                                                                print('MAXIMUM')
                                                                throttle=500
						if (keyboard == '-'):
							throttle = throttle - 20
							if (throttle <= 330):
								throttle=330
						if (keyboard == '='):
							throttle = throttle +20
							if (throttle >= 500):
								throttle=500
 #--------------------------------------------------------------------------------
                                                roll = int(xmouse)- 483
						if (roll>400):
                                                  	roll = 400
						if (roll<0):
							roll = 0
						pitch = int(ymouse)
						if (pitch>400):
							pitch = 400
						if (keyboard == 'Q'): #or 'LCONTROL'):
							roll = 200
							pitch = 200
						if (keyboard == 'LCONTROL'): #or 'LCONTROL'):
                                                        roll = 200
                                                        pitch = 200
#---------------------------------------------------------------------------------
                                                bmp = BMP085(0x77)
                                                temp = bmp.readTemperature()
                                                pressure = bmp.readPressure()
                                                altitude = bmp.readAltitude()
#---------------------------------------------------------------------------------
                                                print(throttle)
						pwm.setPWM(0,0,throttle)
						pwm.setPWM(1,0,560-pitch)
						pwm.setPWM(2,0,160+pitch)
                                                yaw = 410
                                                if (keyboard == 'A' and pitch == 200):
                                                        yaw = yaw-170
                                                        if (yaw <= 240):
                                                                yaw=240
                                                        pwm.setPWM(1,0,yaw)
                                                        pwm.setPWM(2,0,yaw)
                                                if (keyboard == 'D' and pitch == 200):
                                                        yaw = 540
                                                        pwm.setPWM(1,0,yaw)
                                                        pwm.setPWM(2,0,yaw)
						pwm.setPWM(3,0,roll+170)
						pwm.setPWM(4,0,roll+170)
						if (keyboard == 'UP'):
							pwm.setPWM(3,0,370)
							pwm.setPWM(4,0,370)
						if (keyboard == 'DOWN'):
							pwm.setPWM(3,0,570)
							pwm.setPWM(4,0,370)
						pwm.setPWM(5,0,320) #ycam init
						pwm.setPWM(6,0,350) #xcam init
						yaxis=int(ymouse)
						xaxis=int(xmouse)
                                  		xcam = int(xvr)*(2+1/2)+150
						ycam = -int(yvr)*(4+1/2)+350
						pwm.setPWM(6,0,xcam)
						pwm.setPWM(5,0,ycam)
						print xcam,ycam                                 
#---------------------------------------------------------------------------------
						global imu
						imu = GY80()
						rawyaw, rawroll, rawpitch = imu.current_orientation_euler_angles_hybrid()
						gyaw = rawyaw*180.0/pi
						gpitch = 178-rawpitch*180.0/pi
						if gpitch > 180:
							gpitch=gpitch-360
						groll = rawroll*180.0/pi
						print(gpitch, groll)
#---------------------------------------------------------------------------------
						latitude = gpsc.fix.latitude
						longitude = gpsc.fix.longitude
						altitude2 = gpsc.fix.altitude
						speed = gpsc.fix.speed
						climb = gpsc.fix.climb
#---------------------------------------------------------------------------------
						volt1=1
						serial_data = ser.readline()
						v1=re.search(r'V1-(.*)',serial_data)
						if v1:
					                volt1=v1.group(1)
						        print(volt1)
#---------------------------------------------------------------------------------
                                                stringThrottle=bytearray(str(throttle), "utf-8")
                                                stringTemp=bytearray(str(temp), "utf-8")
                                                stringAlt=bytearray(str(altitude), "utf-8")
                                                stringPitch=bytearray(str(gpitch), "utf-8")
                                                stringRoll=bytearray(str(groll), "utf-8")
                                                stringYaw=bytearray(str(gyaw), "utf-8")
                                                stringLat=bytearray(str(latitude), "utf-8")
                                                stringLong=bytearray(str(longitude), "utf-8")
                                                stringVolt1=bytearray(str(volt1), "utf-8")
                                                stringSpeed=bytearray(str(speed), "utf-8")
                                                stringClim=bytearray(str(climb), "utf-8")
						data = 'THROTL'+stringThrottle.zfill(2)+'TEMP'+stringTemp+'ALT'+stringAlt.zfill(4)+'PIT'+stringPitch+'ROLL'+stringRoll+'YAW'+stringYaw+'LATI'+stringLat.zfill(9)+'LONG'+stringLong.zfill(10)+'SPEE'+stringSpeed+'CLIM'+stringClim + 'VOLT1'+stringVolt1 #+ 'VOLT2' + stringVolt2 + 'VOLT3' + stringVolt3 + 'VOLT4' + stringVolt4                                                
                                                s.sendall(data.zfill(2))
#---------------------------------------------------------------------------------
        class Transmit(threading.Thread):
                def run(self):
                  global byteArray2
                  global throttle
                  global s # connection
                  global threadRunning
#                  string = 0     
                  while(threadRunning):
                                # Read text with 1 sec timeout                              
                                i, o, e = select.select( [sys.stdin], [], [], 1 )
                                return
                                sleep(0.05)
                                return
         
        # Running main program
        HOST = '192.168.1.219'#'25.6.207.151' #76' # The remote host - windows machine running the LabVIEW Server
        PORT = 2055 # The same port as used by the server - defined in LabVIEW
        global threadRunning # Used to stop threads
        threadRunning = False
        global s
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((HOST, PORT))
        os.system('clear')
        print('Connection with UAV established')
         
        try:
                print(34 * '-')
                print("  W E L C O M E    P H I L I P!")
                print(' Press CTRL+C to close connection')
                print(34 * '-')
                # Create instance of class
                threadRunning = True
                transmit = Transmit()
                receive = Receive()
                # Start class
                transmit.start()
                receive.start()
                while(threadRunning):
                        sleep(0.05)
        except IOError:
                pass
