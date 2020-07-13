# Quick conversion from Arduino code. Needs refactoring.

# Servo driver board

import time
from adafruit_servokit import ServoKit
kit = ServoKit(channels=16)

# IMU

import adafruit_bno055
# bno = BNO055.BNO055( )

# Barometer

import board
import busio
#change I2C lines
i2c = busio.I2C(board.SCL, board.SDA)
bno = adafruit_bno055.BNO055(i2c)

import ms5837
import time

control = 0

thruster_frontright = kit.continuous_servo[0]
thruster_frontleft = kit.continuous_servo[1]
thruster_backright = kit.continuous_servo[2]
thruster_backleft = kit.continuous_servo[3]
thruster_sideright = kit.continuous_servo[4]
thruster_sideleft = kit.continuous_servo[5]

elapsedTime, time, timePrev
PIDY, PIDX, PIDZ, pwmFrontRight, pwmFrontLeft, pwmBackRight, pwmBackLeft, pwmSideLeft, pwmSideRight, errorY, errorX, errorZ, previous_errorY, previous_errorX, previous_errorZ, current_depth, PID_depth, depth_error, previous_depth_error
pid_pY=0
pid_iY=0
pid_dY=0
pid_pX=0
pid_iX=0
pid_dX=0
pid_pZ=0
pid_iZ=0
pid_dZ=0
pid_p_depth = 0
pid_d_depth = 0
pid_i_depth = 0
y_angle = 0
x_angle = 0
z_angle = 0

##################PID GAINS#################
kp=3.5## 3.5 WORKED
ki=0.0000005## 0 WOKRED
kd=4## 4 WORKED
################################################

### throttle range 0 to 1
### 0.5 midpoint analogous to 1500 microseconds on Arduino
throttle=0.5 ##Motor throttle --> Linearized control about this nominal thruster setting (0 thrust since we don't want thrusters constantly running)
desired_x_angle = 0 ##Setpoint angle
desired_y_angle = 0 
desired_z_angle = 0
desired_depth = 0 ##Setpoint depth



time = millis() ##Time is in miliseconds
thruster_frontright.throttle(0.5)
thruster_frontleft.throttle(0.5)
thruster_backright.throttle(0.5)
thruster_backleft.throttle(0.5)
  ##delay(5000) #* Start up delay *#

#  #* Initialise the IMU *#

if not bno.begin():
  {
    #* There was a problem detecting the BNO055 ... check your connections *#
    print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!")
    # while 1:
  }
bno.setExtCrystalUse(true)

#  #* Set up Barometer *#

sensor = ms5837.MS5837_30BA() 
sensor.setFluidDensity(997) ## kg#m^3 (freshwater, 1029 for seawater)
# We must initialize the sensor before reading it
if not sensor.init():
        print("Sensor could not be initialized")
        exit(1)

# We have to read values from sensor to update pressure and temperature
if not sensor.read():
    print("Sensor read failed!")
    exit(1)


euler = sensor.euler

y_angle = euler.y()
z_angle = euler.x()
x_angle = euler.z() ##Note: IMU interprets vertical axis as x-axis, but we'll refer to it as z-axis

desired_z_angle = z_angle




timePrev = time  ## the previous time is stored before the actual time read
time = millis()  ## actual time read
elapsedTime = (time - timePrev) # 1000

#############################I M U#####################################

euler = sensor.euler

y_angle = euler.y()
z_angle = euler.x()
x_angle = euler.z() ##Note: IMU interprets vertical axis as x-axis, but we'll refer to it as z-axis 

# #*Calculate error between measured and desired angle*#

errorY =  y_angle - desired_y_angle
errorX = x_angle - desired_x_angle
errorZ = z_angle - desired_z_angle

if (errorZ<-180):
  errorZ+=360
elif (errorZ>180):
  errorZ-=360


# #* Calculate proportional PID term *#

pid_pY = kp*errorY
pid_pX = kp*errorX
pid_pZ = kp*errorZ

# #* Calculate the integral PID term, only when error is +#- 10 degrees *#

if(-10 <errorY <10):
  pid_iY = pid_iY+(ki*errorY)

if(-10 <errorX <10):
  pid_iX = pid_iX+(ki*errorX)

# #*Calculate derivative PID term *#

pid_dY = kd*((errorY - previous_errorY)) #elapsedTime
pid_dX = kd*((errorX - previous_errorX))
pid_dZ = kd*((errorZ - previous_errorZ))

# #*Sum all PID terms *#

PIDY = pid_pY + pid_dY
PIDX = pid_pX + pid_dX
PIDZ = pid_pZ + pid_dZ

# #*Place bounds on PID terms *#

###if(PIDY < -0.03)
###{
###  PIDY=-0.03
###}
###if(PIDY > 0.03)
###{
###  PIDY=0.03
###}
###
###if(PIDX < -0.03)
###{
###  PIDX=-0.03
###}
###if(PIDX > 0.03)
###{
###  PIDX=0.03
###}

##*Calculate PWM signal --> Throttle + PID terms = Motor Mixing Algorithm *#

##*
# Layout for how the PID terms interact with front#back#left#right thrusters
pwmFront = throttle + PIDX
pwmBack = throttle - PIDX
pwmRight = throttle - PIDY
pwmLeft = throttle + PIDY
#*#

#* Barometer  *#

sensor.read() ## get data from barometer
current_depth = sensor.depth() ## create current depth variable
depth_error = desired_depth - current_depth ## calculate the error between current depth and desired depth
print("Depth: ")
print(sensor.depth())
print(" m")

#* Barometer PID terms *#

pid_p_depth = kp*depth_error

if(-.01 <depth_error <.01):
	pid_i_depth = pid_i_depth+(ki*depth_error)


pid_d_depth = kd*((depth_error - previous_depth_error)) #elapsedTime
PID_depth = pid_p_depth + pid_d_depth
PID_depth = PID_depth*30

#* Motor Mixing Algorithm *#

##Front Thrusters
##pwmFrontRight = throttle + PIDX - PIDY + PID_depth
##pwmFrontLeft = throttle + PIDX + PIDY + PID_depth

pwmFrontRight = throttle - PIDX - PIDY + PID_depth
pwmFrontLeft = throttle + PIDX - PIDY + PID_depth

##Back Thrusters
##pwmBackRight = throttle - PIDX - PIDY + PID_depth
##pwmBackLeft = throttle - PIDX + PIDY + PID_depth

pwmBackRight = throttle - PIDX + PIDY + PID_depth
pwmBackLeft = throttle + PIDX + PIDY + PID_depth

##Side Thrusters

pwmSideRight = throttle + PIDZ
pwmSideLeft = throttle - PIDZ

#*Place bounds on PWM signal *#

##Front
if(pwmFrontRight < 0.51):
  pwmFrontRight= 0.5

if(pwmFrontRight > 0.53):
  pwmFrontRight=0.53


if(pwmFrontLeft < 0.51):
  pwmFrontLeft= 0.5

if(pwmFrontLeft > 0.53):
  pwmFrontLeft=0.53

##Back
if(pwmBackRight < 0.51):
  pwmBackRight= 0.5


if(pwmBackRight > 0.53):
  pwmBackRight=0.53


if(pwmBackLeft < 0.51):
  pwmBackLeft= 0.5


if(pwmBackLeft > 0.53):
  pwmBackLeft=0.53


if(pwmSideRight > 0.53):

  pwmSideRight=0.53


if(pwmSideRight < 0.51):
  pwmSideRight=0.5


if(pwmSideLeft > 0.53):
  pwmSideLeft=0.53


if(pwmSideLeft < 0.51):
  pwmSideLeft=0.5


#######################################################

## CASE STATEMENTS FOR MANUAL CONTROL


"""

if (input() > 0):
                ## read the incoming byte:
                control = input()

                ## say what you got:
                if ((control >= 49 and control <=57) or (control >= 97 and control <= 119) ):
                	print("I received: ")
                	print(control, DEC)
                else:
                  print("Invalid key pressed.")


if ((control <= 57 and control >= 49) or (control <= 119 and control >= 97)):
     switch(control):
        case 97: ##a : turn left
          thruster_frontright.throttle(pwmFrontRight)
          thruster_frontleft.throttle(pwmFrontLeft)
          thruster_backright.throttle(pwmBackRight)
          thruster_backleft.throttle(pwmBackLeft)
          thruster_sideright.throttle(pwmSideRight-0.1)
          thruster_sideleft.throttle(pwmSideLeft+0.1)
          break
        case 115: ##s : go backwards
          thruster_frontright.throttle(pwmFrontRight)
          thruster_frontleft.throttle(pwmFrontLeft)
          thruster_backright.throttle(pwmBackRight)
          thruster_backleft.throttle(pwmBackLeft)
          thruster_sideright.throttle(pwmSideRight+0.1)
          thruster_sideleft.throttle(pwmSideLeft+0.1)
          break
        case 100: ##d : turn right
          thruster_frontright.throttle(pwmFrontRight)
          thruster_frontleft.throttle(pwmFrontLeft)
          thruster_backright.throttle(pwmBackRight)
          thruster_backleft.throttle(pwmBackLeft)
          thruster_sideright.throttle(pwmSideRight+0.1)
          thruster_sideleft.throttle(pwmSideLeft-0.1)
          break
        case 119: ##w : go forward
          thruster_frontright.throttle(pwmFrontRight)
          thruster_frontleft.throttle(pwmFrontLeft)
          thruster_backright.throttle(pwmBackRight)
          thruster_backleft.throttle(pwmBackLeft)
          thruster_sideright.throttle(pwmSideRight-0.1)
          thruster_sideleft.throttle(pwmSideLeft-0.1)
          break
        case 101: ##e : go up
          thruster_frontright.throttle(pwmFrontRight-.03)
          thruster_frontleft.throttle(pwmFrontLeft-.03)
          thruster_backright.throttle(pwmBackRight-.03)
          thruster_backleft.throttle(pwmBackLeft-.03)
          thruster_sideright.throttle(pwmSideRight)
          thruster_sideleft.throttle(pwmSideLeft)
          break
        case 99: ##c : submerge
          thruster_frontright.throttle(pwmFrontRight+.03)
          thruster_frontleft.throttle(pwmFrontLeft+.03)
          thruster_backright.throttle(pwmBackRight+.03)
          thruster_backleft.throttle(pwmBackLeft+.03)
          thruster_sideright.throttle(pwmSideRight)
          thruster_sideleft.throttle(pwmSideLeft)
          break
        case 103: ##g : the two right thrusters are on (roll)
          thruster_frontright.throttle(pwmFrontRight+0.1)
          thruster_frontleft.throttle(pwmFrontLeft-0.1)
          thruster_backright.throttle(pwmBackRight+0.1)
          thruster_backleft.throttle(pwmBackLeft-0.1)
          thruster_sideright.throttle(pwmSideRight)
          thruster_sideleft.throttle(pwmSideLeft)
          break
        case 104: ##h : the two left thrusters are on (roll)
          thruster_frontright.throttle(pwmFrontRight-0.1)
          thruster_frontleft.throttle(pwmFrontLeft+0.1)
          thruster_backright.throttle(pwmBackRight-0.1)
          thruster_backleft.throttle(pwmBackLeft+0.1)
          thruster_sideright.throttle(pwmSideRight)
          thruster_sideleft.throttle(pwmSideLeft)
          break
        case 105: ##i : the two front thrusters are on (pitch)
          thruster_frontright.throttle(pwmFrontRight+0.1)
          thruster_frontleft.throttle(pwmFrontLeft+0.1)
          thruster_backright.throttle(pwmBackRight-0.1)
          thruster_backleft.throttle(pwmBackLeft-0.1)
          thruster_sideright.throttle(pwmSideRight)
          thruster_sideleft.throttle(pwmSideLeft)
          break
        case 106: ##j : the two back thrusters are on (pitch)
          thruster_frontright.throttle(pwmFrontRight-0.1)
          thruster_frontleft.throttle(pwmFrontLeft-0.1)
          thruster_backright.throttle(pwmBackRight+0.1)
          thruster_backleft.throttle(pwmBackLeft+0.1)
          thruster_sideright.throttle(pwmSideRight)
          thruster_sideleft.throttle(pwmSideLeft)
          break
        case 111: ##o : shut off all motors
          thruster_frontright.throttle(pwmFrontRight)
          thruster_frontleft.throttle(pwmFrontLeft)
          thruster_backright.throttle(pwmBackRight)
          thruster_backleft.throttle(pwmBackLeft)
          thruster_sideright.throttle(pwmSideRight)
          thruster_sideleft.throttle(pwmSideLeft)
          break
        default:
          thruster_frontright.throttle(pwmFrontRight)
          thruster_frontleft.throttle(pwmFrontLeft)
          thruster_backright.throttle(pwmBackRight)
          thruster_backleft.throttle(pwmBackLeft)
          thruster_sideright.throttle(pwmSideRight)
          thruster_sideleft.throttle(pwmSideLeft)
          break

else:
          thruster_frontright.throttle(pwmFrontRight)
          thruster_frontleft.throttle(pwmFrontLeft)
          thruster_backright.throttle(pwmBackRight)
          thruster_backleft.throttle(pwmBackLeft)
          ##thruster_sideright.throttle(pwmSideRight)
          ##thruster_sideleft.throttle(pwmSideLeft)


######################################################

          thruster_backleft.throttle(pwmBackLeft)
          thruster_sideright.throttle(pwmSideRight)
          thruster_sideleft.throttle(pwmSideLeft)
          break
        default:
          thruster_frontright.throttle(pwmFrontRight)
          thruster_frontleft.throttle(pwmFrontLeft)
          thruster_backright.throttle(pwmBackRight)
          thruster_backleft.throttle(pwmBackLeft)
          thruster_sideright.throttle(pwmSideRight)
          thruster_sideleft.throttle(pwmSideLeft)
          break


  else:
          thruster_frontright.throttle(pwmFrontRight)
          thruster_frontleft.throttle(pwmFrontLeft)
          thruster_backright.throttle(pwmBackRight)
          thruster_backleft.throttle(pwmBackLeft)
          ##thruster_sideright.throttle(pwmSideRight)
          ##thruster_sideleft.throttle(pwmSideLeft)


######################################################
"""

##Store errors for next loop

previous_errorY = errorY
previous_errorX = errorX
previous_errorZ = errorZ
previous_depth_error = depth_error


