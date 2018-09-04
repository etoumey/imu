from mpu6050 import mpu6050
import math 
import time
import numpy as np

def calibrate(sensor):
	print "Calibrating..."
	n = 500 #calibration sample size
	sz = (n,1)
	xAcc = np.zeros(sz)
	yAcc = np.zeros(sz)
	zAcc = np.zeros(sz)
	xGyr = np.zeros(sz)
	yGyr = np.zeros(sz)
	zGyr = np.zeros(sz)

	for i in range(0,n-1): #Sample n datapoints to establish baseline
		dataAccel = sensor.get_accel_data()
		dataGyro = sensor.get_gyro_data()
		xAcc[i] = dataAccel['x']
		yAcc[i] = dataAccel['y']
		zAcc[i] = dataAccel['z']
		xGyr[i] = dataGyro['x']
		yGyr[i] = dataGyro['y']
		zGyr[i] = dataGyro['z']
	accOff = np.sqrt(np.mean(xAcc)**2 + np.mean(yAcc)**2 + np.mean(zAcc)**2)
	xGyrOff = np.mean(xGyr)
	yGyrOff = np.mean(yGyr)
	zGyrOff = np.mean(zGyr)
	print "Calibration Complete!"
	return(accOff, xGyrOff, yGyrOff, zGyrOff)	

sensor = mpu6050(0x68)
[accOff, xGyrOff, yGyrOff, zGyrOff] = calibrate(sensor)
print(accOff, xGyrOff, yGyrOff, zGyrOff)

print "Begin rotation"
xAngle = 0
yAngle = 0
zAngle = 0
lastTime = time.time()

for i in range(0,10000):
	dataGyro = sensor.get_gyro_data()
	currentTime = time.time()
	dt = currentTime - lastTime
	xAngle += dt*(dataGyro['x'] - xGyrOff)
	yAngle += dt*(dataGyro['y'] - yGyrOff)
	zAngle += dt*(dataGyro['z'] - zGyrOff)
	lastTime = currentTime
	print (xAngle, yAngle, zAngle)
