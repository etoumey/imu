from mpu6050 import mpu6050
import math 
import datetime
import numpy as np

def calibrate(sensor):
	print "Calibrating..."
	n = 2000 #calibration sample size
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


	print (accOff, xGyrOff, yGyrOff, zGyrOff)	
	
	print "Calibration Complete!"
sensor = mpu6050(0x68)
calibrate(sensor)
