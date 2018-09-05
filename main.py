from mpu6050 import mpu6050
import math 
import time
import numpy as np

def kalman(xEst, x, P, R):
	K = P/(P+R)
	xNew = xEst + K*(x - xEst)
	P = (1-K)*P
	return xNew, P;

def calibrate(sensor): #Implement Kalman Filter to quickly calibrate sensor
	print "Calibrating..."
	n = 100 #calibration sample size
	sz = (n,1)
	xAcc = np.zeros(sz)
	yAcc = np.zeros(sz)
	zAcc = np.zeros(sz)
	xGyr = np.zeros(sz)
	yGyr = np.zeros(sz)
	zGyr = np.zeros(sz)
	xAccEst = 1.0
	yAccEst = 1.0
	zAccEst = 1.0
	xGyrEst = 1.0
	yGyrEst = 1.0
	zGyrEst = 1.0
	accR = .1
	gyrR = .1
	xAccP = 1.0
	yAccP = 1.0
	zAccP = 1.0
	xGyrP = 1.0 
	yGyrP = 1.0
	zGyrP = 1.0

	# Kalman Time
	for i in range(0,n-1): #Sample n datapoints to establish baseline
		dataAccel = sensor.get_accel_data()
		dataGyro = sensor.get_gyro_data()
		xAcc = dataAccel['x']
		yAcc[i] = dataAccel['y']
		zAcc[i] = dataAccel['z']
		xGyr[i] = dataGyro['x']
		yGyr[i] = dataGyro['y']
		zGyr[i] = dataGyro['z']
		xAccEst, xAccP = kalman(xAccEst, xAcc, xAccP, accR)
#                yAccEst, yAccP = kalman(yAccEst, yAcc[i], yAccP, accR)
#                zAccEst, zAccP = kalman(zAccEst, zAcc[i], zAccP, accR)
#                xGyrEst, xGyrP = kalman(xGyrEst, zGyr[i], xGyrP, gyrR)
#                yGyrEst, yGyrP = kalman(yGyrEst, yGyr[i], yGyrP, gyrR)
#                zGyrEst, zGyrP = kalman(zGyrEst, zGyr[i], zGyrP, gyrR)
		print (xAccEst, xGyrEst, yGyrEst)
	print "Calibration Complete!"
	return(xAccEst, xGyrEst, yGyrEst, zGyrEst)	

sensor = mpu6050(0x68)
accOff, xGyrOff, yGyrOff, zGyrOff = calibrate(sensor)
#print(accOff, xGyrOff, yGyrOff, zGyrOff)

print "Begin rotation"
xAngle = 0
yAngle = 0
zAngle = 0
lastTime = time.time()
initialTime = lastTime
outFile = open("outFile.dat", "w")

for i in range(0,1):
	dataGyro = sensor.get_gyro_data()
	currentTime = time.time()
	dt = currentTime - lastTime
	xAngle += dt*(dataGyro['x'] - xGyrOff)
	yAngle += dt*(dataGyro['y'] - yGyrOff)
	zAngle += dt*(dataGyro['z'] - zGyrOff)
	lastTime = currentTime
	outFile.write("%f %f %f %f\n" % (currentTime - initialTime,xAngle, yAngle, zAngle))
	print (xAngle, yAngle, zAngle)
outFile.close()
