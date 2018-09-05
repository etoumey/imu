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
	n = 1000 #calibration sample size
	xAccEst = 1.0
	yAccEst = 1.0
	zAccEst = 1.0
	xGyrEst = 0.0
	yGyrEst = 0.0
	zGyrEst = 0.0
	accR = .05
	gyrR = .05
	xAccP = .5
	yAccP = .5
	zAccP = .5
	xGyrP = .5
	yGyrP = .5
	zGyrP = .5

	# Kalman Time
	for i in range(0,n-1): #Sample n datapoints to establish baseline
		dataAccel = sensor.get_accel_data()
		dataGyro = sensor.get_gyro_data()
		xAcc = dataAccel['x']
		yAcc = dataAccel['y']
		zAcc = dataAccel['z']
		xGyr = dataGyro['x']
		yGyr = dataGyro['y']
		zGyr = dataGyro['z']
		[xAccEst, xAccP] = kalman(xAccEst, xAcc, xAccP, accR)
                [yAccEst, yAccP] = kalman(yAccEst, yAcc, yAccP, accR)
                [zAccEst, zAccP] = kalman(zAccEst, zAcc, zAccP, accR)
                [xGyrEst, xGyrP] = kalman(xGyrEst, xGyr, xGyrP, gyrR)
                [yGyrEst, yGyrP] = kalman(yGyrEst, yGyr, yGyrP, gyrR)
                [zGyrEst, zGyrP] = kalman(zGyrEst, zGyr, zGyrP, gyrR)
	print "Calibration Complete!"
	print (xGyrEst, yGyrEst, zGyrEst)
	return(xAccEst, xGyrEst, yGyrEst, zGyrEst)	




# Main
sensor = mpu6050(0x68)
[accOff, xGyrOff, yGyrOff, zGyrOff] = calibrate(sensor)


print "Begin rotation"
n = 10000 # Number of Samples
xAngle = 0
yAngle = 0
zAngle = 0
lastTime = time.time()
initialTime = lastTime
outFile = open("outFile.dat", "w")

#Some more Kalman initializations
xAccEst = 1.0
yAccEst = 1.0
zAccEst = 1.0
xGyrEst = 0.0
yGyrEst = 0.0
zGyrEst = 0.0
accR = .05
gyrR = .05
xAccP = .5
yAccP = .5
zAccP = .5
xGyrP = .5
yGyrP = .5
zGyrP = .5




for i in range(0,n):
	dataGyro = sensor.get_gyro_data()
	currentTime = time.time()
	dt = currentTime - lastTime
#	[xGyrEst, xGyrP] = kalman(xGyrEst, dataGyro['x'] - xGyrOff, xGyrP,gyrR)
#        [yGyrEst, yGyrP] = kalman(yGyrEst, dataGyro['y'] - yGyrOff, yGyrP,gyrR) 
#        [zGyrEst, zGyrP] = kalman(zGyrEst, dataGyro['z'] - zGyrOff, zGyrP,gyrR) 
	xGyrEst = dataGyro['x'] - xGyrOff
	yGyrEst = dataGyro['y'] - yGyrOff
	zGyrEst = dataGyro['z'] - zGyrOff
	xAngle += dt*(xGyrEst)
	yAngle += dt*(yGyrEst)
	zAngle += dt*(zGyrEst)
	lastTime = currentTime
	outFile.write("%f %f %f %f\n" % (currentTime - initialTime,xAngle, yAngle, zAngle))
outFile.close()
