from mpu6050 import mpu6050
import math 
import time
import numpy as np

#def euler(t1, t2, t3, e): #Euler parameter update




def dcm(t1, t2, t3, C): #Body-three 3-1-2 since it's the only one I could find in my AAE440 hw
	s1 = math.sin(t1)
	s2 = math.sin(t2)
	s3 = math.sin(t3)
	c1 = math.cos(t1)
	c2 = math.cos(t2)
	c3 = math.cos(t3)

	A = [[-s1*s2*s3+c3*c1, -s1*c2, s1*s2*c3+s3*c1],[c1*s2*s3+c3*s1, c1*c2, -c1*s2*c3+s3*s1],[-c2*s3, s2, c2*c3]]
	return np.matmul(C,A);


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
	g = [xGyrEst, yGyrEst, zGyrEst]
	f = [xAccEst, yAccEst, zAccEst]
	return f,g;




# Main
sensor = mpu6050(0x68)
[f, g] = calibrate(sensor)

print "Begin rotation"
n = 100 # Number of Samples
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


C = [[1,0,0],[0,1,0],[0,0,1]]

for i in range(0,n):

	currentTime = time.time()
	xGyrEst = 0
	yGyrEst = 0
	zGyrEst = 0	
	for j in range (0,20):
		dataGyro = sensor.get_gyro_data()
		xGyrEst += dataGyro['x'] - g[0]
		yGyrEst += dataGyro['y'] - g[1]
		zGyrEst += dataGyro['z'] - g[2]
	dt = currentTime - lastTime
	C = dcm(dt*(xGyrEst/20), dt*(yGyrEst/20), dt*(zGyrEst/20), C)
	lastTime = currentTime
	outFile.write("%f %f %f %f\n" % (currentTime - initialTime, C.item(1), C.item(2), C.item(2)))
	print C	
outFile.close()

