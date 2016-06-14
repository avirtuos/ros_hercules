#!/usr/bin/env python

import SimpleHTTPServer
import SocketServer
import thread
import os
import glob
import time
import rospy
import atexit
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from BaseHTTPServer import BaseHTTPRequestHandler,HTTPServer
from PIL import Image
from time import sleep
from urlparse import urlparse, parse_qs
from socket import error as socket_error

class MotorController:
    maxSpeed = 100
    rosPub = None
    leftMotor = 0
    rightMotor = 0
    leftEncoder = 0
    rightEncoder = 0
    lastOdometryUpdate = 0
    lastPublish = {}
    isHalted = True

    def __init__(self):
        self.rosPub = rospy.Publisher('/hercules/motorCtrl', String, queue_size=10)

    def updateOdometry(self, leftEncoder, rightEncoder):
        self.leftEncoder = leftEncoder
        self.rightEncoder = rightEncoder

    def forward(self, frontDistance):
        if(self.isHalted):
            return

        if(self.leftMotor != self.rightMotor):
            self.leftMotor = abs(self.leftMotor + self.rightMotor)/2
            self.rightMotor = self.leftMotor

        if(self.leftMotor > 0 and frontDistance / 3 < self.leftEncoder):
            rospy.loginfo('forward: approaching obstacle, slowing down left motor.')
            self.leftMotor -= self.leftMotor/3
        elif ( self.leftMotor < self.maxSpeed ):
            self.leftMotor += 1

        if(self.rightMotor > 0 and frontDistance / 3 < self.rightEncoder):
            rospy.loginfo('forward: approaching obstacle, slowing down right motor.')
            self.rightMotor -= self.rightMotor/3
        elif ( self.rightMotor < self.maxSpeed ):
            self.rightMotor += 1
        self.publish()

    def turnRight(self, turnInPlace):
        if(self.isHalted):
            return

        if(self.leftMotor == self.rightMotor):
            self.leftMotor = self.leftMotor / 3
            self.rightMotor = self.rightMotor / 3

        if( turnInPlace ):
            self.rightMotor = -1 * self.leftMotor

        if ( self.leftMotor < self.maxSpeed ):
            self.leftMotor += 9

        if ( self.rightMotor > 0 or abs(self.rightMotor) < self.maxSpeed ):
            self.rightMotor -= 9

        self.publish()

    def turnLeft(self, turnInPlace):
        if(self.isHalted):
            return

        if(self.leftMotor == self.rightMotor):
            self.leftMotor = self.leftMotor / 3
            self.rightMotor = self.rightMotor / 3

        if( turnInPlace ):
            self.leftMotor = -1 * self.rightMotor

        if ( self.rightMotor < self.maxSpeed ):
            self.rightMotor += 9

        if ( self.leftMotor > 0 or abs(self.leftMotor) < self.maxSpeed ):
            self.leftMotor -= 9

        self.publish()

    def reverse(self, reverseDistance):
        if(self.isHalted):
            return

        if(self.leftMotor < 0 and reverseDistance / 3 < self.leftEncoder):
            rospy.loginfo('reverse: approaching obstacle, slowing down left motor')
            self.leftMotor += abs(self.leftMotor/3)
        elif ( self.leftMotor > 0 or abs(self.leftMotor) < self.maxSpeed ):
            self.leftMotor -= 1

        if(self.rightMotor < 0 and reverseDistance / 3 < self.leftEncoder):
            rospy.loginfo('reverse: approaching obstacle, slowing down right motor')
            self.rightMotor += abs(self.rightMotor/3)
        elif ( self.leftMotor > 0 or abs(self.leftMotor) < self.maxSpeed ):
            self.rightMotor -= 1
        self.publish()

    def isReverse(self):
        if(self.leftMotor < 0 and self.rightMotor < 0):
            return True
        else:
            return False

    def halt(self, isHalted):
        self.isHalted = isHalted
        if(isHalted):
            self.leftMotor = 0
            self.rightMotor = 0
            self.publish()

    def stop(self):
        self.leftMotor = 0
        self.rightMotor = 0
        self.publish()

    def publish(self):
        leftMotorDirCmd = 'F' if self.leftMotor > 0 else 'R'
        leftMotorSpeedCmd = chr(abs(self.leftMotor) + 48)
        rightMotorDirCmd = 'F' if self.rightMotor > 0 else 'R'
        rightMotorSpeedCmd = chr(abs(self.rightMotor) + 48)
        self.lastPublish['leftMotorDir'] = leftMotorDirCmd
        self.lastPublish['leftMotorSpeed'] = abs(self.leftMotor) + 48
        self.lastPublish['rightMotorDir'] = rightMotorDirCmd
        self.lastPublish['rightMotorSpeed'] = abs(self.rightMotor) + 48
        rospy.loginfo('MotorControlCmd: ' + leftMotorSpeedCmd + leftMotorDirCmd + rightMotorSpeedCmd + rightMotorDirCmd)
        self.rosPub.publish(leftMotorSpeedCmd + leftMotorDirCmd + rightMotorSpeedCmd + rightMotorDirCmd)

    def getState(self):
        return self.lastPublish

httpd = None
sensorData = {}
lidarData = {'discarded': -1, 'maxRange' : -1, 'maxRangeAngle' : -1, 'minRange' : -1, 'minRangeAngle' : -1, 'frontMin': -1, 'frontAvg': -1, 'rearMin': -1, 'rearAvg': -1}
motorController = MotorController()

class MyHandler(BaseHTTPRequestHandler):
    def do_HEAD(s):
        if(s.path == '/map'):
            s.send_response(200)
            s.send_header("Content-type", "image/jpeg")
            s.end_headers()
        else:
            s.send_response(200)
            s.send_header("Content-type", "text/html")
            s.end_headers()
    def do_GET(s):
        if( s.path == '/map' ):
            s.send_response(200)
            s.send_header("Content-type", "image/jpeg")
            #s.send_header("Content-type", "image/gif")
            s.end_headers()
            #f = open("/mnt/sdcard/") 

            newest = max(glob.iglob('/mnt/sdcard/output/maps/*.tif'), key=os.path.getctime)
            im = Image.open(newest)
            im.thumbnail(im.size)
            im.save('/mnt/sdcard/output/maps/newest.jpeg', "JPEG", quality=100)
            f = open('/mnt/sdcard/output/maps/newest.jpeg')
            s.wfile.write(f.read())
            f.close()
        else:
            s.send_response(200)
            s.send_header("Content-type", "text/html")
            s.end_headers()

            titleSuffix = ''
            params = parse_qs(urlparse(s.path).query)
            if('halt' in params):
                motorController.halt(True)
                titleSuffix = 'Halting...'
            elif('resume' in params):
                motorController.halt(False)
                titleSuffix = 'Resuming...'


            s.wfile.write("<html><head><title>Hercules Management Console" + titleSuffix + "</title><meta http-equiv='refresh' content='10'></head><body>")
            s.wfile.write("<br><a href='/?halt=true'>click to halt</a>");
            s.wfile.write("<br><a href='/?resume=true'>click to resume</a>");
            # If someone went to "http://something.somewhere.net/foo/bar/",
            # then s.path equals "/foo/bar/".
            s.wfile.write("<br>Rover Sensors</p><br><table><tr><th>Sensor</th><th>Value</th></tr>")
            for key, value in sensorData.iteritems():
                 s.wfile.write("<tr><td>" + key + "</td><td>" + `value` + "</td></tr>")
            s.wfile.write("</table>")

            s.wfile.write("<br>Lidar Data</p><br><table><tr><th>Property</th><th>Value</th></tr>")
            for key, value in lidarData.iteritems():
                 s.wfile.write("<tr><td>" + key + "</td><td>" + `value` + "</td></tr>")
            s.wfile.write("</table>")

            s.wfile.write("<br>MotorController Data</p><br><table><tr><th>Property</th><th>Value</th></tr>")
            for key, value in motorController.getState().iteritems():
                 s.wfile.write("<tr><td>" + key + "</td><td>" + `value` + "</td></tr>")
            s.wfile.write("</table>")

            s.wfile.write("<br><img src='/map'</img>")
            s.wfile.write("</body></html>")






def scanCb(msg):
    global lidarData
    count = 0
    outStr = ''
    lidarData = {'discarded': -1, 'maxRange' : -1, 'maxRangeAngle' : -1, 'minRange' : -1, 'minRangeAngle' : -1, 'frontMin': -1, 'frontAvg': -1, 'rearMin': -1, 'rearAvg': -1}

    frontCount = 0
    frontSum = 0
    rearCount = 0
    rearSum = 0
    discards = 0
    for ray in msg.ranges:
        count += 1
        if( ray < 0.2 or ray > msg.range_max ):
            discards += 1
            continue

        ray = ray * 100

        #35 is a good range for front
        if( count < 90 and count > 45):
            frontCount += 1
            frontSum += ray
            if( lidarData['frontMin'] == -1 or lidarData['frontMin'] > ray):
                lidarData['frontMin'] = ray

        #55 is a good range for rear
        if( count > 200 and count < 220):
            rearCount += 1
            rearSum += ray
            if( lidarData['rearMin'] == -1 or lidarData['rearMin'] > ray * 1.2):
                lidarData['rearMin'] = ray

        if( lidarData['maxRange'] < 0  or lidarData['maxRange'] *1.2 < ray):
            lidarData['maxRange'] = ray
            lidarData['maxRangeAngle'] = count

        if( lidarData['minRange'] < 0  or lidarData['minRange'] > ray * 1.2):
            lidarData['minRange'] = ray
            lidarData['minRangeAngle'] = count


    lidarData['frontAvg'] = (frontSum/frontCount) if (frontCount > 0)  else -1
    lidarData['rearAvg'] = (rearSum/rearCount) if (rearCount > 0)  else -1 
    lidarData['discarded'] = discards
    outStr = `lidarData`
    #rospy.loginfo('LaserScan: ' + outStr)


    if(lidarData['minRange'] < 25 and (lidarData['minRangeAngle'] > 220 or lidarData['minRangeAngle'] < 45)):
        rospy.loginfo("Turning left to avoid proxity on right." + `lidarData`)
        motorController.turnLeft(True)
    elif(lidarData['minRange'] < 25 and (lidarData['minRangeAngle'] > 90 and lidarData['minRangeAngle'] < 200)):
        rospy.loginfo("Turning right to avoid proxity on left." + `lidarData`)
        motorController.turnRight(True)
    elif( lidarData['frontMin'] > 45):
        motorController.forward(lidarData['frontMin'])
    elif ( lidarData['frontMin'] < 45 and lidarData['frontMin'] > 35):
        if(lidarData['maxRangeAngle'] > 220 or lidarData['maxRangeAngle'] < 45):
            rospy.loginfo("Turning right to avoid forward obstacle." + `lidarData`)
            motorController.turnRight(False)
        else:
            rospy.loginfo("Turning left to avoid forward obstacle." + `lidarData`)
            motorController.turnLeft(False)
    elif ( lidarData['frontMin'] < 35 and lidarData['frontMin'] >= 25):
        if(lidarData['maxRangeAngle'] > 220 or lidarData['maxRangeAngle'] < 45):
            rospy.loginfo("Turning right (in place) to avoid forward obstacle." + `lidarData`)
            motorController.turnRight(True)
        else:
            rospy.loginfo("Turning left (in place) to avoid forward obstacle." + `lidarData`)
            motorController.turnLeft(True)
    elif ( lidarData['frontMin'] <= 25):
        rospy.loginfo("Reversing to avoid forward obstacle.")
        motorController.reverse(lidarData['rearMin'])
    
    if(lidarData['rearMin'] == -1 and motorController.isReverse()):
        rospy.loginfo("no rear data and moving in rervse, stop")
        motorController.stop()
    elif(lidarData['rearMin'] < 55 and motorController.isReverse()):
        rospy.loginfo("rear obstacle and moving in reverse, move forard instead")
        motorController.forward()
            
def roverCb(msg):
    for nextSensor in msg.data.split(';',6):
        values = nextSensor.split(':',2) 
        sensorData[values[0]] = values[1]

    motorController.updateOdometry(int(sensorData['LE']), int(sensorData['RE']))
    #A:0;P:31;IR:0;RE:0;LE:0;LM:

def hercules_core():
    rospy.init_node('hercules_core', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, scanCb)
    rospy.Subscriber("/hercules/sensors", String, roverCb)
    rospy.spin()

def web_ui():
    global httpd 
    PORT = 8000
    Handler = MyHandler
    httpd = None

    while httpd == None:
        try:
            httpd = SocketServer.TCPServer(("", PORT), Handler)
            rospy.loginfo("Started web server!")
        except socket_error as err:
            httpd = None
            rospy.loginfo("Error {0} starting http servier, retrying after sleeping 4 seconds.".format(err))
            sleep(4)

    atexit.register(shutdown)
    #rospy.loginfo('HTTP Server started on port: %d',  PORT)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        server.socket.close()

def shutdown():
    global httpd 
    print "Shutting down..."
    motorController.stop()
    httpd.shutdown()
    httpd.server_close()


if __name__ == '__main__':
    try:
        thread.start_new_thread( web_ui, ())
        hercules_core()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        shutdown()