#!/usr/bin/env python

import SimpleHTTPServer
import SocketServer
import thread
import os
import glob
import time
import rospy
import atexit
from std_msgs.msg import String, Int16
from sensor_msgs.msg import LaserScan
from BaseHTTPServer import BaseHTTPRequestHandler,HTTPServer
from PIL import Image
from time import sleep
from urlparse import urlparse, parse_qs
from socket import error as socket_error
from math import sin, cos, pi, ceil

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster


class OdometryPub:
    def __init__(self):
        self.ticks_meter = float(100)  # The number of wheel encoder ticks per meter of travel
        self.base_width = float(0.129) # The wheel base width in meters
        
        self.base_frame_id = 'base_link' # the name of the base frame of the robot
        self.odom_frame_id = 'odom' # the name of the odometry reference frame
        
        self.encoder_min = -32768
        self.encoder_max = 32768
        self.encoder_low_wrap = (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min 
        self.encoder_high_wrap = (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min 
        
        # internal data
        self.enc_left = None        # wheel encoder readings
        self.enc_right = None
        self.left = 0               # actual values coming back from robot
        self.right = 0
        self.x = 0                  # position in xy plane 
        self.y = 0
        self.th = 0
        self.dx = 0                 # speeds in x/rotation
        self.dr = 0
        self.then = 0
        
        self.odomPub = rospy.Publisher("/odom", Odometry,queue_size=10)
        self.odomBroadcaster = TransformBroadcaster()
     
    def update(self, left, right):
        if(self.then == 0):
            self.then = rospy.Time.now()
            return

        self.right = right;
        self.left = left;
        now = rospy.Time.now()
        elapsed = now - self.then
        self.then = now
        elapsed = elapsed.to_sec()
            
        # calculate odometry
        if self.enc_left == None:
            d_left = 0
            d_right = 0
        else:
            d_left = (self.left - self.enc_left) / self.ticks_meter
            d_right = (self.right - self.enc_right) / self.ticks_meter
        self.enc_left = self.left
        self.enc_right = self.right
          
        # distance traveled is the average of the two wheels 
        d = ( d_left + d_right ) / 2
        # this approximation works (in radians) for small angles
        th = ( d_right - d_left ) / self.base_width
        # calculate velocities
        self.dx = d / elapsed
        self.dr = th / elapsed
           
             
        if (d != 0):
            # calculate distance traveled in x and y
            x = cos( th ) * d
            y = -sin( th ) * d
            # calculate the final position of the robot
            self.x = self.x + ( cos( self.th ) * x - sin( self.th ) * y )
            self.y = self.y + ( sin( self.th ) * x + cos( self.th ) * y )
        if( th != 0):
            self.th = self.th + th
                
        # publish the odom information
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = sin( self.th / 2 )
        quaternion.w = cos( self.th / 2 )
        self.odomBroadcaster.sendTransform(
            (self.x, self.y, 0),
            (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
            rospy.Time.now(),
            self.base_frame_id,
            self.odom_frame_id
        )
            
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0
        odom.pose.pose.orientation = quaternion
        odom.child_frame_id = self.base_frame_id
        odom.twist.twist.linear.x = self.dx
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = self.dr
        self.odomPub.publish(odom)
            

class MotorController:
    maxSpeed = 8
    rosPub = None
    leftMotor = 0
    rightMotor = 0
    leftEncoder = 0
    rightEncoder = 0
    leftEncoderDelta = 0
    rightEncoderDelta = 0
    lastPublish = {}
    isHalted = True
    leftAccel = 0
    rightAccel = 0

    requiredLeft = 0
    requiredRight = 0
    owner = ""
    wayPointTimeout = 0
    odom = OdometryPub()

    oneRotation = 6000.0
    circumference = 39.2399

    def __init__(self):
        self.rosPub = rospy.Publisher('/hercules/motorCtrl', String, queue_size=10)

    def updateOdometry(self, leftEncoder, rightEncoder):
        realLeftEncoder = ((leftEncoder/self.oneRotation) * self.circumference)
        realRightEncoder = ((rightEncoder/self.oneRotation) * self.circumference)

        #if(leftEncoder != 0 or rightEncoder != 0):
        #    rospy.loginfo('updateOdometry: CR: ' +  str(self.rightEncoder)  + ' CL: ' + str(self.leftEncoder) + ' NR: ' +  str(rightEncoder)  + ' NL: ' + str(leftEncoder) + ' RR: ' +  str(realRightEncoder)  + ' RL: ' + str(realLeftEncoder))

        self.leftEncoderDelta = realLeftEncoder
        self.rightEncoderDelta = realRightEncoder
        self.leftEncoder = self.leftEncoder  + realLeftEncoder
        self.rightEncoder = self.rightEncoder + realRightEncoder

        if(self.isHalted):
            return

        #self.odom.update(leftEncoder, rightEncoder)
        self.evaluateWayPoint()

        self.handleAccelerationLeft(self.leftAccel)
        self.handleAccelerationRight(self.rightAccel)
        self.publish()

    def evaluateWayPoint(self):
        if(self.leftMotor == 0 and self.rightMotor == 0):
            return

        if(self.requiredRight <= self.rightEncoder and self.requiredLeft <= self.leftEncoder):
            rospy.loginfo('evaluateWayPoint: waypoint completed')
            self.rightAccel = 0
            self.leftAccel = 0
            self.owner = ""
            self.stop()
        
        if(self.leftEncoderDelta > (2 * self.maxSpeed)):
            rospy.loginfo('evaluateWayPoint: Far over speed, lowing down left aggressively: ' + str(self.leftEncoderDelta) + ' vs max: ' + str(self.maxSpeed))
            self.leftMotor = self.leftMotor / (self.leftEncoderDelta / self.maxSpeed);
        elif(self.leftEncoderDelta > self.maxSpeed):
            rospy.loginfo('evaluateWayPoint: Slowing down left: ' + str(self.leftEncoderDelta) + ' vs max: ' + str(self.maxSpeed))
            self.leftMotor = self.leftMotor - 1

        if(self.rightEncoderDelta > (2 * self.maxSpeed)):
            rospy.loginfo('evaluateWayPoint: Far over speed, lowing down right aggressively: ' + str(self.rightEncoderDelta) + ' vs max: ' + str(self.maxSpeed))
            self.rightMotor = self.rightMotor / (self.rightEncoderDelta / self.maxSpeed);
        elif(self.rightEncoderDelta > self.maxSpeed):
            rospy.loginfo('evaluateWayPoint: Slowing down right: ' + str(self.rightEncoderDelta) + ' vs max: ' + str(self.maxSpeed))
            self.rightMotor = self.rightMotor - 1

    def setWayPoint(self, owner, requiredDistance):
        if(time.time() - self.wayPointTimeout > 20):
            rospy.loginfo('setWayPoint: Timeout - Owner is was ' + owner + '! Left: ' + str(self.requiredLeft) + ' Right: ' + str(self.requiredLeft))
            self.requiredRight = self.rightEncoder
            self.requiredLeft = self.leftEncoder
        elif(self.owner == owner):
            #rospy.loginfo('setWayPoint: Owner is me (' + owner + ')! Left: ' + str(self.requiredLeft) + ' Right: ' + str(self.requiredLeft) + ' Actual -> Left: ' + str(self.leftEncoder) + ' Right: ' + str(self.rightEncoder))
            return True
        elif(self.requiredRight > self.rightEncoder and self.requiredLeft > self.leftEncoder):
            #rospy.loginfo('setWayPoint(' + owner +'): Existing waypoint exists! Left: ' + str(self.requiredLeft) + ' Right: ' + str(self.requiredLeft))
            return False

        self.rightAccel = 0
        self.leftAccel = 0
        self.owner = ""
        self.stop()
        self.requiredRight = self.rightEncoder + requiredDistance
        self.requiredLeft = self.leftEncoder + requiredDistance
        self.owner = owner
        self.wayPointTimeout = time.time()
        rospy.loginfo('setWayPoint: Set waypoint Left: ' + str(self.requiredLeft) + ' Right: ' + str(self.requiredLeft))
        return True

    def handleAccelerationLeft(self, direction):
        if(self.leftMotor < 1 and direction > 1):
            self.leftMotor = 0
        elif(self.leftMotor > 1 and direction < 1):
            self.leftMotor = 0
            
        if(self.leftEncoderDelta < self.maxSpeed):
            self.leftMotor += (4 * direction)

    def handleAccelerationRight(self, direction):
        if(self.rightMotor < 1 and direction > 1):
            self.rightMotor = 0
        elif(self.rightMotor > 1 and direction < 1):
            self.rightMotor = 0

        if(self.rightEncoderDelta < self.maxSpeed):
            self.rightMotor += (4 * direction)

    def forward(self, requiredDistance):
        if(self.isHalted):
            return

        if(not self.setWayPoint("forward", requiredDistance)):
            return

        self.leftAccel = 1
        self.rightAccel = 1

    def turnRight(self):
        if(self.isHalted):
            return

        if(not self.setWayPoint("turnRight", random.randint(10, 30))):
            return

        self.leftAccel = 1
        self.rightAccel = -1

    def turnLeft(self):
        if(self.isHalted):
            return

        if(not self.setWayPoint("turnLeft", random.randint(10, 30))):
            return

        self.leftAccel = -1
        self.rightAccel = 1

    def reverse(self, requiredDistance):
        if(self.isHalted):
            return

        if(not self.setWayPoint("reverse", requiredDistance)):
            return

        self.leftAccel = -1
        self.rightAccel = -1

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
            self.requiredRight = 0
            self.requiredLeft = 0
            self.wayPointTimeout = 0
            self.wayPointOwner = None

    def stop(self):
        self.leftMotor = 0
        self.rightMotor = 0
        self.publish()

    def publish(self):
        global lidarData
        #RIGHT MOTOR 1  - ( 1 - reverse - 64 - forward - 127)
        #LEft MOTOR - ( 128 - reverse - 192 - forward - 255)
        leftMotorSpeedCmd = int(ceil(self.leftMotor)) + 192
        if(leftMotorSpeedCmd < 128):
            leftMotorSpeedCmd = 128
        elif(leftMotorSpeedCmd > 255):
            leftMotorSpeedCmd = 255

        rightMotorSpeedCmd = int(ceil(self.rightMotor)) + 64
        if(rightMotorSpeedCmd < 1):
            rightMotorSpeedCmd = 1
        elif(rightMotorSpeedCmd > 127):
            rightMotorSpeedCmd = 127

        if(rightMotorSpeedCmd == 127 and self.rightEncoderDelta == 0):
            rospy.loginfo('No motor feedback but at full throttle right, reversing')
            self.wayPointTimeout = 0
            self.reverse(lidarData['rearMin'])
            return

        if(leftMotorSpeedCmd == 255 and self.leftEncoderDelta == 0):
            rospy.loginfo('No motor feedback but at full throttle left, reversing')
            self.wayPointTimeout = 0
            self.reverse(lidarData['rearMin'])
            return
        
        if(rightMotorSpeedCmd == 1 and self.rightEncoderDelta == 0):
            rospy.loginfo('No motor feedback but at full throttle right, forward')
            self.wayPointTimeout = 0
            self.forward(min(lidarData['frontMin'], 50))
            return

        if(leftMotorSpeedCmd == 128 and self.leftEncoderDelta == 0):
            rospy.loginfo('No motor feedback but at full throttle left, forward')
            self.wayPointTimeout = 0
            self.forward(min(lidarData['frontMin'], 50))
            return

        leftMotorDirCmd = 'F' if self.leftMotor > 0 else 'R'
        rightMotorDirCmd = 'F' if self.rightMotor > 0 else 'R'

        self.lastPublish['leftMotorSpeed'] = leftMotorSpeedCmd
        self.lastPublish['rightMotorSpeed'] = rightMotorSpeedCmd
        self.lastPublish['requiredLeft'] = self.requiredLeft
        self.lastPublish['requiredRight'] = self.requiredRight
        self.lastPublish['leftEncoder'] = self.leftEncoder
        self.lastPublish['rightEncoder'] = self.rightEncoder
        self.lastPublish['wayPointOwner'] = self.owner

        if(leftMotorSpeedCmd != 192 or rightMotorSpeedCmd != 64):
            rospy.loginfo('MotorControlCmd(' + self.owner + '): L:' + str(self.leftMotor) + ' R:' + str(self.rightMotor) + ' RL:' + str(leftMotorSpeedCmd) + ' RR:' + str(rightMotorSpeedCmd) + ' requiredRight: ' + str(self.requiredRight) + ' rightEncoder: ' + str(self.rightEncoder) + ' requiredLeft: ' + str(self.requiredLeft) + ' leftEncoder: '  + str(self.leftEncoder))
        self.rosPub.publish(chr(leftMotorSpeedCmd)  + chr(rightMotorSpeedCmd))

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

            newest = max(glob.iglob('/home/ubuntu/tmp/maps/*.tif'), key=os.path.getctime)
            im = Image.open(newest)
            im.thumbnail(im.size)
            im.save('/home/ubuntu/tmp/maps/newest.jpeg', "JPEG", quality=100)
            f = open('/home/ubuntu/tmp/maps/newest.jpeg')
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
            elif('turnRight' in params):
                motorController.halt(False)
                motorController.turnRight()
                titleSuffix = 'Turning Right...'
            elif('turnLeft' in params):
                motorController.halt(False)
                motorController.turnLeft()
                titleSuffix = 'Turning Left...'
            elif('reverse' in params):
                motorController.halt(False)
                motorController.reverse(40)
                titleSuffix = 'Reversing...'


            s.wfile.write("<html><head><title>Hercules Management Console" + titleSuffix + "</title><meta http-equiv='refresh' content='10'></head><body>")
            s.wfile.write("<br><a href='/?halt=true'>click to halt</a>");
            s.wfile.write("<br><a href='/?resume=true'>click to resume</a>");
            s.wfile.write("<br><a href='/?reverse=1'>click to reverse</a>");
            s.wfile.write("<br><a href='/?turnLeft=1'>click to turn Left</a>");
            s.wfile.write("<br><a href='/?turnRight=1'>click to turn Right</a>");

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




lastMsg = ""

def scanCb(msg):
    global lidarData
    global lastMsg
    count = 0
    outStr = ''
    lidarData = {'discarded': -1, 'maxRange' : -1, 'maxRangeAngle' : -1, 'minRange' : -1, 'minRangeAngle' : -1, 'frontMin': -1, 'frontAvg': -1, 'rearMin': -1, 'rearAvg': -1}
    newMsg = ""
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
        if( count < 300 and count > 230):
            frontCount += 1
            frontSum += ray
            if( lidarData['frontMin'] == -1 or lidarData['frontMin'] > ray):
                lidarData['frontMin'] = ray

        #55 is a good range for rear
        if( count <100 and count > 60):
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

    if(lidarData['rearMin'] == -1):
        lidarData['rearMin'] = 80

    lidarData['frontAvg'] = (frontSum/frontCount) if (frontCount > 0)  else -1
    lidarData['rearAvg'] = (rearSum/rearCount) if (rearCount > 0)  else -1 
    lidarData['discarded'] = discards
    outStr = `lidarData`
    if(lidarData['minRange'] < 25):
        rospy.loginfo('LaserScan: ' + outStr)

    if( lidarData['frontMin'] > 80):
        motorController.forward(ceil(lidarData['frontMin'] - 30))
    elif ( lidarData['frontMin'] <= 30):
        newMsg = "Reversing to avoid forward obstacle."
        motorController.reverse(min(lidarData['rearMin'], 80))
    elif(lidarData['minRange'] < 50 and (lidarData['minRangeAngle'] > 100 and lidarData['minRangeAngle'] < 230)):
        newMsg = "Turning left to avoid proxity on right."
        motorController.turnLeft()
    elif(lidarData['minRange'] < 50 and (lidarData['minRangeAngle'] > 300 or lidarData['minRangeAngle'] < 60)):
        newMsg = "Turning right to avoid proxity on left."
        motorController.turnRight()
    elif ( lidarData['frontMin'] <= 81):
        if(lidarData['maxRangeAngle'] > 100 and lidarData['maxRangeAngle'] < 230):
            newMsg = "Turning right to avoid forward obstacle."
            motorController.turnRight()
        else:
            newMsg = "Turning left to avoid forward obstacle."
            motorController.turnLeft()

    if(lastMsg != newMsg):
        rospy.loginfo(newMsg)
        lastMsg = newMsg
            
def roverCb(msg):
    for nextSensor in msg.data.split(';',6):
        values = nextSensor.split(':',2) 
        if(len(values) == 2):
            sensorData[values[0]] = values[1]

    motorController.updateOdometry(abs(int(sensorData['LE'])), abs(int(sensorData['RE'])))
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
