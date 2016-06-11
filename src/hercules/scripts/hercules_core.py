#!/usr/bin/env python

import SimpleHTTPServer
import SocketServer
import thread
import os
import glob
import rospy
import atexit
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from BaseHTTPServer import BaseHTTPRequestHandler,HTTPServer
from PIL import Image

httpd = None
sensorData = {}
lidarData = {'maxRange' : -1, 'maxRangeAngle' : -1}

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
            s.wfile.write("<html><head><title>Title goes here.</title><meta http-equiv='refresh' content='30'></head>")
            s.wfile.write("<body><p>This is a test.</p>")
            # If someone went to "http://something.somewhere.net/foo/bar/",
            # then s.path equals "/foo/bar/".
            s.wfile.write("<p>You accessed path: %s</p>" % s.path)
            s.wfile.write("<br>Rover Sensors: %s</p>" % sensorData)
            s.wfile.write("<br>Lidar Data: %s</p>" % lidarData)
            s.wfile.write("<br><img src='/map'</img>")
            s.wfile.write("</body></html>")

pub = rospy.Publisher('hercules/debug', String, queue_size=10)


def scanCb(msg):
    count = 0
    outStr = ''
    for ray in msg.ranges:

        if( ray < msg.range_min or ray > msg.range_max ):
            continue

        ray = ray * 100
        if( lidarData['maxRange'] < 0  or lidarData['maxRange'] < ray):
            lidarData['maxRange'] = ray
            lidarData['maxRangeAngle'] = count

        count += 1
    outStr = `lidarData`
    rospy.loginfo('LaserScan: ' + outStr)

def roverCb(msg):
    for nextSensor in msg.data.split(';',6):
        id, value = nextSensor.split(':',2) 
        sensorData[id] = value

    #A:0;P:31;IR:0;RE:0;LE:0;LM:
    rospy.loginfo("Rover: " + `sensorData`)

def hercules_core():
    rospy.init_node('hercules_core', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, scanCb)
    rospy.Subscriber("/hercules/sensors", String, roverCb)
    rospy.spin()

def web_ui():
    global httpd 
    PORT = 8000
    Handler = MyHandler
    httpd = SocketServer.TCPServer(("", PORT), Handler)
    atexit.register(shutdown_web_ui)
    rospy.loginfo('HTTP Server started on port: %d',  PORT)
    httpd.serve_forever()

def shutdown_web_ui():
    global httpd 
    print "Shutting down..."
    httpd.shutdown()
    httpd.server_close()

if __name__ == '__main__':
    try:
        thread.start_new_thread( web_ui, ())
        hercules_core()
    except rospy.ROSInterruptException:
        pass