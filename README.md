# ros_hercules
A basic autonomous rover built on ROS Indigo and using Seeduino's Hercules base with an RPLIDAR 360 for mapping and an Nvidia Tegra TX1 for higher level robot functions.

![alt tag](https://raw.githubusercontent.com/avirtuos/ros_hercules/master/doc/img/rover_slam.gif)

# Intro

I recently read about the impressive performance people are getting from tiny (< 10 watt) Nvidia-embedded Jetson TX1 platform. This SOC has 256 GPU Cores courtesy of Nvidia's Maxwell architecture. Thanks to the CUDA extensions for C++ or OpenCL/CV, it has become quite reasonable to use all these cores for general tasks, not just graphics. That led me to the realization that the AWS G2 instance type, with its 4 NVidia GPUs, has ~5,000 cores! There is enormous potential in this rapidly maturing area, such as the work Facebook is doing with Torch and Google with TensorFlow. Both of these projects allow for deep learning accelerated by GPUs either using OpenCV or CUDA directly. I felt these technologies could help my team deliver some innovative features for our customers; however, I didn't know enough about them to be sure. I personally learn by doing, so I needed a project where I could use this technology to solve a real world problem. I needed a way to see, first hand, what Nvidia's CUDA or OpenCV could really do when pitted against a top-of-the-line CPU in an intensive task. So, I did what any bored engineer would do I fabricated a complex problem to answer a simple question: "How difficult is it to use a GPU to speed up a largely compute-bound operation?" As a software engineer, I've always been interested in hardware because it's such a different world, yet incredibly coupled/important to what I do every day. Naturally, I decided to build a robot that could use an embedded GPU (Nvidia's Jetson TX1 ... available on Amazon ;)) to run a statistical SLAM (Simultaneous Location and Mapping) algorithm for obstacle detection and avoidance (think Roomba, but without all the annoying bumping-into-stuff). The below GIF shows the first step in this project where the rover uses a spinning laser to get distance information and turns that into a map of the room the rover is currently in. Then, as the rover moves, the software uses a statistical model to "guess" (with relatively high accuracy) the location of the rover in the room given the new set of distances from objects it has seen before. 

If you find this interesting, my team at Amazon's NYC headquarters is hiring all levels of engineers! Principal Engineers included. Message me with your resume for more info. If you come for an interview and do well, you might just find out why I'm so excited about the prospect of using the AWS G2 Instance type to accelerate our workloads! <a href='https://www.linkedin.com/in/avirtuos'>Contact me on LinkedIn</a>

# Parts List

The core of this robotics platform is based around NVidia's Jetson TX1 Single Chip Computer, an RPLidar 360 Laser Scanner, and a LynxMotion 4WD Rover. You should feel free to substitude similar components if you wish but I've specifically selected the Jetson TX1 because of its lower power usage, broad support for <a href='http://www.ros.org/'>ROS (Robot Operating System)</a>, and the 256 Cuda Compute Cores on its embedded graphics processor.

* <a href="http://www.nvidia.com/object/jetson-tx1-dev-kit.html">Nvidia Jetson TX1 Module + Dev Kit</a> ($300 after educational discount)
* <a href="http://www.connecttech.com/sub/products/ASG003.asp">Orbitty Carier Board ~$148</a>
* <a href="https://www.amazon.com/gp/product/B00LGC2CTI/ref=oh_aui_detailpage_o05_s00?ie=UTF8&psc=1">RPLidar 360 ~ $440</a>
* <a href="http://www.robotshop.com/en/4wd1-robot-aluminum-kit.html">Lynxmotion Aluminum A4WD1 Rover Kit ~ $220</a>
* <a href="http://www.robotshop.com/en/lynxmotion-qme-01-quadrature-encoder.html">2 x Lynxmotion Quadrature Motor Encoder (with Cable) ~ $30</a>
* <a href="http://www.robotshop.com/en/sabertooth-dual-regenerative-motor-driver.html">Sabertooth Dual 12A 6V-24V Regenerative Motor Driver ~ $80</a>
* <a href="https://www.amazon.com/gp/product/B019SXN84E/ref=oh_aui_detailpage_o01_s00?ie=UTF8&psc=1">Arduino Pro Micro ~ $8
* <a href="https://www.amazon.com/gp/product/B018U19MN6/ref=oh_aui_detailpage_o02_s00?ie=UTF8&psc=1">LM317 Step Down Converter  ~ $2</a>
* <a href="https://www.amazon.com/gp/product/B00SL0U3RG/ref=oh_aui_detailpage_o02_s00?ie=UTF8&psc=1">2 x USB Seriat UART ~ $5.44</a>
* <a href="https://www.amazon.com/gp/product/B00W77C2FA/ref=oh_aui_detailpage_o05_s00?ie=UTF8&psc=1">16 GB Micro SdCard ~ $9</a>
* <a href="https://www.amazon.com/gp/product/B00MU44JS8/ref=oh_aui_detailpage_o01_s00?ie=UTF8&psc=1">Tamiya Connectors ~ $10</a>
* <a href="https://www.amazon.com/gp/product/B00WEBJRE8/ref=oh_aui_detailpage_o04_s01?ie=UTF8&psc=1">LM2596 Voltage and Current Regulator ~ $18</a>
* <a href="https://www.amazon.com/gp/product/B0064SHG0Y/ref=oh_aui_detailpage_o04_s01?ie=UTF8&psc=1">Low Voltage Monitor for 2S to 8S LiPO Batteries ~$10</a>
* <a href="https://www.amazon.com/gp/product/B00NAB8VQG/ref=oh_aui_detailpage_o01_s01?ie=UTF8&psc=1">Set of 20 4-Pin Plug Male and Female Wire Cable Connectors ~ $7</a>
* <a href="https://www.amazon.com/gp/product/B00MMWDYI4/ref=oh_aui_detailpage_o02_s00?ie=UTF8&psc=1">Assorted Spacers and Stand Offs</a>
 
# Software List

* Comming Soon!

# Step By Step Instructions

* Comming Soon!

## Step 1: Jetson TX1 Setup

* Comming Soon!

## Step 2: RPLidar Setup

* Comming Soon!

Includes building some Jetson TX1 kernel modules for our USB UART, which were oddly excluded from the Jetson default build but already present in the kernel source!

## Step 3: Lynxmotion Platform Setup

* Comming Soon!


# Some Lessons Learned


## SoftSerial on Leonardo, Pro Micro, ATmega32u4

SoftSerial is notoriously difficult to get to work with any ATmega32u4 based Arduino due to an error in the interrupt table. The key is to ensure you are using pin 9 for RX, you can use pretty much any pin you like for TX. 

If you don't use pin 9 for RX, what you'll see is that SoftSerial.read() will always return -1 even though there is indeed data to be read. This is because the interupts are not properly set on many (maybe all) ATmega32u4 based Arduinos. I confirmed this with my own set of Pro Micro(s) and a Leonardo. 

In fact I spent several hours troubleshooting why I could not get any data from RPLidar 360 using their own Arduino Library when using SoftSerial but it worked when using HardwareSerial. Thankfully I happened to have <a href='https://www.amazon.com/Saleae-8-Channel-Logic-Analyzer/dp/B018RE3O7G/'>a Logic analyzer</a> handy and confirmed that my TX was being sent and the RPLidar was indeed responding on my RX line but SoftSerial could not see it.

I didn't fully undertand the difference between interrupts and the variety of pin change interrupt used by SoftSerial until after my several hours of troubleshooting. You can find more on differences <a href='http://www.geertlangereis.nl/Electronics/Pin_Change_Interrupts/PinChange_en.html'>Here</a>.

## RPLidar 360 Arduino Performance

At some point I decided to try and use the amazing Pro Micro (ATmega32u4) to process laser scan data from the RPLidar. After a bit of soldering, and the above SoftSerial nightmare, and the below ATmega32u4 bricking nightmare... I was finally able to read laser data from the arduino using a software serial port. Here is what I learned:

If you plan to do anything but trival activities (Whats direction is the closest obstacle), don't bother doing it on an Arduino. I was originally hoping to use the Arduino to reduce the number of USB ports needed by the Jetson TX1 but found that the ATmega32u4 could only read:

* ~110 samples a second from the RPLidar when also publishing the samples via ROS Serial for Arduino.
* ~900 samples a second from the RPLidar with minimal processing (min direction).
* ~1500 samples a second from the RPLidar with minimal processing (min direction) and <a href='https://github.com/robopeak/rplidar_arduino/pull/7'>my RPLidar perfomance patch</a>

* RPLidar is capable of 2000 samples a second.

This was rather disappointing but reaffirmed my choice of using the Jetson TX1 because even while processing 2,000 samples a second from the RPLidar it is able to run a hector_mapping node, save generated maps, publish to a remote RViz instance, and run my custom robot software.


## Bricking ATmega32u4 Is Way Too Easy!

If you plan on using an Arduino Leonardo, the Pro Micro, or anything based on the ATmega32u4 you should be careful _not_ to upload a sketch that causes a reset loop. This is because the ATmega32u4 actually uses a compile time injected piece of code that allows for programming of the board. You WILL brick your arduino if you submit a sketch like the one below.

```c++

void setup(){
	
}

void loop(){
	Print* myPrinter = null;
	pointer->->print("This call triggers an NPE and reset of the board because myPrinter is null!");
}

```

I bricked three ATmega32u4s before realizing what was going on, oddly there is VERY little online about how easy it is to brick them. There are some tricks to try and recover from this but in the above example none of them will work because the board resets so quickly that your Operating Systems's USB port will not be able to recognize the device. Your only option at that point is to program the chip directly, assuming your board has the appropriate pinouts... unlike the Pro Micro.

So, as a safety I recommend always adding a short sleep to the setup() method so you ensure your Operating System will have time to recognize the device and allow you to upload a corrected sketch.

```c++

void setup(){
	delay(4000);	//sufficient for USB to be detected and sketch upload to begin.
}

void loop(){
	Print* myPrinter = null;
	pointer->->print("This call triggers an NPE and reset of the board because myPrinter is null!");
}

```
