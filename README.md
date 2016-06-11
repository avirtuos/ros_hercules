# ros_hercules
A basic autonomous rover built on ROS Indigo and using Seeduino's Hercules base with an RPLIDAR 360 for mapping and an Nvidia Tegra TX1 for higher level robot functions.

![alt tag](https://raw.githubusercontent.com/avirtuos/ros_hercules/master/doc/img/rover_slam.gif)

# Intro

I recently read about the impressive performance people are getting from tiny (< 10 watt) Nvidia-embedded Jetson TX1 platform. This SOC has 256 GPU Cores courtesy of Nvidia's Maxwell architecture. Thanks to the CUDA extensions for C++ or OpenCL/CV, it has become quite reasonable to use all these cores for general tasks, not just graphics. That led me to the realization that the AWS G2 instance type, with its 4 NVidia GPUs, has ~5,000 cores! There is enormous potential in this rapidly maturing area, such as the work Facebook is doing with Torch and Google with TensorFlow. Both of these projects allow for deep learning accelerated by GPUs either using OpenCV or CUDA directly. I felt these technologies could help my team deliver some innovative features for our customers; however, I didn't know enough about them to be sure. I personally learn by doing, so I needed a project where I could use this technology to solve a real world problem. I needed a way to see, first hand, what Nvidia's CUDA or OpenCV could really do when pitted against a top-of-the-line CPU in an intensive task. So, I did what any bored engineer would do I fabricated a complex problem to answer a simple question: "How difficult is it to use a GPU to speed up a largely compute-bound operation?" As a software engineer, I've always been interested in hardware because it's such a different world, yet incredibly coupled/important to what I do every day. Naturally, I decided to build a robot that could use an embedded GPU (Nvidia's Jetson TX1 ... available on Amazon ;)) to run a statistical SLAM (Simultaneous Location and Mapping) algorithm for obstacle detection and avoidance (think Roomba, but without all the annoying bumping-into-stuff). The below GIF shows the first step in this project where the rover uses a spinning laser to get distance information and turns that into a map of the room the rover is currently in. Then, as the rover moves, the software uses a statistical model to "guess" (with relatively high accuracy) the location of the rover in the room given the new set of distances from objects it has seen before. 

If you find this interesting, my team at Amazon's NYC headquarters is hiring all levels of engineers! Principal Engineers included. Message me with your resume for more info. If you come for an interview and do well, you might just find out why I'm so excited about the prospect of using the AWS G2 Instance type to accelerate our workloads! <a href='https://www.linkedin.com/in/avirtuos'>Contact me on LinkedIn</a>

# Parts List

# Software List

# Step By Step Instructions

## Step 1: Installing ROS