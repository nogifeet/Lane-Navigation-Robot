# Lane-Navigation-Robot

![alt text](https://github.com/nogifeet/Lane-Navigation-Robot/blob/main/Resources/intro_img.jpg "Introduction Image")


Lane Navigation Robot using Nvidia Jetson Nano and OpenCV for lane detection and navigation. The movement of robot is decided on three factors (i)steering angle ,
(ii)Deviation and (iii)error based on three variables the motor instruction is passed on to the motor driver. 

## Circuit Diagram of the Autonomous Vehicle
![alt text](https://github.com/nogifeet/Lane-Navigation-Robot/blob/main/Resources/circuit.jpeg "Circuit Diagram")

#Images of the Lane Navigation Robot

![alt text](https://github.com/nogifeet/Lane-Navigation-Robot/blob/main/Resources/img_1%20(1).jpeg "Image_1")

![alt text](https://github.com/nogifeet/Lane-Navigation-Robot/blob/main/Resources/img_1%20(2).jpeg "Image_1")

![alt text](https://github.com/nogifeet/Lane-Navigation-Robot/blob/main/Resources/img_1%20(3).jpeg "Image_1")

##File Structure
motor_control.py allows Jetson Nano to send motor instructions to motor driver for movement.
motor_user_control.py with the Tkinter library allows user to control the robot using keyboard instruction.
lane_detection_check.py reads frame from camera, calculates steering angle, error and deviation and sends motor instruction using motor_control.py

## GIF Demonstration
![Alt Text](https://github.com/nogifeet/Lane-Navigation-Robot/blob/main/Resources/frame_1.gif)

##Video Link


##Sources
