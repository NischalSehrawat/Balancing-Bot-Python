# Balancing-Robot-RaspberryPi
This is my project for balancing an inverted pendulum using an Arduino Mega and Raspberry Pi. In my [earlier ](https://www.youtube.com/watch?v=-TRXWSr9_dE&list=PLveRMPt4kAsA41ivMscrzFSWWx54CI52J&t=0s&index=2) approach I was successful in making the robot balance but in order to move it like a **Segway**, it needs more thorough analysis. Therefore I decided to go into greater details and study it more thoroughly. The main aim of the project is to learn about the following topics
- Gain an insight into the physics of the problem and understand the equation of motions. I have used [this paper](https://content.sciendo.com/view/journals/meceng/61/2/article-p331.xml) and followed [this course](https://www.coursera.org/learn/mobile-robot). 
- Learn about classic and modern control theories. [Source](https://www.youtube.com/watch?v=Pi7l8mMjYVE&list=PLMrJAkhIeNNR20Mz-VpzgfQs5zrYi085m)
- Learn about concepts like controllability, observability, LQR technique to derive controller gain. [Source](https://www.youtube.com/watch?v=Pi7l8mMjYVE&list=PLMrJAkhIeNNR20Mz-VpzgfQs5zrYi085m)
- Learn and code my own Kalman Filter for Sensor Fusion. I struggled a lot to find sources that explained it from the very basics and finally found some good reads. Sources [1](https://home.wlu.edu/~levys/kalman_tutorial/), [2](http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it/) and [3](https://github.com/balzer82/Kalman/blob/master/Kalman-Filter-CV.ipynb?create=1).
- Check how do Kalman and Complimentary filters compare with each other.
- Learn how to implement the concepts learnt on physical hardware and understand the problems encountered.
- Learn how to make RaspberryPi communicate with arduino over Serial port. [Source](http://forum.arduino.cc/index.php?topic=396450)
- Learn how to use rotory encoders to get RPM of a motor and smoothen the data. Info about using encoders can be found [here](https://www.youtube.com/watch?v=oLBYHbLO8W0). For smoothing RPM data I have used exponential averaging.
- Info about dc motor control (using 2 wires instead of 3 for speed control) can be found [here](https://www.bluetin.io/python/gpio-pwm-raspberry-pi-h-bridge-dc-motor-control/).  However for robot control it is easier to use **Robot** builtin class on gpiozero library on RasPi. Info about using various builtin classes on RaspPi can be found [here](https://projects.raspberrypi.org/en/projects/physical-computing/16). Info about using MPU6050 library on a RaspberryPi can be found [here](https://libraries.io/pypi/mpu6050-raspberrypi).
- An important observation in this project is that while the Raspberry Pi is significantly faster than an Arduino, but since it runs an OS, it cannot be used for realtime control that requires precise timing. For example,when the potentiometer data was sent over Serial port to the Pi from Arduino, the Pi responded very late and to compensate that, the data was sent to the Pi at 20 mS interval instead of 1-2 ms. This will slow down the control loop considerably. Therefore, The main balancing task will be performed on the Arduino board, while the Pi will be used for higher level tasks such as decision making.

# Implementation on Arduino: Problems, findings and solutions.

A lot of problems were encountered and solved during the implementation on a microcontroller. Problems / findings are listed below in decreasing order of importance.

- The main control loop must be fast. I found that the robot doesn't work at all if the loop time is more than 40 ms. In my current implementation, I have kept it at 5 ms. The loop time effects a lot of parameters such as the complimentary filter weighting parameter alpha. 
- Serial baudrate must be higher than 115200. It won't work at all if baudrate is set at 9600;
- Make sure that the IMU6050 is correctly caliberated and when vertical, the angles donot vary more than +/- 0.5 deg.
