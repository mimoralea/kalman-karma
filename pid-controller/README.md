Line Follower
=============

# Algorithm used
For the line following robot, we used a full PID controller. A PID controller is a common technique used to control robot motion. 
A PID controller is composed of a proportional, integral and derivative components to calculate an error value as the difference between the measured data points and the target value. In this particular mission, we used the PID controller to keep track of the error between the edge of the red tape and the actual position the color sensor in the robot was measuring. Also, the integral component of the controller, would keep track of the error through time. This, as we can see on the video, would allow the robot to increase the speed on which it would come out of the middle of the line, if it kept reading higher values of red than the desired. Finally, the D component of the controller, would try to predict the next error value and attempt to compensate the error correction so the robot would not overshoot the correction to far off the edge.

# Remarks
The most particular challenge from a PID controller was to find the right constant values to actually allow the robot, with its particular mechanical design and the specific track features, perform well following the line. The PID controller can be tricky to customized since adjusting a constant might influence on not only one but all factors of following the line. The proportion to which the robot tries to correct its errors, the magnitude to which the robot increases the proportion that is using to fix the error, and the accuracy in which the robot predicts its next error based on previous one, all three components are tightly coupled and work together in the line follower robot.

# Video
Final video run is located at http://youtu.be/Sn_-JHKXU-Y

# References
http://www.inpharmix.com/jps/PID_Controller_For_Lego_Mindstorms_Robots.html
