[EPFL] Semester Project

Github repository for the embedded code of the STM32 for the STI INTERDISCIPLINARY ROBOT COMPETITION 2019 at EPFL.

Team 5 : Green Eye, Winner of 2019 with 250 points.

Position control with maximum speed customizable and smooth deceleration.

Multiple control algorithms are available to the user:
  * Position controller should be used for unknown terrain but well-known goal and robotposition. The robot will try to reach the goal (specified in an absolute coordinate system bytwo coordinates X and Y)
  
  * Speed controller should be used mainly to re-calibrate the robot to a reference point whenthe distance or angle is not known perfectly. We mostly used that controller to re-calibratethe robot against wall or to do long distance when we knew there was no obstacle. To avoid excessive drift against the reference goal. The robot should first move with sensors enabled. It will then stop when sensing an obstacle forward (or Backward if Speed <0). Then the user should disable the obstacle avoidance and move slowly for a short amount of time. The robot will then align itself to the reference point and the user can specify the position of therobot.
  
  * PWM Mode is mostly for test purposes or brute force behaviour.  If you encounter a casewhere the robot needs to push something, and the speed controller does not behave the wayit should you can then use that mode. It is not a control algorithm, it is mostly about setting a known amount of power in the motors for a known amount of time.
 
Two functions are running at constant frequencies of 100Hz and 1000Hz on the micro-controller. 
  * The fastest function is calculating the robot position by looking at the delta between the previous and the actual encoder value. 
By comparing the right and left delta encoder we can estimate the distance and rotation done by the robot. Then by approximating curves by multiple small segments we can estimate the robot position in the field. 
  * The slowest function is calculating the speed of the robot (By doing that calculation at the rate of a 100Hz and not 1000Hz we low-pass the information thus avoiding jerk from the robot). It is also calculating the distance and angle error to reach a coordinate and feed the motors according to these errors. The two levels of control (Position/Angle and speed) allow the robot to move to a point with a predetermined speed. We can then ask the robot to do long distance fast and small distance slowly to be accurate

The STM32 use its DMA units to process the 13 IR sensors through the ADC which is continuously converted.
Another DMA unit is capturing data from the UARTs port in a circular buffer, and send data stored in another buffer. 
Also, two timers are setup as quadrature encoder capture, thus capturing ticks from the encoder without using the CPU. 
Then a timers is setup to generate PWM at a frequency of 20kHz.

|    Command                        |    Description                                                                                                                                                                                                                                                                                                                                                                                                                            |   |
|-----------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---|
|    G0 X... Y... A... T... R...    |    Specify   goal coordinate (X and Y), T is the timeout, when reached the robot doesn’t   control the position anymore. R specify if the robot can move backward or   not. *1   Need the M3 H3 command to start the   control.                                                                                                                                                                                                           |   |
|    G1 R... L... T...              |    Specify   goal speed. R for the right wheel, L for the Left wheel. If unspecified it   will reuse the value in memory (0 by default) T is the timeout, when reached   the robot doesn’t control the speed anymore. Need the M3 H2 command to start   the control.                                                                                                                                                                      |   |
|    G2 R... L... T...              |    Specify PWM value and   direction with the sign (-2100/2100 is the maximum value accepted). R for the   right wheel, L for the Left wheel. If unspecified it will reuse the value in   memory (0 by default).   T is the timeout, when reached the robot   set the PWM to 0.   Need the M3 H1 command to start the   control.                                                                                                          |   |
|    G92 X... Y... A...             |    Set the robot position, erase the previously calculated position.   Useful for recalibration or external measurement system. X and Y in mm for   the position, A for the angle in degree.                                                                                                                                                                                                                                              |   |
|    M3 H... S...                   |    Command to activate a controller, H3 for a   position control, H2 for a speed control, H1 to set the PWM. This will use   the parameters set previously with the G0/G1/G2 commands.   S:   Parameters to disable sensors, it should be followed by the decimal value of the   sensors enabled. (15 will enable all sensors, 0 will disabled all of them).   More information about that value can be   found in the avoidance part.    |   |
|    M15 S...                       |    Ask the sensors value from the ADC conversion (12Bits   value). 1 for the back sensors, 2 for the left sensors, 4 for the right   sensors and 8 for the front sensors. You can ask for two, or more, groups of   sensors by adding the values (so 15 will return the 4 lines with every   sensors values)                                                                                                                              |   |
|    M92 D... A...                  |    D parameter: Set the value of ticks of   encoder per mm   A parameter: set the spacing, in mm,   between the encoders wheel.   These values should be changed only if   encoder wheels are changed and the robot isn’t calculating the correct   distance/ angle when moving.                                                                                                                                                          |   |
|    M112                           |    Emergency stop                                                                                                                                                                                                                                                                                                                                                                                                                         |   |
|    M135 S...                      |    Specify the frequency of the control   loop, 100Hz by default.                                                                                                                                                                                                                                                                                                                                                                         |   |
|    M201 H... S... A... B...       |    Set   the Maximum speed, Acceleration and braking values for the position control   algorithm *2   H0 will set these values for the Distance   control algorithm H1 will set these values for the Angle control algorithm                                                                                                                                                                                                              |   |
|    M202 D...                      |    Specify the size of the “Final Bool”.   This is tolerance size for when the robot achieved the goal specified.                                                                                                                                                                                                                                                                                                                         |   |
|    M301 H... P... I... D...       |    P, I , D are the parameters for the PID   control loop.   H2 will set these for the Distance   control loop,   H1 will set these for the Angle control   loop,   H0 will set these for the Speed control   loop,                                                                                                                                                                                                                       |   |

The robot can answer to every command but most of the answers has been disabled to do not overload the communication. M0 answer are usually the expected answer the user must look at "M0 X... Y... A... T... S..." or M0 H2 T..."
In that answer the parameters :
  * X, Y, A provide the coordinate of the robot in mm and degrees
  * T equal to 0 (Control position only) means that the robot reached the goal
    T equal to 1 means that the controller has timeout
    T equal to 2 means that the robot detect and obstacle and is blocked.
  * S provide the sensor information, each sensor is represented by a bit, 1 for triggered, in the order MLKJIHFEDCBA

*1 : Parameter A is not implemented, backward movement can still happen if the distance to the goal is <200mm or if the robot cannot turn around.

*2 : Acceleration and braking hasn’t been implemented

![alt text](https://github.com/Guilhem74/STI_Robotic_Competition_Electronics/blob/master/Pictures/Control_Schematic.png?raw=true)
![alt text](https://github.com/Guilhem74/STI_Robotic_Competition_Electronics/blob/master/Pictures/CubeMX.png?raw=true)

 
Mechanics Real Life pictures, more information available at https://github.com/Guilhem74/STI_Robotic_Competition_Mechanics/

![alt text](https://github.com/Guilhem74/STI_Robotic_Competition_Mechanics/blob/master/Pictures/Back_Render.png?raw=true)
![alt text](https://github.com/Guilhem74/STI_Robotic_Competition_Mechanics/blob/master/Pictures/Bottom_Render.png?raw=true)
![alt text](https://github.com/Guilhem74/STI_Robotic_Competition_Mechanics/blob/master/Pictures/Front_Render.png?raw=true)
