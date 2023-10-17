Explanation taken from http://ai2001.ifdef.jp/uvc/code_Eng.html and https://ai2001.ifdef.jp/uvc/code2_Eng.html:

Explain the operation.

The following operations can be performed by key input.

w:　Start walking

u:　Specify UVC (upper body vertical control) enable / disable as an alternative.
　　When the body is blue, UVC is enabled,and when it is red, UVC is disabled.

r:　Reset (return to the initial position)

q:　Finished 

j: Ｆ←

k: Ｆ→

p: Ｆ↑

l: Ｆ↓

1:　Experiment of applying external force
Set the experimental environment to push the robot from behind.

2:　Experiment to start walking naturally
Tilt the robot's upper body forward and start walking naturally.

3:　Experiment to walk on a slope (provisional specification)
The integrated value of the tilt angle of the robot is superimposed on the upper body angle to enable walking on slopes.

4:　Experiment to traverse steps (provisional specifications)
If the ground contact of the swing leg cannot be detected, the knee of the support leg is bent.

5:　Apply all algorithms (provisional specifications)

※3-4 is a provisional specification, and there is still room for improvement. 