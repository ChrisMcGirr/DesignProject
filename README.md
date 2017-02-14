# DesignProject

Group Members:
Christopher McGirr and
Yulric Sequeira

Design Project Supervisor: Professor Joseph Vybihal

Date: 11/4/2014

The main goal of this semester’s design project was to implement a simple motion detection algorithm that can detection areas of motion and track where that motion is so that the robot may avoid that motion. The challenges that we faced were the speed of the computation of our algorithm.
We attempted to offload the image processing over Bluetooth to another computer, but found the data throughput too small for any real-time application of our algorithm. Then we develop an optimized version of our algorithm in C and minimized the computation time by eliminating the number of pixels the difference filter has to check in the image and the complexity of the motion quadrant detection algorithm.
What we have achieved is a reasonably quick algorithm that can react to movement, albeit with a delayed response. Though the range of our motion detection algorithm far exceeds what the onboard ultrasonic sensors are capable of which is an advantage of our algorithm.
