Designing a context-aware adaptation strategy for the development of an autonomous braking system for Level 3 autonomous cars involves considering two distinct driving conditions: a rainy day or high cognitive workload (HCW) and a normal day or low cognitive workload (LCW). The autonomous braking control should have a braking limit based on road conditions (rainy or normal), which is lower than the brake limit of the human driver. This means that humans have the capability to apply greater braking force than the autonomous system.<br/>

<img width="523" alt="Screenshot 2024-01-10 000421" src="https://github.com/vharsoor/MATLAB-Projects/assets/70684031/d78cabf3-5fc8-4e97-a820-42fb44139f1b"><br/>

Given: Model of Vehicle kinematics and Braking control of the car.<br/>

The advisory control's primary task is to predict whether the autonomous braking control can stop the car before a collision. If the autonomous braking system cannot achieve this, the advisory control needs to assess whether the human driver, considering their brake limit and reaction time, can stop the car. If the human can do so, the system should switch to human control; otherwise, it should refrain from switching.<br/>

To implement this strategy, several key elements need to be considered:<br/>
1.	**Road Conditions (LCW or HCW)**: Randomly generate road conditions for each time step, distinguishing between low cognitive workload (LCW) on a normal day and high cognitive workload (HCW) on a rainy day.<br/>
2.	**Human Physiology (Heart Rate and Respiratory)**: Generate heart rate and respiratory values within the user's specified range to calculate human reaction time. This physiological data is crucial for determining how quickly a human driver can react to a potential collision.<br/>
3.	**Advisory Control Implementation**: Develop an advisory control system that considers the autonomous braking control's effectiveness and the human driver's capability. If the autonomous system cannot prevent a collision, assess the human driver's ability to stop the car within a safe timeframe. If the human can do so, switch to human control; otherwise, maintain autonomous control.<br/>

These considerations should be applied across a range of initial velocities during experimentation. It is important to note that beyond a certain initial velocity, increasing braking force (adjusting gain) may become ineffective in preventing a collision. Therefore, the advisory control strategy must be robust enough to make decisions based on the limitations of both the autonomous system and the human driver.<br/>

Full explanation of project available in "Complete README.pdf".<br/>


