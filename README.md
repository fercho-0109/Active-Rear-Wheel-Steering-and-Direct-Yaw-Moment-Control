# Active-Rear-Wheel-Steering-and-Direct-Yaw-Moment-Control

Code developed for "A. Marino, - Receding Horizon Tracking Trajectory Strategy for Feedback Linearized Differential-Drive".  
Master's student "Calabria University".  
For any questions or suggestions write to alexismarino0109@gmail.com  

# Sumary.

In this project, an integrated control system design for active rear-wheel steering and direct yaw moment control is presented. The proposed control system is a model matching controller (MMC) which makes the vehicle follow the desired dynamic model by using the state feedback of both yaw rate and side slip angle. Although this integrated MMC is designed by linear control theory, it can greatly improve the handling and stability and show the expected robustness when the vehicle parameter changes, such as the vehicle mass, even in large lateral acceleration ranges. The steps developed in this project are:  
• Obtain the Model Matching Controller and implement it in the Simulink environment.  
• Implemented optimal controls like H_inf, H_2, L_1, LQ, and robust LPV controller.  
• Comparison of optimal control techniques.  
• Integrate an integral effect to the model and implement the optimal controllers.  
• Used the Polytropic model description to represent the uncertain system when the plant is subject to velocity changes.  
• Implement robust optimal controls.
• Finally, varying the velocity to observe the robustness of the controls and implement a gain scheduling control.  

  
# Prerequisites
- The code was created and tested on the Matlab/Simulink 2023a environment

# File description
The repository contains three files
1. **Electric_Bicycle**: This Matlab file contains the configuration parameters of the program and shows the results of the analysis.
2. **Electric_Bike**: This Simulink file contains the complete simulation of the e-bike system and the implementation of the observer and control.
3. **Report**: This contains the complete explanation, the mathematical formulations, and the control configuration.


# Example to run the experiment  
**"e-BIKE"**
### Matlab/Simulink simulation 
1. Download the files. 
2. Run the Matlab file "**Electric_Bicycle**".
3. Open and run the Simulink file "**Electric_Bike**"
4. The Scope blocks should start to show the results  
![image](https://github.com/fercho-0109/Dynamic-system-analysis-of-a-e-Bike-/assets/40362695/38b221e3-3071-4afd-bd2a-a567903d0a51)  
fig1. In blue is shown the graph of the free response without feedback. On the other hand, the orange graph is the response after feedback control which shows a noticeable reduction of the overshoot that is generated in the beginning


