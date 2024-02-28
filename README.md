# Active-Rear-Wheel-Steering-and-Direct-Yaw-Moment-Control

Code developed for "A. Marino, - Receding Horizon Tracking Trajectory Strategy for Feedback Linearized Differential-Drive".  
Master's student "Calabria University".  
For any questions or suggestions write to alexismarino0109@gmail.com  

# Sumary.

In this project, an integrated control system design for active rear-wheel steering and direct yaw moment control is presented. The proposed control system is a model matching controller (MMC) which makes the vehicle follow the desired dynamic model by using the state feedback of both yaw rate and side slip angle. Although this integrated MMC is designed by linear control theory, it can greatly improve the handling and stability and show the expected robustness when the vehicle parameter changes, such as the vehicle mass, even in large lateral acceleration ranges. The steps developed in this project are:  
- Obtain the Model Matching Controller and implement it in the Simulink environment.  
- Implemented optimal controls like H_inf, H_2, L_1, LQ, and robust LPV controller, using LMI formulations.  
- Comparison of optimal control techniques.  
- Integrate an integral effect to the model and implement the optimal controllers.  
- Used the Polytropic model description to represent the uncertain system when the plant is subject to velocity changes.  
- Implement robust optimal controls.
- Finally, varying the velocity to observe the robustness of the controls and implement a gain scheduling control.  

The front steering angle corresponds to a disturbance signal. Two different tests were considered **"curve test", "Moose test."**  
![image](https://github.com/fercho-0109/Active-Rear-Wheel-Steering-and-Direct-Yaw-Moment-Control/assets/40362695/c746b6ac-bf86-4d74-ae94-ff23a112aaca)

  
# Prerequisites
- The code was created and tested on the Matlab/Simulink 2023a environment
- Yalmip solver is necessary, use **install_mpt3** file to install 

# File description
The repository contains three files
1. **Matlab 2023**: It contains the main programs and functions to run the program.
2. **install_mpt3**: it is the file used to install Yalmip solver
3. **Report**: This contains the complete explanation, the mathematical formulations, and the control configuration.


# Example to run the experiment  
**"Rear Wheel Steering and Direct Yaw Moment Control"**
### Matlab/Simulink simulation 
1. Download the files or clone the repository. 
2. Open the Matlab file "**Model**".
3. Choose the optimal control that want to implement with 1
![image](https://github.com/fercho-0109/Active-Rear-Wheel-Steering-and-Direct-Yaw-Moment-Control/assets/40362695/b15c5b4d-724d-4345-8670-a3d402a0b2f7)
4. Run the program
5. Open and run the Simulink model "primera_parte"
6. The Scope blocks should start to show the results in the base of the chosen control.
![image](https://github.com/fercho-0109/Active-Rear-Wheel-Steering-and-Direct-Yaw-Moment-Control/assets/40362695/717dc4a6-6633-4144-b919-b8072c482544)
  
7. To see the comparison between controls choose comparison in the "**Model**" and then run the simulink file called "**comparison**".    
![image](https://github.com/fercho-0109/Active-Rear-Wheel-Steering-and-Direct-Yaw-Moment-Control/assets/40362695/ab467eaf-421b-4ff5-8b72-ab4a7575e88a)

8. To see the robust controls choose Robust_control_H_inf in the "**Model**" and then run the simulink file called "**robust**".    

9. To see the controls with integral effect choose integral_control_H2_Hinf_L1_LQ in the "**Model**" and then run the simulink file called "**integral_1_pole**".
10. finally to see the LPV controller run the simulink file called "**LPV_control**".


