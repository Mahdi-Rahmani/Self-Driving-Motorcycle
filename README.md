# Self balancing motorcycle
<p align="center">
  <a href="https://github.com/Mahdi-Rahmani/Self-driving-Motorcycle-/blob/main/Results/Motorcycle2.png">
    <img src="https://github.com/Mahdi-Rahmani/Self-driving-Motorcycle-/blob/main/Results/Motorcycle2.png" alt="Mech-AUT Motorcycle">
  </a>
</p>

- **By**: Mahdi Rahmani  

- **Supervisor**: Dr. Ali Azimi

------------------
n this project, we aim to simulate a self-balancing motorcycle. To achieve this, we have several solutions at our disposal. First, we need to select the appropriate balancer mass and then choose a suitable controller to maintain the motorcycle's balance.

As you may be aware, experimental work can incur significant costs, such as being time-consuming and requiring financial resources, since our motorcycle may fall to the ground and get damaged during testing. Therefore, we utilize Vortex Studio, which helps us simulate our environment and the vehicle we are working on.

I will start by providing a brief introduction to the software, and then you can review the results.

## Vortex Studio
Vortex Studio is a platform for creating and deploying real-time, interactive simulations of mechanical systems. Vortex Studio is a key component of all of CM Labs’ industry-leading equipment simulators and it is available for you to design, build and develop your own real-time, interactive simulation products.

**Why use Vortex Studio?**
- Build and deploy real-time, interactive simulations of mechanisms, vehicles, and vessels.
- Validate design through virtual prototyping.
- Solve training needs for complex equipment with a high-fidelity experience.

In our project, we use this software in two modes:

1) When we want to design a PID controller, we can use Python code. Our Python code can interact with the simulator in real-time, receive feedback from sensors, and then apply an appropriate reaction to the actuator.

2) For reinforcement learning, we require an environment that we can run multiple times. This can be achieved by creating an application from our modeled environment and motorcycle with the assistance of Vortex Studio's features.

You can buy a license and download this software from the [CM-Labs website.](https://www.cm-labs.com/en/vortex-studio/)
## Results
### No controller
First of all, we should check what our motorcycle's behavior is if we don't use any controller. You can see the results in the GIF below.

<p align="center">
  <a href="https://github.com/Mahdi-Rahmani/Self-driving-Motorcycle-/blob/main/Results/No_controller.gif">
    <img src="https://github.com/Mahdi-Rahmani/Self-driving-Motorcycle-/blob/main/Results/No_controller.gif" alt="no_control" width=600 height=400>
  </a>
</p>

### PID and Reaction wheel
We use a flywheel as a reaction wheel and a PID controller to calculate the desired torque for the motorcycle to maintain its balance. As you can observe in the GIF below, our motorcycle successfully navigates all obstacles.

<p align="center">
  <a href="https://github.com/Mahdi-Rahmani/Self-driving-Motorcycle-/blob/main/Results/test_pid.gif">
    <img src="https://github.com/Mahdi-Rahmani/Self-driving-Motorcycle-/blob/main/Results/test_pid.gif" alt="fly_wheel_pid" width=600 height=400>
  </a>
</p>

### RL and Reaction wheel
We use a flywheel as a reaction wheel and reinforcement learning (RL) as our controller to calculate the desired torque for the motorcycle to maintain its balance. In this phase, we need to train our model first. The training process is visible in the GIF below.

<p align="center">
  <a href="https://github.com/Mahdi-Rahmani/Self-driving-Motorcycle-/blob/main/Results/train_RL.gif">
    <img src="https://github.com/Mahdi-Rahmani/Self-driving-Motorcycle-/blob/main/Results/train_RL.gif" alt="fly_wheel_RL_train" width=600 height=400>
  </a>
</p>

You can also view the training plot in the image below.

<p align="center">
  <a href="https://github.com/Mahdi-Rahmani/Self-driving-Motorcycle-/blob/main/Results/RL_Train_plot.png">
    <img src="https://github.com/Mahdi-Rahmani/Self-driving-Motorcycle-/blob/main/Results/RL_Train_plot.png" alt="fly_wheel_RL_train" width=600 height=400>
  </a>
</p>

After training, we can test our model. As demonstrated in the GIF below, our motorcycle successfully navigates all obstacles.

<p align="center">
  <a href="https://github.com/Mahdi-Rahmani/Self-driving-Motorcycle-/blob/main/Results/test_RL1.gif">
    <img src="https://github.com/Mahdi-Rahmani/Self-driving-Motorcycle-/blob/main/Results/test_RL1.gif" alt="fly_wheel_RL_test" width=600 height=400>
  </a>
</p>

### Other methods like using gyroscope
While we have explored different methods for controlling the motorcycle's balance, using a flywheel has proven to be the most effective. You can find details about the other methods we have investigated in the ["Future Works_Gyro_Steering"](https://github.com/Mahdi-Rahmani/Self-driving-Motorcycle-/tree/main/Future%20works_gyro_steering) directory. 

For example, by utilizing a gyroscope, we achieved the results shown below:

<p align="center">
  <a href="https://github.com/Mahdi-Rahmani/Self-driving-Motorcycle-/blob/main/Results/gyro.gif">
    <img src="https://github.com/Mahdi-Rahmani/Self-driving-Motorcycle-/blob/main/Results/gyro.gif" alt="gyro" width=600 height=400>
  </a>
</p>

## Discussion
These three methods each perform better in specific simulation situations. We collect data from the simulations to verify this, and additional explanations are provided in my [final report.](https://github.com/Mahdi-Rahmani/Self-driving-Motorcycle-/blob/main/Final%20Report/Final_Report_Rahmani.pdf)

My presentation file can also be accessed at this [link].(https://docs.google.com/presentation/d/1-7Z2EOImf09F18AmHH1wWCepT36cc-Xw/edit?usp=sharing&ouid=112561970420312111928&rtpof=true&sd=true)