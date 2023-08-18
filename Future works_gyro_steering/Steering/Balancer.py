from configparser import ExtendedInterpolation
import Vortex
import math

prev_err = 0

prev_thetaDot_err = 0
sum_thetaDot = 0

prev_theta_err = 0
sum_theta = 0

prev_thetaDotDot_err = 0
sum_thetaDotDot = 0

prev_fly_wheel_vel_err = 0
sum_fly_wheel_vel = 0

prev_straight_err = 0
prev_obstacle_err = 0
pose = 0
#R_velocity = 0

def saveLidarSensorDistanceField(inputLidarSensor):
    # Here, inputLidarSensor is an input extension field to which the depth camera is connected
    distanceFieldVectorFloat = inputLidarSensor.getOutput("Distance field").toVectorFloat()
 
    width = int(inputLidarSensor.getParameter("Horizontal step resolution").value)
    height = int(inputLidarSensor.getParameter("Number of channels").value)
    maxRange = inputLidarSensor.getParameter("Range").value
 
    # Convert distance field into row-major float values normalized in the [0, 255] range for Pillow.
    if distanceFieldVectorFloat:
        return min(distanceFieldVectorFloat)
    else:
        return math.inf

def roll_finder(T):
    #pitch = math.atan2(T[2][1],T[2][2])
    roll = math.atan2(-T[2][0],math.sqrt(T[2][1]**2 + T[2][2]**2))
    #yaw = math.atan2(T[1][0],T[0][0])
    roll = math.degrees(roll)
    return roll

def yaw_finder(T):
    #pitch = math.atan2(T[2][1],T[2][2])
    #roll = math.atan2(-T[2][0],math.sqrt(T[2][1]**2 + T[2][2]**2))
    yaw = math.atan2(T[2][0],T[1][0])
    yaw = math.degrees(yaw)
    return yaw

def get_torque(theta, thetaDot, thetaDotDot):

    global prev_thetaDot_err, sum_thetaDot, prev_theta_err, sum_theta, prev_fly_wheel_vel_err, sum_fly_wheel_vel
    theta = math.radians(theta)
    dt = 1/60
    # First PID: thetaDot
    kp1 = 42 #800
    ki1 = 0.005
    kd1 = 4.5    # 280
    thetaDot = -thetaDot
    sum_thetaDot += thetaDot*dt
    tau_m1 = kp1 * thetaDot + ki1*sum_thetaDot + kd1 * (thetaDot - prev_thetaDot_err)
    prev_thetaDot_err = thetaDot

    # Second PID: theta
    kp2 = 15
    ki2 = 0.005
    kd2 = 1
    theta = -theta
    sum_theta += theta*dt
    tau_m2 = kp2 * theta + ki2*sum_theta + kd2 * (theta - prev_theta_err)
    prev_theta_err = theta

    ''' # third PID: fly_wheel_vel
    kp3 = 2
    ki3 = 0
    kd3 = 3.5
    sum_fly_wheel_vel += fly_wheel_vel*dt
    tau_m3 = kp3 * fly_wheel_vel + ki3*sum_fly_wheel_vel + kd3 * (fly_wheel_vel - prev_fly_wheel_vel_err)
    prev_fly_wheel_vel_err = fly_wheel_vel
    '''
    tau_m = tau_m1+tau_m2
    return tau_m

def post_step(extension):
    theta = roll_finder(extension.inputs.wt.value)
    thetaDot = extension.inputs.av.value[1]
    thetaDotDot = extension.inputs.aa.value[1]

    angle = get_torque(theta,thetaDot, thetaDotDot)
    #print(angle)
    extension.outputs.steer_ang.value = -math.radians(angle)

 