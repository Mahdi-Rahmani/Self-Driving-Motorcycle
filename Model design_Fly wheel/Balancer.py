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

def get_torque(theta, thetaDot, thetaDotDot, fly_wheel_vel):

    # ################################## 1) Dynamic model formula ##################################
    # define parameters
    #thetaDotDot = math.radians(thetaDotDot)
    '''theta = math.radians(theta)

    # body
    m1 = 180
    l1 = 0.48672
    I1 = 4.54
    # fly wheel
    m2 = 100
    l2 = 0.75
    I2 = 4.491
    g = 9.81

    It = m1 * l1**2 + m2 * l2**2 + I1 + I2
    tau_m = (m1*l1 + m2*l2)*g*math.sin(theta) - It * thetaDotDot + I2 * thetaDotDot'''
    #print("torque: , theta, thetaDotDot  ", tau_m, theta, thetaDotDot)

    # ################################## 2) static mode ##################################
    '''global prev_thetaDot_err, sum_thetaDot, prev_theta_err, sum_theta, prev_thetaDotDot_err, sum_thetaDotDot
    theta = math.radians(theta)
    dt = 1/60
    # First PID: thetaDot
    kp1 = 2100
    ki1 = 350
    kd1 = 2800
    thetaDot = -thetaDot
    sum_thetaDot += thetaDot*dt
    tau_m1 = kp1 * thetaDot + ki1*sum_thetaDot + kd1 * (thetaDot - prev_thetaDot_err)
    prev_thetaDot_err = thetaDot

    # Second PID: theta
    kp2 = 2000
    ki2 = 500
    kd2 = 2700
    theta = -theta
    sum_theta += theta*dt
    tau_m2 = kp2 * theta + ki2*sum_theta + kd2 * (theta - prev_theta_err)
    prev_theta_err = theta

    # third PID: thetaDotDot
    kp3 = 0.01
    ki3 = 0
    kd3 = 0.5
    thetaDotDot = -thetaDotDot
    sum_thetaDotDot += thetaDotDot*dt
    tau_m3 = kp3 * thetaDotDot + ki3*sum_thetaDotDot + kd3 * (thetaDotDot - prev_thetaDotDot_err)
    prev_thetaDotDot_err = thetaDotDot

    tau_m = tau_m1 + tau_m2 + tau_m3'''

    # ################################## 3) move on a straight line ##################################
    '''global prev_thetaDot_err, sum_thetaDot, prev_theta_err, sum_theta, prev_fly_wheel_vel_err, sum_fly_wheel_vel
    theta = math.radians(theta)
    dt = 1/60
    # First PID: thetaDot
    kp1 = 50    #800
    ki1 = 0
    kd1 = 3    # 280
    thetaDot = -thetaDot
    sum_thetaDot += thetaDot*dt
    tau_m1 = kp1 * thetaDot + ki1*sum_thetaDot + kd1 * (thetaDot - prev_thetaDot_err)
    prev_thetaDot_err = thetaDot

    # Second PID: theta
    kp2 = 1900
    ki2 = 0
    kd2 = 400
    theta = -theta
    sum_theta += theta*dt
    tau_m2 = kp2 * theta + ki2*sum_theta + kd2 * (theta - prev_theta_err)
    prev_theta_err = theta

    # third PID: fly_wheel_vel
    kp3 = 2
    ki3 = 0
    kd3 = 3.5
    sum_fly_wheel_vel += fly_wheel_vel*dt
    tau_m3 = kp3 * fly_wheel_vel + ki3*sum_fly_wheel_vel + kd3 * (fly_wheel_vel - prev_fly_wheel_vel_err)
    prev_fly_wheel_vel_err = fly_wheel_vel

    tau_m = tau_m1 + tau_m2 + tau_m3'''

    # ################################## 4) obstacle avoidance ##################################
    global prev_thetaDot_err, sum_thetaDot, prev_theta_err, sum_theta, prev_fly_wheel_vel_err, sum_fly_wheel_vel
    theta = math.radians(theta)
    dt = 1/60
    # First PID: thetaDot
    kp1 = 80    #800
    ki1 = 0
    kd1 = 6    # 280
    thetaDot = -thetaDot
    sum_thetaDot += thetaDot*dt
    tau_m1 = kp1 * thetaDot + ki1*sum_thetaDot + kd1 * (thetaDot - prev_thetaDot_err)
    prev_thetaDot_err = thetaDot

    # Second PID: theta
    kp2 = 1900
    ki2 = 0
    kd2 = 400
    theta = -theta
    sum_theta += theta*dt
    tau_m2 = kp2 * theta + ki2*sum_theta + kd2 * (theta - prev_theta_err)
    prev_theta_err = theta

    # third PID: fly_wheel_vel
    kp3 = 2
    ki3 = 0
    kd3 = 3.5
    sum_fly_wheel_vel += fly_wheel_vel*dt
    tau_m3 = kp3 * fly_wheel_vel + ki3*sum_fly_wheel_vel + kd3 * (fly_wheel_vel - prev_fly_wheel_vel_err)
    prev_fly_wheel_vel_err = fly_wheel_vel

    tau_m = tau_m1 + tau_m2 + tau_m3
    return tau_m

def post_step(extension):
    """ Called after the collision detection and after the dynamic solver.
    Use this method to set outputs or get values from dynamics objects.
     
    Parameters
    ----------
    extension : object
        The DynamicsScript extension referring to this script.
     
    """
    global prev_straight_err, prev_obstacle_err, pose

    """if R_velocity < 360:
        R_velocity += 2
    extension.outputs.R_wheel_velocity.value = math.radians(R_velocity)
    """
    theta = roll_finder(extension.inputs.wt.value)
    thetaDot = extension.inputs.av.value[1]
    thetaDotDot = extension.inputs.aa.value[1]
    fly_wheel_vel = extension.inputs.fly_wheel_vel.value

    torque = get_torque(theta,thetaDot, thetaDotDot, fly_wheel_vel)

    #print("torque: , theta, thetaDotDot  ", torque, theta, thetaDotDot)
    #print("theta, thetaDotDot  ", theta, thetaDotDot)
   
    extension.outputs.torque.value = torque

    dist = saveLidarSensorDistanceField(extension.inputs.sensor.value)
    cur_x = extension.inputs.steering_wt.value[0][3]

    rear_wheel_vel = 400

    target_point = [50, 0]
    if dist < 11:
        prev_straight_err = 0
        kp = 2.5
        kd = 1.3
        
        error = 11 - dist
        rear_wheel_vel = (dist/11)*400
        
        pose = (kp*error + kd*(error-prev_obstacle_err))
        prev_obstacle_err = error
        #pose += 0.4

    else:
        max_x = 8
        rear_wheel_vel = ((max_x-abs(cur_x))/max_x)*400
        kp = 4.5
        kd = 2.5

        dist_from_line = cur_x
        pose = -(kp*dist_from_line + kd*(dist_from_line-prev_straight_err))
        prev_straight_err = dist_from_line

    
    extension.outputs.rear_wheel_vel.value = math.radians(rear_wheel_vel)
    extension.outputs.steering_pose.value = math.radians(pose)
    #print('dist,pose, cur_x', dist,pose, cur_x)
    #print(dist)
 