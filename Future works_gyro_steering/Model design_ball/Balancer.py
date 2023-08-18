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

prev_ball_vel_err = 0
sum_ball_vel = 0

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

def get_torque(theta, thetaDot, thetaDotDot, ball_vel):

    global prev_thetaDot_err, sum_thetaDot, prev_theta_err, sum_theta, prev_thetaDotDot_err, sum_thetaDotDot, prev_ball_vel_err, sum_ball_vel 
    theta = math.radians(theta)
    dt = 1/60
    # First PID: thetaDot
    kp1 = 110    #800
    ki1 = 0
    kd1 = 8   # 280
    sign = 1
    if thetaDot<0:
        sign = -1

    thetaDot = 1.1*thetaDot**2
    sum_thetaDot += thetaDot*dt
    tau_m1 = kp1 * thetaDot + ki1*sum_thetaDot + kd1 * (thetaDot - prev_thetaDot_err)
    prev_thetaDot_err = thetaDot

    # Second PID: theta
    kp2 = 10
    ki2 = 0
    kd2 = 1
    theta = -theta
    sum_theta += theta*dt
    tau_m2 = kp2 * theta + ki2*sum_theta + kd2 * (theta - prev_theta_err)
    prev_theta_err = theta
    tau_m2 += 1.1*math.tan(theta)

    # third PID: fly_wheel_vel
    thetaDotDot = -1.1*thetaDotDot
    #print(thetaDotDot)

    kp3 = 51
    ki3 = 0
    kd3 = 8
    sum_thetaDotDot += thetaDotDot*dt
    tau_m3 = kp3 * thetaDotDot + ki3*sum_thetaDotDot + kd3 * (thetaDotDot - prev_thetaDotDot_err)
    prev_thetaDotDot_err = thetaDotDot

    kp4 = 60
    ki4 = 0
    kd4 = 7
    sum_ball_vel += ball_vel*dt
    tau_m4 = kp4 * ball_vel + ki4*sum_ball_vel + kd4 * (ball_vel - prev_ball_vel_err)
    prev_ball_vel_err = ball_vel

    tau_m = tau_m3-tau_m4 +tau_m1
    return (sign*thetaDot+thetaDotDot)*42.18-((1/(0.175+abs(theta)))**2)+1.1*math.tan(theta)*2 
    #return (sign*thetaDot+thetaDotDot)*49+1.1*math.tan(theta)*4


def post_step(extension):
    """ Called after the collision detection and after the dynamic solver.
    Use this method to set outputs or get values from dynamics objects.
     
    Parameters
    ----------
    extension : object
        The DynamicsScript extension referring to this script.
     
    """
    """if R_velocity < 360:
        R_velocity += 2
    extension.outputs.R_wheel_velocity.value = math.radians(R_velocity)
    """
    theta = roll_finder(extension.inputs.wt.value)
    thetaDot = extension.inputs.av.value[1]
    thetaDotDot = extension.inputs.aa.value[1]
    ball_vel = extension.inputs.ball_vel.value


    force = get_torque(theta,thetaDot, thetaDotDot, ball_vel)

    #print("torque: , theta, thetaDotDot  ", torque, theta, thetaDotDot)
    #print("theta, thetaDotDot  ", theta, thetaDotDot)
   
    extension.outputs.force.value = force

    
    #print('dist,pose, cur_x', dist,pose, cur_x)
    #print(dist)
 