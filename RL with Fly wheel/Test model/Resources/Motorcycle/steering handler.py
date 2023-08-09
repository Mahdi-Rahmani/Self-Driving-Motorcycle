from configparser import ExtendedInterpolation
import Vortex
import math

prev_err = 0


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


def post_step(extension):
    """ Called after the collision detection and after the dynamic solver.
    Use this method to set outputs or get values from dynamics objects.
     
    Parameters
    ----------
    extension : object
        The DynamicsScript extension referring to this script.
     
    """
    global prev_straight_err, prev_obstacle_err, pose

    dist = saveLidarSensorDistanceField(extension.inputs.sensor.value)
    cur_x = extension.inputs.steering_wt.value[0][3]

    rear_wheel_vel = 400

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
        kp = 6
        kd = 3

        dist_from_line = cur_x
        pose = -(kp*dist_from_line + kd*(dist_from_line-prev_straight_err))
        prev_straight_err = dist_from_line

    
    extension.outputs.rear_wheel_vel.value = math.radians(rear_wheel_vel)
    extension.outputs.steering_pose.value = math.radians(pose)
