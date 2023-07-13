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

pose = 0
#R_velocity = 0

def on_simulation_start(extension):
    """
    # Creating an input of type boolean
    extension.createInput("boolean input", Vortex.Types.Type_Bool)
    # Creating an output of type double
    extension.createOutput("an output double", Vortex.Types.Type_Double)
    # Creating a parameter of type double
    extension.createParameter("number", Vortex.Types.Type_Double)
    """ 
    #extension.sensor = LidarSensor()
    pass
    

 
def on_simulation_stop(extension):
    """ Called when the application mode changes from simulating to editing.
    Use this method to define specific actions that must be taken at the end of the simulation.
     
    Parameters
    ----------
    extension : object
        The DynamicsScript extension referring to this script.
     
    """
    pass
 
def pre_step(extension):
    """ Called before the collision detection and before the dynamic solver.
    Use this method to get inputs or set values to dynamics objects.
     
    Parameters
    ----------
    extension : object
        The DynamicsScript extension referring to this script.
     
    """
    #extension.outputs.pos.value = extension.outputs.pos.value - 0.1
    
    pass
 
def paused_update(extension):
    """ Called at every update in editing mode and when the simulation is paused.
     
    Parameters
    ----------
    extension : object
        The DynamicsScript extension referring to this script.
     
    """
     
    pass
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

    # Dynamic model formula
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
    global prev_thetaDot_err, sum_thetaDot, prev_theta_err, sum_theta, prev_fly_wheel_vel_err, sum_fly_wheel_vel
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

    # third PID: fly_wheel_vel
    kp3 = 20
    ki3 = 0
    kd3 = 8
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
    global prev_err, pose, R_velocity

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

    if dist < 11:
        if pose < 30:
            pose += 2

    else:
        kp = 8
        kd = 2

        dist_from_line = cur_x
        pose = -(kp*dist_from_line + kd*(dist_from_line-prev_err))
        prev_err = dist_from_line

    extension.outputs.steering_pose.value = math.radians(pose)
    #print('dist,pose, cur_x', dist,pose, cur_x)
    #print(dist)
 
def on_keyframe_save(extension, data):
    """ Called after a keyframe has been taken.
    Use the data parameter to store values that will be provided back in the on_keyframe_restore
    These values can be anything used in that script that you want restore to this point in time.
    The extension itself will be properly restored and thus it's data do not need to be saved manually.
     
    Parameters
    ----------
    extension : object
        The DynamicsScript extension referring to this script.
     
    data : dictionnary
        Store values that will be provided back in the on_keyframe_restore
         
        The following types are supported: booleans, integers, long integers, floating point
        numbers, complex numbers, strings, Unicode objects, tuples, lists, sets, frozen sets,
        dictionaries, and code objects.
         
        Tuples, lists, sets, frozen sets and dictionaries are only supported as long as the values contained therein are
        themselves supported; and recursive lists, sets and dictionaries should not be written (they will cause infinite loops).
        The singletons None, Ellipsis and StopIteration can also be saved and restored.
         
    """
    pass
     
def on_keyframe_restore(extension, data):
    """ Called after a keyframe has been fully restored.
     
    Use the data parameter to restore the script to the point where the data was captured.
     
    Parameters
    ----------
    extension : object
        The DynamicsScript extension referring to this script.
     
    data : dictionnary
        Stored values capture during the last call to on_keyframe_save
    """
    pass