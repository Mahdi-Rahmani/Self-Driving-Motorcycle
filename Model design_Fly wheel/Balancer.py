from configparser import ExtendedInterpolation
import Vortex
import math

prev_err = 0
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

def get_torque(theta, thetaDotDot):

    # define parameters
    thetaDotDot = math.radians(thetaDotDot)
    # body
    m1 = 180
    l1 = 0.557
    I1 = 7.226
    # fly wheel
    m2 = 70
    l2 = 0.901
    I2 = 3.02
    g = 9.81

    It = m1 * l1**2 + m2 * l2**2 + I1 + I2
    tau_m = (m1*l1 + m2*l2)*g*math.sin(theta) - It * thetaDotDot + I2 * thetaDotDot

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

    torque = get_torque(theta, thetaDotDot)

    #print("torque: , theta, thetaDotDot  ", torque, theta, thetaDotDot)
   
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
    print('dist,pose, cur_x', dist,pose, cur_x)
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