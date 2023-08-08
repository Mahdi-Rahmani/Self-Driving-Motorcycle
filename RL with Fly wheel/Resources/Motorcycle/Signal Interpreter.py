import math
import Vortex

def roll_finder(T):
    #pitch = math.atan2(T[2][1],T[2][2])
    roll = math.atan2(-T[2][0],math.sqrt(T[2][1]**2 + T[2][2]**2))
    #yaw = math.atan2(T[1][0],T[0][0])
    #roll = math.degrees(roll)
    return roll

def post_step(extension):

    theta = roll_finder(extension.inputs.wt.value)
    thetaDot = extension.inputs.av.value[1]
    thetaDotDot = extension.inputs.aa.value[1]
    #print("rot", theta)
    extension.outputs.theta.value = theta
    extension.outputs.thetaDot.value = thetaDot
    extension.outputs.thetaDotDot.value = thetaDotDot