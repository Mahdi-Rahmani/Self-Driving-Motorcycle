from Vortex import *

def on_simulation_start(extension):
    extension.lowpassframerate = 0


def pre_step(extension):
    
    freward = float(extension.inputs.Reward.value)
    
    r = max(- freward + 100, 0)/100
    g = max(freward + 100, 0)/100

    extension.outputs.color.value = VxColor(r, g, 0, 1)

    framerate = (1/extension.getApplicationContext().getApplicationUpdateTime())
    extension.lowpassframerate = lowpass_filter(extension, framerate, extension.lowpassframerate, 0.1)
    extension.outputs.framerate.value = str(round(extension.lowpassframerate,1))

def lowpass_filter(extension, input_signal, output_signal, time_constant):
    """Apply a low-pass filter to a signal."""
    delta_time = extension.getApplicationContext().getSimulationTimeStep()
    value = (((delta_time * input_signal) + (time_constant * output_signal))
             / (delta_time + time_constant))
    return value