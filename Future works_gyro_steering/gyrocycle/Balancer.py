from configparser import ExtendedInterpolation
import Vortex
import math
pris1 = 0
pris2 = 0
counter = 0

def post_step(extension):
    global pris1, pris2, counter
    counter += 1
    if counter>400:
        if pris1<0.2:
            pris1 += 0.01
            pris2 += 0.01
        extension.outputs.pris1.value = pris1
        extension.outputs.pris2.value = pris2


 