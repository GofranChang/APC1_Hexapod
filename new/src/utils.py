import time

def getCurrentTimeMs():
    return round(time.time() * 1000)

def map(x, inMin, inMax, outMin, outMax):
    return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin