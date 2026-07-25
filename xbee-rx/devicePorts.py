import os 

def getXbeePort():
    
    xbeePipe = os.popen('./device.sh 0403:6015')
    xbeePort = xbeePipe.read()
    xbeePipe.close()
    
    return  xbeePort
