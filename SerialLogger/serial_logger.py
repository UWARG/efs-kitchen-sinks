# Serial Logger, heavily inspired by boseji's serial logger

# usage:
# python serial_logger.py

import serial

import logging, time
from signal import signal, SIGINT
from sys import exit
from queue import Queue, Empty

# com port - change as needed
PORT = "COM14"

# baud rate - change as needed
# BAUD = 460800  # 460800 default baud for gemini
BAUD = 115200 # nice uart baud rate :D

# filename - change as desired
LOGFILENAME = "serial_log.txt"

q = Queue(2)

def setupLogger(filename):
    logger = logging.getLogger('Serial Logger')
    logging.basicConfig(filemode = "w")
    logger.setLevel(logging.NOTSET)
    # logger.setLevel(logging.DEBUG)
    # logger.setLevel(logging.WARNING)

    # create file handler
    fh = logging.FileHandler(filename)
    fh.setLevel(logging.NOTSET)

    # create console handler
    ch = logging.StreamHandler()
    ch.setLevel(logging.NOTSET)

    # create formatter, add to handlers
    formatter = logging.Formatter("%(asctime)s;%(levelname)s;%(message)s","%Y-%m-%d %H:%M:%S")
    fh.setFormatter(formatter)
    ch.setFormatter(formatter)

    # add handlers to the logger
    logger.addHandler(fh)
    logger.addHandler(ch)

    # return logger
    return logger

def main (qSignal):
    # Logger config
    global LOGFILENAME
    log = setupLogger(LOGFILENAME)

    # begin
    log.info("Program Started")
    ser = None
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
    except Exception as e:
        log.error("Got Fatal error - {}".format(e))
        print("Got Fatal error\n");
        exit(4)

    while 1:
        # wait for ctrl-c
        try:
            squit = qSignal.get(block=False, timeout=0.1)
        except Empty as e:
            squit = False
            if squit == True:
                log.inf("Exiting")
                ser.close()
                exit(0)
            
        # get data
        try:
            data = ser.readline()
            if len(data) > 0:
                log.info(data)
        except KeyboardInterrupt as e:
            q.put(True)
            log.info("Ctrl + C pressed")

def handler(signal_received, frame):
    # handle cleanup
    print("SIGINT or CTRL-C detected. Exiting\n")
    q.put(True)
    exit(0)

if __name__ == "__main__":
    signal(SIGINT, handler)
    print("Running logger. Press ctrl-c to exit.\n")
    main(q)