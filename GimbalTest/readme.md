This is the test scripts for the WARG Gimbal and Grabber test.

### INSTRUCTIONS
1) Connect the Y servo to pin 5 and the X servo to pin 6 (yellow wire). Red wires of the servo motor should be connected to vcc (5V), and brown wires to ground pins.
2) Upload the `Gimbal_Grabber_Test.ino` sketch to the Arduino board

### Clean Solution
- Install the keyboard and pyserial lib by running
```
pip install keyboard pyserial
```
- Change the `COM` port variable to the appropriate string
- Run the Python Script (`sudo` is required to give terminal access to the keyboard) 
```
sudo python3 warg_servotest.py
```
- use WASD to move the servo!

### Unclean (but easier) solution:
- Open the Arduino Serial Monitor and type in either 1, 2, 3, or 4 (and press ENTER) to control the Servo

> Note: Use the clean solution if you want smoother movements. Using the unclean one will result in lots of jitter since you need to hit enter to send a command.
