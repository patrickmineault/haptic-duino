# Haptic Duino

Haptic Duino is a project to create a linked set of Arduino devices that 
communicate with each other. One device, a Feather 32u4, has a BNO055 9-dof 
device on it; it beams 9-dof data as a packet via RF  to the other 
Arduino. The other Arduino, also a Feather 32u4, receives said information, and
translates it into a haptic signal via ERM motors. It also has capacitive touch
buttons that can be used to initiate a calibration sequence; when the button is 
pressed, the other device should first be held stable, then tilted, and finally
help stable, in sync with an LED indicator on the second, receiver device.

The project can be loaded and edited in platform.io.
