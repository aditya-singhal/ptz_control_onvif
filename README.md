# ptz_control_onvif
This script controls the movement of a PTZ camera. The user has to define the position of PTZ camera with respect to the origin(0, 0 ,0). Then provide a point of interest(x, y, z) with respect to origin. The camera will perform all required axis transformations to calcualte pan,tilt and zoom values and rotate the camera towards that point.
The script is written in python 2.7. and depends upon the following onvif library:
https://github.com/quatanium/python-onvif

# Camera axis
Axis of the camera is according to the right hand thumb rule where:
Thumb points towards +z axis
Index finger points towards +y axis
Middle finger points towards +x axis

# PTZ terminilogies:
Pitch is rotation about the x axis
Roll is rotation about the y axis
Yaw is rotation about the z axis

# Limitations:
This script is tested with a only PTZ camera as of now. To make it work with other cameras necessary changes has to be done to map the calculated angle values(pan, tilt and zoom) with actual camera input values.
