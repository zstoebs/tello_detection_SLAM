# Tello Face Detection and vSLAM

Implementation of face detection / following and vSLAM on a [Ryze Tello](https://www.ryzerobotics.com/tello) using its [Matlab toolkit](https://www.mathworks.com/hardware-support/tello-drone-matlab.html).

## Usage
1. Clone the repo, unzip help_fncs.zip, and move all the helper functions into the same directory as main, track, and vslam. Make an `imgs` directory in project directory in order to save the ORB feature and particle map figures. 
2. Make sure that the Tello's firmware is updated and that the ground station is connected to the Tello's wifi.
3. Run main.m in a Matlab environment.

## Contents
- main.m: control flow script to demonstrate each of these on the Tello. 
- track.m: facial detection algorithm that returns the movement vector required to center the drone on the detected face, if there is one. 
- vslam.m: implements vSLAM using the drone's pinhole camera given a predetermined movement sequence that should be cycled a handful of times. 

## References
- The vslam.m code is modified from the [vSLAM Matlab example](https://www.mathworks.com/help/vision/ug/monocular-visual-simultaneous-localization-and-mapping.html).

References from my first exposure to quad programming and face detection:
- [TelloTV](https://github.com/Jabrils/TelloTV)
- [TelloPython](https://github.com/dji-sdk/Tello-Python)

Feel free to use, make improvements, or ask questions!
