# Tello Face Following and vSLAM

Implementation of face detection / following and vSLAM on a [Ryze Tello](https://www.ryzerobotics.com/tello) using its [Matlab toolkit](https://www.mathworks.com/hardware-support/tello-drone-matlab.html).

## Usage
You will need MATLAB R2020a or greater to access the Tello support package. You will also need the Computer Vision and Parallel Computing toolboxes. 
1. Clone the repo, unzip help_fncs.zip, and move all the helper functions into the same directory as main, track, and vslam. Make an `imgs` directory in project directory in order to save the ORB feature and particle map figures. Make `faces` and `slam` subdirectories in `imgs`.
2. Make sure that the Tello's firmware is updated and that the ground station is connected to the Tello's wifi.
3. Run main.m in a Matlab environment.

**Using this code may cause damage to the operating UAV, your person, or other persons. Your choice to fly using this code is yours alone and your responsibility. I am not liable.**

## Contents
- main.m: control flow script to demonstrate each of these on the Tello. 
- follow.m: face detection and following algorithm that returns the movement vector required to center the drone on the detected face, if there is one. 
- vslam.m: implements vSLAM using the drone's pinhole camera given a predetermined movement sequence that should be cycled a handful of times. 

## Future
1. Streamline main.m with user input to guide the program and improve the functionality of vslam.m as best I can for Tello. 
2. Implement general object detection alongside the face detection pipeline. 
3. Add autonomous movement based on point cloud --> remove need for a predetermined path. 
4. The implementations here are stepping stones to some more intelligent autonomous UAV behavior. I have the idea that I'll implemennt path planning on a Tello as well. Once I have that, I may integrate these three features into a Tello hide-n-seek project.

## References
### Papers

P. Viola and M. Jones, “Rapid Object Detection using a Boosted Cascade of Simple Features,” 2001 Comput. Vis. Pattern Recognit., 2001.

E. Rublee, V. Rabaud, K. Konolige, and G. Bradski, “ORB: An efficient alternative to SIFT or SURF,” Proc. IEEE Int. Conf. Comput. Vis., pp. 2564–2571, 2011, doi: 10.1109/ICCV.2011.6126544.

C. Cadena et al., “Past, present, and future of
simultaneous localization and mapping: Toward the robust-perception age,” IEEE Trans. Robot., vol. 32, no. 6, pp. 1309–1332, 2016, doi: 10.1109/TRO.2016.2624754.

R. Mur-Artal, J. M. M. Montiel, and J. D. Tardos, “ORB-SLAM: A Versatile and Accurate Monocular SLAM System,” IEEE Trans. Robot., vol. 31, no. 5, pp. 1147–1163, 2015, doi: 10.1109/TRO.2015.2463671.

B. Williams and I. Reid, “On combining visual SLAM and visual odometry,” Proc. - IEEE Int. Conf. Robot. Autom., pp. 3494–3500, 2010, doi: 10.1109/ROBOT.2010.5509248.

### Code 
- The vslam.m code is modified from the [vSLAM Matlab example](https://www.mathworks.com/help/vision/ug/monocular-visual-simultaneous-localization-and-mapping.html).

References from my first exposure to quad programming and face detection:
- [TelloTV](https://github.com/Jabrils/TelloTV)
- [TelloPython](https://github.com/dji-sdk/Tello-Python)

Feel free to use, make improvements, or ask questions!
