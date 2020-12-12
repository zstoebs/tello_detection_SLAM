%% Considerations
%
% Environment: Tello, OpenCV face detection, and vSLAM are extremely sensitive
% to lighting in order to function properly. Tello doesn't counteract 
% drafts --> blown off course often. Theoretically, the best space for this
% problem is a spacious, well-lit, draftless, indoor area... make do with 
% what you got. 
%
% Connectivity: Tello programming depends on a ground station's connection
% to Tello's local wifi, which isn't great even for what it is. Often
% times, clearing the environment
%
% Reset: Tello's destructor includes auto-landing. No need to manually land
% the drone if an error is raised and it's left hovering. Run the cleanup
% section and it will force it to land and clean the environment.
%
% Common errors often solvable on reset: 
% 1. "Unable to receive response from the drone..." because the wifi is
% spotty and will often bug out in Matlab even though it technically still 
% has connection to the drone and can subsequently send move commands.
% 2. "Unable to execute 'move'. Execute 'takeoff' command first." because, 
% in my experience, there is a lot of overhead in passing the drone object 
% to function to control the drone within the function instead of the
% top-level code, which may not be the case for the others. I attempted to
% handle these errors where they occur. 
% 
% https://www.mathworks.com/matlabcentral/fileexchange/74434-matlab-support-package-for-ryze-tello-drones?s_tid=FX_rc2_behav
% The package is new and not widely used so the forum is small. Many of the
% above issues may be caused by poor connection, overheating, low battery,
% and a flurry of other picky statuses. Just clear the environment and
% rerun from the start, no need to disconnect from its wifi unless it needs
% to be turned off to cool down. 
% 
% Face detection %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The source is in track.m but main executes the move commands. It works
% pretty well as is but if it's misclassifying too many objects as faces
% then tweak the parameters of the detector (e.g. increase the minimum box
% size, increase the merge threshold, change the model, etc.). To make the
% drone adjust more "responsively" tweak the thresholds for the vector
% components in track.m. 
%
% vSLAM %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 

%% initial cleanup

close all
clear

%% connect, setup, and takeoff

drone = ryze();
cam = camera(drone);
fprintf("Battery: %d\n",drone.Battery)

takeoff(drone)
pause(2) % after takeoff, the drone needs a grace period for connection
moveup(drone,1.6,'WaitUntilDone',false)

%% face detection

% setup face detector s.t. false positives are minimized
faceDetector = vision.CascadeObjectDetector;
faceDetector.ClassificationModel = 'FrontalFaceLBP';
faceDetector.MinSize = [60 60];
faceDetector.MergeThreshold = 10;

stop = false;
count = 0;

start = tic;
while ~stop
    [dX,dY,dZ,angle,faces] = track(cam,faceDetector,2);
    
    imshow(faces)
    
    % try the command
    % if not received, continue and try again
    try
        if dX || dY || dZ
            
            % move
            move(drone,[dX,dY,dZ],'WaitUntilDone',false)
            fprintf("Moving with relative coords:\n")
            fprintf("dX = %d\n",dX)
            fprintf("dY = %d\n",dY)
            fprintf("dZ = %d\n",dZ)
            
            % turn to face
            turn(drone,angle)
            fprintf("Turning by %d radians\n",angle)

        end

    catch
        fprintf("Command not received. Continuing.\n")
    end
    
    count = count + 1;
    % reached threshold loops
    if count == mx
        stop = true;
    end
end
land(drone)

%% vSLAM

moveseq = {[0, 0.75, 0], 0; 
           [0, -0.5, 0], 0};

% vslam
% drone: drone object
% cam: drone's camera
% moveseq: cell array with rows of a vector [x,y,z] and an angle in radians
% cycles: total number of cycles for movement
% minMatches: minimum number of ORB feature matches for triangulation
vslam(drone, cam, moveseq, 5, 10)

land(drone)