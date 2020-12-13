%% Face Tracking 
% detects faces  
% returns distance changes needed to center the drone on the detected face 
%
% Notes: 
% place in a while loop for the drone to follow faces
% consider threading this function in a background loop

%% follow
% cam: drone camera object
% detect: CascadeObjectDetector
% dist: index [1-5] for distance to maintain 
function [dX,dY,dZ,angle,faces] = follow(cam,detector,dist)

% bbox sizes for OpenCV
sizes = [140,120,100,80,60];

% capture frame and compute center
frame = snapshot(cam);
[width,height,~] = size(frame); % 720 x 960 x 3
center_x = floor(width/2);
center_y = floor(height/2);

% detect bounding boxes for faces
bbox = detector(frame);
faces = insertObjectAnnotation(frame,'Rectangle',bbox,'Face','Color','r','LineWidth',15);

% find the closest face
face_count = size(bbox,1);
closest = false;
mn = -Inf;
for i = 1:face_count
    face = bbox(i,:);
    w = face(3);
    h = face(4);
    area = w*h;
    if area > mn
        mn = area;
        closest = i;
    end
end

dX = 0;
dY = 0;
dZ = 0;
angle = pi/4;

% move drone to approx. center on the closest face
if closest
    
    closest_face = bbox(closest,:);
    x = closest_face(1);
    y = closest_face(2);
    w = closest_face(3);
    h = closest_face(4);
    bbox_x = floor((2*x+w)/2);
    bbox_y = floor((2*y+h)/2);
    
    % see move() documentation for axes info
    X_diff = (sizes(dist) - w); % move forward or backward
    Y_diff = (bbox_x - center_x)/width; % move left or right
    Z_diff = (center_y - bbox_y)/height; % move up or down (right hand rule -->> positive is down)

    % intervals for move coords are [-5,-0.2] or [0.2,5]
    % shift to fit
    % NOTE: change thresholds to change drone responsiveness
    if X_diff < -0.1
        dX = -0.2;
    elseif X_diff > 0.1
        dX = 0.2;
    else
        dX = 0;
    end
    
    if Y_diff < -0.1
        dY = -0.2;
    elseif Y_diff > 0.1
        dY = 0.2;
    else
        dY = 0;
    end
    
    if Z_diff < -0.5
        dZ = -0.2;
    elseif Z_diff > 0.5
        dZ = 0.2;
    else
        dZ = 0;
    end
    
    % angle for turning to face 
    angle = Y_diff*pi/4;
end