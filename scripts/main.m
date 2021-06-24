%clear;

%% Connect web camera
%Define webcamlist
myWebcamlist = webcamlist;

%Display connected web cameras
camNum = length(myWebcamlist);
for cam = 1:camNum
    disp(string(myWebcamlist(cam)))
end

%Select camera by index of desired camera in list
deviceNum = input('Which camera index should be used?');
disp('Selected web camera: ' + string(myWebcamlist(deviceNum)))

%% Capture and save images of calibration(checkerboard) pattern
%Use non-square checkerboard to obtain orientation and origin of pattern

capture_calibration_data = false;
if capture_calibration_data
    %An accurate calibration requires at least 10-20 images
    numImgs = 20;
    %Define path to folder for image storage
    folder = ('C:\Users\Brukar\Documents\HVLRoboticsLab\robotic_vision_system\data\calibrationImgs');
    set_webcam_images(deviceNum, numImgs, folder);
end


