%clear;

%% Capture and save images of calibration(checkerboard) pattern with a web camera
%Use non-square checkerboard to obtain orientation and origin of pattern

capture_calibration_data = false;
%Define path to folder for image storage
folder = ('C:\Users\Brukar\Documents\HVLRoboticsLab\robotic_vision_system\data\calibrationImgs');
if capture_calibration_data
    %An accurate calibration requires at least 10-20 images
    numImgs = 20;
    set_webcam_images(numImgs, folder);
end

%% Compute camera intrincis and extrinsics
cameraParams = get_camera_parameters(folder);

%Capture image with chekerboard and object
capture_recognition_data = false;
folder = '..\data\objectImgs';
if capture_recognition_data
    numImgs = 1;
    set_webcam_images(numImgs, folder);
end

%Load image with checkerboard and objects to recognice
%Undistort image
%Filter image such that objects are easily seen