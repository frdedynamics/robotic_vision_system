clear all; clc

%% Add paths
addpath('../functions')

%% Checkerboard specifications
global BOARD_SIZE
global SQUARE_SIZE
BOARD_SIZE = [5,8];
SQUARE_SIZE = 35; %[mm]

%% Capture and save images of calibration(checkerboard) pattern with a web camera
%Use non-square checkerboard to obtain orientation and origin of pattern

capture_calibration_data = false;
%Define path to folder for image storage
folder = ('..\data\calibrationImgs');
if capture_calibration_data
    %An accurate calibration requires at least 10-20 images
    numImgs = 30;
    set_webcam_images(numImgs, folder);
end

%% Compute camera intrincis and extrinsics
[cameraParams, worldPoints] = get_camera_parameters(folder);

%Capture image with chekerboard and object
capture_recognition_data = false;
folder = ('..\data\objectImgs');
if capture_recognition_data
    numImgs = 5;
    set_webcam_images(numImgs, folder);
end

recogImgNum = 8;
[imgSegmented, imgUndistorted, origin] = filter_recognition_image(folder, recogImgNum, cameraParams);

%Compute extrinsics
[R, t] = get_extrinsics(cameraParams, imgUndistorted, origin, worldPoints);