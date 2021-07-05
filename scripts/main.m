clear all; clc

%% Add paths
addpath('../functions')

%% Checkerboard specifications
checkerboardSize = [5,8];
squareSize = 35; %[mm]

%% Flags
capture_calibration_data = false;
numCalibImg = 30; %An accurate calibration requires at least 10-20 images
capture_recognition_data = false;
numRecogImg = 1;

%% Capture and save images of calibration(checkerboard) pattern with a web camera
%Use non-square checkerboard to obtain orientation and origin of pattern

%Define path to folder for image storage
folder = ('..\data\calibrationImgs');
if capture_calibration_data
    set_webcam_images(numCalibImgs, folder);
end

%% Compute camera intrincis and extrinsics
[cameraParams, worldPoints] = get_camera_parameters(folder, checkerboardSize, squareSize);

%Capture image with chekerboard and object
folder = ('..\data\objectImgs');
if capture_recognition_data
    set_webcam_images(numRecogImgs, folder);
end

recogImgNum = 8;
[imgSegmented, imgUndistorted, origin] = filter_recognition_image(folder, recogImgNum, cameraParams);

[centroids, boundingBoxes] = detect_objects(imgSegmented, imgUndistorted);

[R, t] = get_extrinsics(cameraParams, imgUndistorted, origin, worldPoints);

[objLocation, objWidth] = get_obj_measurements(cameraParams, R, t, origin, centroids, boundingBoxes);

z = zeros(1,3);
H_cam_obj = [R t.'; z 1];
H_base_ee = get_homogeneous_transform_robot();

%H_base_obj = H_base_ee * H_ee_cam * H_cam_obj;

   
