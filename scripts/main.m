clear all; clc

%% Add paths
addpath('../functions') %These functions are moved within @VisionNode folder
%which removes them from the path and automatically treats the .m files as
%methods of the class

%% Checkerboard specifications
checkerboardSize = [5,8];
squareSize = 35; %[mm]

%% Flags
capture_calibration_data = false;
folderCalibData = ('..\data\calibrationImgs');
numCalibImg = 30; %An accurate calibration requires at least 10-20 images
capture_recognition_data = false;
folderRecogData = ('..\data\objectImgs');
numRecogImg = 1;

%% Capture and save images of calibration(checkerboard) pattern with a web camera
%Use non-square checkerboard to obtain orientation and origin of pattern

%Define path to folder for image storage
if capture_calibration_data
    set_webcam_images(numCalibImgs, folderCalibData);
end

%% Compute camera intrincis and extrinsics
[cameraParams, worldPoints] = get_camera_parameters(folderCalibData, checkerboardSize, squareSize);

%Capture image with chekerboard and object
if capture_recognition_data
    set_webcam_images(numRecogImgs, folderRecogData);
end

recogImgNum = 8;
[imgSegmented, imgUndistorted, origin] = filter_recognition_image(folderRecogData, recogImgNum, cameraParams);

[centroids, boundingBoxes] = detect_objects(imgSegmented, imgUndistorted);

[R, t] = get_extrinsics(cameraParams, imgUndistorted, origin, worldPoints);

[objLocation, objWidth] = get_obj_measurements(cameraParams, R, t, origin, centroids, boundingBoxes);

z = zeros(1,3);
H_cam_obj = [R t.'; z 1];
H_base_ee = get_homogeneous_transform_robot();

%H_base_obj = H_base_ee * H_ee_cam * H_cam_obj;

   
