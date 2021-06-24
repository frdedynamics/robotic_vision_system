%clear;

%% Capture and save images of calibration(checkerboard) pattern with a web camera
%Use non-square checkerboard to obtain orientation and origin of pattern

capture_calibration_data = true;
if capture_calibration_data
    %An accurate calibration requires at least 10-20 images
    numImgs = 20;
    %Define path to folder for image storage
    folder = ('C:\Users\Brukar\Documents\HVLRoboticsLab\robotic_vision_system\data\calibrationImgsTest');
    set_webcam_images(numImgs, folder);
end
