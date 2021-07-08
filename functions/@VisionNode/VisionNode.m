classdef VisionNode < dynamicprops
    properties (Constant)
        % Checkerboard specifications
        Checkerboard = struct( ...
            'checkerboardSize', [5,8], ...
            'squareSize', 35 ... 
            );
        
        % Memory location
        Folder = struct(...
            'calibration', ('..\data\calibrationImgs'), ...
            'recognition', ('..\data\objectImgs')...
            );
        %FolderCalibData = ('..\data\calibrationImgs');
        %FolderRecogData = ('..\data\objectImgs');
    end
    
    properties
        % Image capture
        numCalibImg = 30; %An accurate calibration requires at least 10-20 images
        numRecogImg = 1;
        
        % Object measurements
        ObjectCenter = [];
        ObjectWidth = [];
        
        % Message content
        CmdTcpPosition = [0, 0, 0, 0, 0, 0];
        
        % Network
        RobotIp = '172.31.1.138';
        Socket %initialize?
    end
    
    methods
        %constructor of class VisionNode
        function node = VisionNode(objCenter, objWidth, newTcpPosition)
            node.ObjectCenter = objCenter;
            node.ObjectWidth = objWidth;
            node.CmdTcpPosition = newTcpPosition;
            %other things?
        end
        
        %set_webcam_images(node, numImgs, folder)
        
        [cameraParams, worldPoints] = get_camera_parameters(node, folderCalibData, checkerboardSize, squareSize)
        
        [imgSegmented, imgUndistorted, origin] = filter_recognition_image(node, folderRecogData, recogImgNum, cameraParams)
        
        [centroids, boundingBoxes] = detect_objects(node, imgSegmented, imgUndistorted)
        
        [R, t] = get_extrinsics(node, cameraParams, imgUndistorted, origin, worldPoints)
        
        [objLocation, objWidth] = get_obj_measurements(node, cameraParams, R, t, origin, centroids, boundingBoxes)
        
        [H_base_obj] = get_mapping_robot_object(node, R, t)
    end
    
    %methods (Static)
        %Static methods do not require an object of the class
        %Call staticMethod using the syntax classname.methodname:
    %end
    %https://se.mathworks.com/matlabcentral/answers/356254-function-in-class-not-working-which-normally-works
end


