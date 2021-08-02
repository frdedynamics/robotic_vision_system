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
        
        % Message types: Robot tasks
        MsgType = struct(...
            'openGrip', ('(3)'), ...
            'closeGrip', ('(4)'), ...
            'moveTcp', ('(6)') ...
            );
    end
    
    properties
        % Image capture
        numCalibImg = 30; %An accurate calibration requires at least 10-20 images
        numRecogImg = 1;
        
        % Object measurements
        ObjectPositions = []; %[X1,Y1; X2, Y2; ...; XnumObjs,YnumObjs]
        ObjectWidths = [];
        
        %Object position relative to robot base
        ObjBasePostions = []; %[X1, Y1, Z1; ...; XnumObjs, YnumObj, ZnumObj]
        
        % Message content
        %Z = 0.4; 
        CmdTcpPosition = [0.065,-0.520, 0.4, 1.863, -0.083, -0.1]; %[X, Y, Z(FIXED), Rx, Ry, Rz]
        %Z fixed (For Z to be constant, the axis of the camera must be perpendicular to a flat surface being photographed), orientation unknown
        %Get orientation from RBT
        
        % Network
        RobotIp = '172.31.1.101';
        Socket %initialize?
    end
    
    methods
        %constructor of class VisionNode
        function node = VisionNode(currentTcpPosition)
            % Initialize with current tcp position of robot
            if length(currentTcpPosition) == 6 % needs more/better check
                node.CmdTcpPosition = currentTcpPosition;
            else
                ERROR("TCP position expected as array of 6 with the format [X, Y, Z, Rx, Ry, Rz]")
            end
        end
        
        %set_webcam_images(node, numImgs, folder)
        
        [cameraParams, worldPoints] = get_camera_parameters(node, folderCalibData, checkerboardSize, squareSize)
        
        [imgSegmented, imgUndistorted, origin] = filter_recognition_image(node, folderRecogData, recogImgNum, cameraParams)
        
        [centroids, boundingBoxes] = detect_objects(node, imgSegmented, imgUndistorted)
        
        [R, t] = get_extrinsics(node, cameraParams, imgUndistorted, origin, worldPoints)
     
        [objBasePositions] = get_obj_position(node, cameraParams, R, t, Z, origin, centroids, boundingBoxes)
        
        %% Network specific methods
        % must be able to perform these tasks:
        % 1. Init Socket and establish connection
        % 2. Receive messages from robot
        % 3. Send messages to robot - TODO: generalize sending by introducing message types
        % 4. Close Socket
        
        function initSocket(node)
            % Connect to robot
            Socket_conn = tcpip(node.RobotIp, 30000,'NetworkRole','server');
            fclose(Socket_conn);
            disp('Press Play on Robot...')
            fopen(Socket_conn);
            disp('Connected!');
            node.Socket = Socket_conn;
        end
        
        function closeSocket(node)
            clear node.Socket
        end
        
        function setCmdTcpPosition(node, objBasePosition) 
            if size(objBasePosition, 1) == 1 % needs more/better check
                node.CmdTcpPosition(1:3) = objBasePosition;
            else
                ERROR("TCP position expected as array of 6 with the format [X, Y, Z, Rx, Ry, Rz]")
            end
        end
        function [currentCmdTcpPosition] = getCmdTcpPosition(node)
            currentCmdTcpPosition = node.CmdTcpPosition;
        end
        
        function moveToTcpPosition(node)
            tcpChar = ['(',num2str(node.CmdTcpPosition(1)),',',... 
                num2str(node.CmdTcpPosition(2)),',',...
                num2str(node.CmdTcpPosition(3)),',',...
                num2str(node.CmdTcpPosition(4)),',',...
                num2str(node.CmdTcpPosition(5)),',',...
                num2str(node.CmdTcpPosition(6)),...
                ')'];
            
            fprintf(node.Socket, node.MsgType.moveTcp);
            pause(0.01);% Tune this to meet your system
            fprintf(node.Socket, tcpChar);
            while node.Socket.BytesAvailable==0 % wait for response
                %node.Socket.BytesAvailable
            end
            success = fscanf(node.Socket,'%c',node.Socket.BytesAvailable);
            if ~success
                ERROR("Failed to send TCP command.")
            else
                disp("TCP command sent.")
            end
        end
        
        function openGripper(node)
            % Open the gripper
            % Long Qian 2016
            if node.Socket.BytesAvailable>0
                fscanf(node.Socket,'%c',node.Socket.BytesAvailable);
            end
            fprintf(node.Socket, node.MsgType.openGrip);
            while node.Socket.BytesAvailable==0
                %node.Socket.BytesAvailable
            end
            success = fscanf(node.Socket,'%c',node.Socket.BytesAvailable);
            if ~strcmp(success,'1')
                error('error sending open gripper command')
            end
        end
        
        function closeGripper(node)
            % Close the gripper
            % Long Qian 2016
            if node.Socket.BytesAvailable>0
                fscanf(node.Socket, '%c', node.Socket.BytesAvailable);
            end
            fprintf(node.Socket, node.MsgType.closeGrip);
            while node.Socket.BytesAvailable==0
                %node.Socket.BytesAvailable
            end
            success = fscanf(node.Socket,'%c',node.Socket.BytesAvailable);
            if ~strcmp(success,'1')
                error('error sending close gripper command')
            end
        end

    end
    
    %methods (Static)
        %Static methods do not require an object of the class
        %Call staticMethod using the syntax classname.methodname:
    %end
    %https://se.mathworks.com/matlabcentral/answers/356254-function-in-class-not-working-which-normally-works
end


