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
            'recognition', ('..\data\recognitionImgs')...
            );
        
        % Message types: Robot tasks
        MsgType = struct(...
            'moveJoint', ('(1)'), ...
            'getJointPos', ('(2)'), ...
            'openGrip', ('(3)'), ...
            'closeGrip', ('(4)'), ...
            'getTcpPos', ('(5)'), ...
            'moveTcp', ('(6)') ...
            );
    end
    
    properties
        % Image capture
        numImgCapture = struct(...
        'calibration', 30,... %An accurate calibration requires at least 10-20 images
        'recognition', 1 ...
        );
        
        % Object measurements
        ObjectPositions = []; %[X1,Y1; X2, Y2; ...; XnumObjs,YnumObjs]
        ObjectWidths = [];
        
        %Object position relative to robot base
        ObjBasePostions = []; %[X1, Y1, Z1; ...; XnumObjs, YnumObj, ZnumObj]
        
        % Message content
        CmdTcpPosition = [0.130, -0.440, 0.530, 3.141, -0.002, -0.012]; %[X, Y, Z(FIXED), Rx, Ry, Rz]
        CmdJointPosition = [pi/2, -pi/2, pi/3, (5/3)*pi, -pi/2, -pi];
        %Z fixed (For Z to be constant, the axis of the camera must be perpendicular to a flat surface being photographed), orientation unknown

        % Network
        RobotIp = '172.31.1.252';
        Socket
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
        
        set_webcam_images(node, numImgs, folder)
        
        [cameraParams, worldPoints] = get_camera_parameters(node, folderCalibData, checkerboardSize, squareSize)
        
        [imgSegmented, imgUndistorted, origin] = filter_recognition_image(node, folderRecogData, recogImgNum, cameraParams)
        
        [centroids, boundingBoxes] = detect_objects(node, imgSegmented, imgUndistorted)
        
        [R, t] = get_extrinsics(node, cameraParams, imgUndistorted, origin, worldPoints)
     
        [objBasePositions] = get_obj_position(node, cameraParams, R, t, origin, centroids, boundingBoxes)
        
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
        
        function setCmdTcpPosition(node, currentCmdPosition) 
            if size(currentCmdPosition, 2) == 6 % needs more/better check
                node.CmdTcpPosition = currentCmdPosition;
            else
                ERROR("TCP position expected as array of 6 with the format [X, Y, Z, Rx, Ry, Rz]")
            end
        end
        
        function setCmdTcpObjPosition(node, objBasePosition) 
            if size(objBasePosition, 1) == 1 % needs more/better check
                node.CmdTcpPosition(1:2) = objBasePosition(1:2);
            else
                ERROR("TCP position expected as array of 6 with the format [X, Y, Z, Rx, Ry, Rz]")
            end
        end
        
        function [currentCmdTcpPosition] = getCmdTcpPosition(node)
            currentCmdTcpPosition = node.CmdTcpPosition;
        end

        function moveRobot(node, msgType)
            if msgType == node.MsgType.moveTcp
                moveMsg = ['(',num2str(node.CmdTcpPosition(1)),',',... 
                num2str(node.CmdTcpPosition(2)),',',...
                num2str(node.CmdTcpPosition(3)),',',...
                num2str(node.CmdTcpPosition(4)),',',...
                num2str(node.CmdTcpPosition(5)),',',...
                num2str(node.CmdTcpPosition(6)),...
                ')'];
            elseif msgType == node.MsgType.moveJoint
                moveMsg = ['(',num2str(node.CmdJointPosition(1)),',',...
                num2str(node.CmdJointPosition(2)),',',...
                num2str(node.CmdJointPosition(3)),',',...
                num2str(node.CmdJointPosition(4)),',',...
                num2str(node.CmdJointPosition(5)),',',...
                num2str(node.CmdJointPosition(6)),...
                ')'];
            else
                disp("Invalid move request sent to robot.")
            end
            
            fprintf(node.Socket, msgType);
            pause(0.01);
            fprintf(node.Socket, moveMsg);
            while node.Socket.BytesAvailable==0
            end
            
            success = fscanf(node.Socket,'%c',node.Socket.BytesAvailable);
            if ~success
                ERROR("Failed to send command.")
            else
                if msgType == node.MsgType.moveTcp
                    disp("Tcp command sent.")
                elseif msgType == node.MsgType.moveJoint
                    disp("Joint command sent.")
                end
            end
        end
        
        function [msg] = retrieveRobotInfo(node, msgType)
%             if (msgType ~= node.MsgType.getTcpPos) && (msgType ~= node.MsgType.getJointPos)
%                 disp("Invalid read request.")
%                 return
%             end
            
            fprintf(node.Socket, msgType);
            pause(0.01);
            while node.Socket.BytesAvailable==0
            end

            robotMsg = fscanf(node.Socket,'%c', node.Socket.BytesAvailable);
            if ~strcmp(robotMsg(1),'[') || ~strcmp(robotMsg(end),']')
                error('Robot read error.')
            end
            
            robotMsg(end) = ',';
            c = 1;
            msg = zeros(1,6);
            for i = 1 : 6
                C = [];
                isDone = 0;
                while(~isDone)
                    
                    c = c + 1;
                    if strcmp(robotMsg(c) , ',')
                        isDone = 1;
                    else
                        C = [C,robotMsg(c)];
                    end
                end
                msg(i) = str2double(C);   
            end
            for i = 1 : length(msg)
                if isnan(msg(i))
                    error('Robot read error (Nan)')
                end
            end
  
            if msgType == node.MsgType.getTcpPos
                disp("Current tcp pose of robot received.")
                node.CmdTcpPosition = msg;
            elseif msgType == node.MsgType.getJointPos
                disp("Current joint position of robot received.")
                node.CmdJointPosition = msg;
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
            end
            
            success = fscanf(node.Socket,'%c',node.Socket.BytesAvailable);
            if ~strcmp(success,'1')
                error('error sending close gripper command')
            end
        end

    end
end


