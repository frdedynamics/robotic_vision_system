classdef VisionNode < dynamicprops
    properties (Constant)
        % Checkerboard specifications
        Checkerboard = struct( ...
            'checkerboardSize', [5,8], ...
            'squareSize', 35 ... 
            );
        
        % Object specifications 
        Object = struct(...
            'height', 0.016 ...
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
        
        % Set standardised movement in Z direction
        % Tune to meet your system
        MoveZ = struct(...
            'up', 0.52, ...
            'down', 0.12, ... 
            'pick_obj', 0.02 ...
            );
    end
    
    properties
        % Image capture
        numImgCapture = struct(...
        'calibration', 30,... % An accurate calibration requires at least 10-20 images
        'recognition', 1 ...
        );
        
        % Set tcp and joint positions
        CmdTcpPosition = zeros(1,6); %[0.130, -0.440, 0.530, 3.141, -0.002, -0.012];       % [X, Y, Z(FIXED), Rx, Ry, Rz]
        CmdJointPosition = zeros(1,6); %[pi/2, -pi/2, pi/3, (5/3)*pi, -pi/2, -pi];
        % Tcp and joint position for image capture
        ImgCaptureTcpPosition = [0.130, -0.440, 0.530, 3.141, -0.002, -0.012]; % Tune when joints at ImgCapureJointPosition
        ImgCaptureJointPosition = [pi/2, -pi/2, pi/3, (5/3)*pi, -pi/2, -pi];  % Camera must be perpendicular to the surface being photographed
        
        % Network
        RobotIp = '172.31.1.101'; 
        Socket
        
        % Object position relative to robot base
        objPositions = []; %[X1, Y1, Z1; ...; XnumObjs, YnumObj, ZnumObj]
        
        % State of the gripper
        GripperState = struct(...
            'closed', true, ...
            'open', false ...
            );  
    end
    
    methods
        %constructor of class VisionNode
        function node = VisionNode()
        end
        
        set_webcam_images(node, numImgs, folder)
        
        [cameraParams, worldPoints] = get_intrinsics(node, folderCalibData, checkerboardSize, squareSize)
        
        [imgSegmented, imgUndistorted, newOrigin] = filter_recognition_image(node, folderRecogData, recogImgNum, cameraParams)
        
        [centroids, boundingBoxes] = detect_objects(node, imgSegmented, imgUndistorted)
        
        [R, t] = get_extrinsics(node, cameraParams, imgUndistorted, newOrigin, worldPoints)
     
        [objPositions] = get_obj_position(node, cameraParams, R, t, newOrigin, centroids, boundingBoxes)
        
        %% Network specific methods
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
        
        function setCmdTcpPosition(node, currentTcpPosition) 
            if size(currentTcpPosition, 2) == 6 % needs more/better check
                node.CmdTcpPosition = currentTcpPosition;
            else
                ERROR("TCP position expected as array of 6 with the format [X, Y, Z, Rx, Ry, Rz]")
            end
        end
        
        function setCmdTcpXYPosition(node, objPositions, objNum) 
            if size(objPositions, 2) == 3 % needs more/better check
                xOffset = 0.005; yOffset = 0.09; % Due to inaccurate measurements for the dh parameters
                node.CmdTcpPosition(1) = node.ImgCaptureTcpPosition(1) - objPositions(objNum, 1) + xOffset;
                node.CmdTcpPosition(2) = node.ImgCaptureTcpPosition(2) - objPositions(objNum, 2) - yOffset;
            else
                ERROR("TCP XY position expected as array of 3 with columns [X, Y, Z] and as many rows as detected objects. Please spescify the object you want the tcp to position on top of from left to right (?).")
            end
        end
        
        function setCmdTcpZPosition(node, moveZ)
            if moveZ == node.MoveZ.pick_obj
                if node.GripperState.open == true
                    node.CmdTcpPosition(3) = moveZ + (node.Object.height/2);
                else
                    disp('Gripper has to be opened in order to move to the object picking position.')
                    return
                end
            else
                node.CmdTcpPosition(3) = moveZ;
            end
        end
        
        function [currentCmdTcpPosition] = getCmdTcpPosition(node)
            currentCmdTcpPosition = node.CmdTcpPosition;
        end
        
        function setCmdJointPosition(node, currentJointPosition) 
            if size(currentJointPosition, 2) == 6 % needs more/better check
                node.CmdJointPosition = currentJointPosition;
            else
                ERROR("Joint position expected as array of 6 with the format [Base, Shoulder, Elbow, Wrist 1, Wrist 2, Wrist 3] in radians.")
            end
        end
        
        function [currentCmdJointPosition] = getCmdJointPosition(node)
            currentCmdJointPosition = node.CmdJointPosition;
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
                disp('Invalid move request.')
                return
            end
            
            fprintf(node.Socket, msgType);
            pause(0.01);
            fprintf(node.Socket, moveMsg);
            while node.Socket.BytesAvailable==0
            end
            
            success = fscanf(node.Socket,'%c',node.Socket.BytesAvailable);
            if ~success
                ERROR("Failed to send move command.")
            else
                if msgType == node.MsgType.moveTcp
                    disp('Tcp move command sent.')
                elseif msgType == node.MsgType.moveJoint
                    disp('Joint move command sent.')
                end
            end
        end
        
        function [msg] = retrieveRobotInfo(node, msgType)
            if msgType == node.MsgType.getTcpPos
                disp('Tcp read request sent.')
            elseif msgType == node.MsgType.getJointPos
                disp('Joint read request sent.')
            else
                disp('Invalid read request.')
                return
            end
            
            fprintf(node.Socket, msgType);
            pause(0.01);
            while node.Socket.BytesAvailable==0
            end

            robotMsg = fscanf(node.Socket,'%c', node.Socket.BytesAvailable);
            if ~strcmp(robotMsg(1),'[') || ~strcmp(robotMsg(end),']')
                ERROR("Robot read error.")
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
                    ERROR("Robot read error (Nan)")
                end
            end
  
            if msgType == node.MsgType.getTcpPos
                disp('Current tcp pose of robot received.')
                %node.CmdTcpPosition = msg;
            elseif msgType == node.MsgType.getJointPos
                disp('Current joint position of robot received.')
                %node.CmdJointPosition = msg;
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
                ERROR("Error sending open gripper command.")
            else
                node.GripperState.closed = false;
                node.GripperState.open = true;  
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
                ERROR("Error sending close gripper command.")
            else
                node.GripperState.open = false;
                node.GripperState.closed = true; 
            end
        end

    end
end


