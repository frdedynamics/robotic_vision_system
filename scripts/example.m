clear all; clc

%% Example script
% This example is specially made to show functionality usage
% The code can be run section by section or all in one.
% Before playing around with the code:
% * Make sure that the static properties of the class VisionNode are correct for your application
% * Check that the ip address on both nodes are correct wrt. the chosen communication channel
% * The class method getCmdTcpPosition is used quite enough throughout the example to visualize the
% importance of being alert of what information that is being fed to the robot.

%%
% Construct class object
node = VisionNode()
% Initialize network communication
node.initSocket
% Initialize robot to photographing position
node.setCmdJointPosition(node.ImgCaptureJointPosition)
node.moveRobot(node.MsgType.moveJoint)
% Wait until robot has reached target position
node.atTargetPosition(node.CmdJointPosition, node.MsgType.getJointPos)
% Fetch tcp position of photographing position(ImgCaputeJointPosition)
node.retrieveRobotInfo(node.MsgType.getTcpPos)
node.ImgCaptureTcpPosition = ans;
% Determine position of objects from the photographing position
node.setCmdTcpPosition(node.ImgCaptureTcpPosition)
node.getCmdTcpPosition
%% Run robotic vision system

% Photograph scenery with objects and checkerboard
% For this example script three objects are given
% Two object for picking and one for placing the objects(leftmost in scenery)
node.set_webcam_images(node.numImgCapture.recognition, node.Folder.recognition)
% Find the tcp position of the objects wrt. the robot base
[cameraParams, worldPoints] = node.get_intrinsics(node.Folder.calibration, node.Checkerboard.checkerboardSize, node.Checkerboard.squareSize);
[imgSegmented, imgUndistorted, origin] = node.filter_recognition_image(node.Folder.recognition, 1, cameraParams);      
[centroids, boundingBoxes] = node.detect_objects(imgSegmented, imgUndistorted);
[R, t] = node.get_extrinsics(cameraParams, imgUndistorted, origin, worldPoints);
[objPositions] = node.get_obj_position(cameraParams, R, t, origin, centroids, boundingBoxes);
%% Pick and place procedure

% The leftmost object is in this example where the pieces are being placed
bowl = 1;
%Pick object from leftmost to rightmost
for obj=2:size(objPositions, 1)
    node.setCmdTcpXYPosition(objPositions, obj)
    % Check that tcp position is correctly set before applying it to the robot
    node.getCmdTcpPosition
    % Robot is positioned on top of the given objects estimated position
    node.moveRobot(node.MsgType.moveTcp)
    node.atTargetPosition(node.CmdTcpPosition, node.MsgType.getTcpPos)

    % Procedure for picking object
    node.setCmdTcpZPosition(node.MoveZ.down)
    node.getCmdTcpPosition
    node.moveRobot(node.MsgType.moveTcp)
    node.atTargetPosition(node.CmdTcpPosition, node.MsgType.getTcpPos)
    node.openGripper
    node.setCmdTcpZPosition(node.MoveZ.pick_obj)
    node.getCmdTcpPosition
    node.moveRobot(node.MsgType.moveTcp)
    node.atTargetPosition(node.CmdTcpPosition, node.MsgType.getTcpPos)
    node.closeGripper
    node.setCmdTcpZPosition(node.MoveZ.up)
    node.getCmdTcpPosition
    node.moveRobot(node.MsgType.moveTcp)
    node.atTargetPosition(node.CmdTcpPosition, node.MsgType.getTcpPos)

    % Example: Drop object into bowl
    node.setCmdTcpPosition(node.ImgCaptureTcpPosition)
    node.setCmdTcpXYPosition(objPositions, bowl)
    node.getCmdTcpPosition
    node.moveRobot(node.MsgType.moveTcp)
    node.atTargetPosition(node.CmdTcpPosition, node.MsgType.getTcpPos)
    %Place object
    node.setCmdTcpZPosition(node.MoveZ.down)
    node.getCmdTcpPosition
    node.moveRobot(node.MsgType.moveTcp)
    node.atTargetPosition(node.CmdTcpPosition, node.MsgType.getTcpPos)
    node.openGripper
    node.closeGripper
    node.setCmdTcpZPosition(node.MoveZ.up)
    node.getCmdTcpPosition
    node.moveRobot(node.MsgType.moveTcp)
    node.atTargetPosition(node.CmdTcpPosition, node.MsgType.getTcpPos)
end

% Close socket when done
node.closeSocket
disp("Example finished.")