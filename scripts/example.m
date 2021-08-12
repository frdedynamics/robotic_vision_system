clear all; clc

%%

% Before playing around with the code:
% Make sure that the static properties of the class VisionNode are correct for your application
% Check that the ip address on both nodes are correct wrt. the chosen communication channel

node = VisionNode()
% Initialize network communication
node.initSocket
% Initialize robot to photographing position
node.setCmdJointPosition(node.ImgCaptureJointPosition)
node.moveRobot(node.MsgType.moveJoint)
% Fetch tcp position of photographing position(ImgCaputeJointPosition)
node.retrieveRobotInfo(node.MsgType.getTcpPos)
node.ImgCaptureTcpPosition = ans;
% Determine position of objects from the photographing position
node.setCmdTcpPosition(node.ImgCaptureTcpPosition)
node.getCmdTcpPosition

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

% Second input argument indicates that the leftmost object is selected
node.setCmdTcpXYPosition(objPositions, 2)
% Check that tcp position is correctly set before applying it to the robot
node.getCmdTcpPosition
% Robot is positioned on top of the given object
node.moveRobot(node.MsgType.moveTcp)
% Procedure for picking object
node.setCmdTcpZPosition(node.MoveZ.down)
node.getCmdTcpPosition
node.moveRobot(node.MsgType.moveTcp)
node.openGripper
node.setCmdTcpZPosition(node.MoveZ.pick_obj)
node.getCmdTcpPosition
node.moveRobot(node.MsgType.moveTcp)
node.closeGripper
node.setCmdTcpZPosition(node.MoveZ.up)
node.getCmdTcpPosition
node.moveRobot(node.MsgType.moveTcp)

% Do something here
% Example: drop object into a bowl
node.setCmdTcpPosition(node.ImgCaptureTcpPosition)
node.setCmdTcpXYPosition(objPositions, 1)
node.getCmdTcpPosition
node.moveRobot(node.MsgType.moveTcp)
%Place object
node.setCmdTcpZPosition(node.MoveZ.down)
node.getCmdTcpPosition
node.moveRobot(node.MsgType.moveTcp)
node.openGripper
node.closeGripper
node.setCmdTcpZPosition(node.MoveZ.up)
node.getCmdTcpPosition
node.moveRobot(node.MsgType.moveTcp)

% Pick second object
% Procedure performed analouguesly
node.setCmdTcpPosition(node.ImgCaptureTcpPosition)
node.setCmdTcpXYPosition(objPositions, 3)
node.moveRobot(node.MsgType.moveTcp)
node.setCmdTcpZPosition(node.MoveZ.down)
node.moveRobot(node.MsgType.moveTcp)
node.openGripper
node.setCmdTcpZPosition(node.MoveZ.pick_obj)
node.moveRobot(node.MsgType.moveTcp)
node.closeGripper
node.setCmdTcpZPosition(node.MoveZ.up)
node.moveRobot(node.MsgType.moveTcp)

% Do something here
% Example: drop object into a bowl
node.setCmdTcpPosition(node.ImgCaptureTcpPosition)
node.setCmdTcpXYPosition(objPositions, 1)
node.getCmdTcpPosition
node.moveRobot(node.MsgType.moveTcp)
%Place object
node.setCmdTcpZPosition(node.MoveZ.down)
node.getCmdTcpPosition
node.moveRobot(node.MsgType.moveTcp)
node.openGripper
node.closeGripper
node.setCmdTcpZPosition(node.MoveZ.up)
node.getCmdTcpPosition
node.moveRobot(node.MsgType.moveTcp)

% Close socket when done
node.closeSocket