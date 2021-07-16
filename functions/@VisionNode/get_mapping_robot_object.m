function [H_base_obj] = get_mapping_robot_object(node, R, t)
    z = zeros(1,3);
    H_cam_obj = [R t.'; z 1];
    H_base_cam = get_homogeneous_transform_robot();
    
    H_base_obj = H_base_ee * H_cam_obj;
end

function [H_base_cam] = get_homogeneous_transform_robot()
% Get Rigid body tree
[robot, robotData] = loadrobot('universalUR5',...
    'DataFormat', 'row',...
    'Gravity', [0, 0, -9.80665]);

% Camera mounted on robot
% Configure robot set-up with camera mounting in addition to end-effector
zividOnArmExtender = rigidBody('extender');
setFixedTransform(zividOnArmExtender.Joint, [0 0 0.071 0], 'dh'); %[a alpha d theta]
addBody(robot, zividOnArmExtender, 'tool0');

tcp = rigidBody('tcp');
setFixedTransform(tcp.Joint, [0 0 0.147 0], 'dh');
addBody(robot, tcp, 'extender');

zividCameraMount = rigidBody('webcam_mount');
setFixedTransform(zividCameraMount.Joint, [0.087 0 -0.132 pi/2], 'dh');
addBody(robot, zividCameraMount, 'tcp');

config = randomConfiguration(robot); %Get robot config when taking snapshot
H_base_ee = getTransform(robot, config, 'base_link', 'tcp');
H_ee_cam = getTransform(robot, config, 'tcp', 'camera');

H_base_cam =  H_base_ee * H_ee_cam;
end