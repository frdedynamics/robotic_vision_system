function [H_base_obj] = get_mapping_robot_object(node, R, t)
    z = zeros(1,3);
    H_cam_obj = [R t.'; z 1];
    H_base_cam = get_homogeneous_transform_robot();

    H_base_obj = H_base_cam * H_cam_obj;
end

function [H_base_cam] = get_homogeneous_transform_robot()
% Get Rigid body tree
[robot, robotData] = loadrobot('universalUR5',...
    'DataFormat', 'row',...
    'Gravity', [0, 0, -9.80665]);

%Camera mounted on robot
%configure robot set-up with camera mounting in addition to end-effector

config = randomConfiguration(robot); %Get robot config when taking snapshot
H_base_ee = getTransform(robot, config, 'ee_link', 'base_link');
H_ee_cam = eye(4); %? Find when specifications are known

H_base_cam =  H_base_ee * H_ee_cam;
end