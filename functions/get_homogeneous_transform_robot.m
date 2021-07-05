function [H_base_ee] = get_homogeneous_transform_robot()
% Get Rigid body tree
[robot, robotData] = loadrobot('universalUR5',...
    'DataFormat', 'row',...
    'Gravity', [0, 0, -9.80665]);

%configure robot set-up with camera mounting in addition to end-effector
config = randomConfiguration(robot); %Get robot config when taking snapshot
H_base_ee = getTransform(robot, config, 'ee_link', 'base_link');
%H_ee_cam = ? Find when specifications are known
end