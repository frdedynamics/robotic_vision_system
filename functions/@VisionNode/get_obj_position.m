function [objBasePositions] = get_obj_position(node, cameraParams, R, t, origin, centroids, boundingBoxes)
    [objPositions, ~] = get_obj_measurements(cameraParams, R, t, origin, centroids, boundingBoxes);
    H_base_obj = get_mapping_robot_object(R, t);
    
    objBasePositions = zeros(size(objPositions, 1), 3);
    for i=1:size(objPositions,1)
        XYZ1 = [objPositions(i,:).'; 0; 1] * 0.001; %[mm]->[m]
        XYZ1Base = H_base_obj * XYZ1;
        objBasePositions(i, :)  = XYZ1Base(1:3);
    end
end

function [H_base_obj] = get_mapping_robot_object(R, t)
    z = zeros(1,3);
    H_cam_obj = [R t.'; z 1];
    H_base_cam = get_homogeneous_transform_robot();
    
    H_base_obj = H_base_cam * H_cam_obj;
end

function [H_base_cam] = get_homogeneous_transform_robot()
    % Get Rigid body tree
    addpath('../data/ur5e')
    %addpath('C:\Users\Brukar\Documents\HVLRoboticsLab\robotic_vision_system\data\ur5e')

    robot = importrobot('ur5e_joint_limited_robot.urdf');
    robot.DataFormat = 'column';
    robot.Gravity = [0, 0, -9.80665];

    % Camera mounted on robot
    % Configure robot set-up with camera mounting in addition to end-effector
    zividOnArmExtender = rigidBody('extender');
    setFixedTransform(zividOnArmExtender.Joint, [0 0 0.071 pi], 'dh'); %[a alpha d theta]
    addBody(robot, zividOnArmExtender, 'tool0');

    tcp = rigidBody('tcp');
    setFixedTransform(tcp.Joint, [0 0 0.147 0], 'dh');
    addBody(robot, tcp, 'extender');

    cameraMount = rigidBody('cam_mount');
    setFixedTransform(cameraMount.Joint, [-0.087 0 -0.132 pi/2], 'dh');
    addBody(robot, cameraMount, 'tcp');
    
     cam = rigidBody('webcam');
     setFixedTransform(cam.Joint, [0 0 0 -pi/2], 'dh');
     addBody(robot, cam, 'cam_mount');
   
    %config = randomConfiguration(robot); %Replace by robot config when taking snapshot
    config = [pi/2, -pi/2, pi/2, (3/2)*pi, -pi/2, -pi]'; 
    %show(robot,config);
    H_base_ee = getTransform(robot, config, 'base', 'tcp');
    H_ee_cam = getTransform(robot, config, 'tcp', 'webcam');

    H_base_cam =  H_base_ee * H_ee_cam;
end

function [objPositions, objWidth] = get_obj_measurements(params, R, t, origin, centroids, boundingBoxes)
    [numObj, ~] = size(boundingBoxes);
    objWidth = zeros(numObj, 1);
    objPositions = zeros(numObj, 2);
    for i=1:numObj
        %Fetch coordinates of box i: use top-left and top-right corner to
        %compute object width
        box = boundingBoxes(i,:); %[x,y, width, height], where (x,y) = top-left corner
        %Use origin for undistorted image and zero pad
        box  = double(box) + double([origin 0 0]);
        topLCorner = box(1:2);
        topRCorner = [(box(1) + box(3)) box(2)];
        boxPoints = [topLCorner; topRCorner];
        %Map to world
        worldBoxPoints = pointsToWorld(params, R, t, boxPoints);
        %Compute distanse between corners
        objWidth(i) = abs(worldBoxPoints(2,1) - worldBoxPoints(1,1)); %[mm]

        %Map center points to world coordinates
        objPositions(i,:) = pointsToWorld(params, R, t, centroids(i,:));
    end
end