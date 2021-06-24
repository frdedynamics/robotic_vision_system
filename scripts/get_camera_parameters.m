function [cameraParams] = get_camera_parameters(folder)
    calib_imgs = load_calibration_imgs(folder);
    
    %Detect checkerboard corners in images
    [imgPoints, boardSize] = detectCheckerboardPoints(calib_imgs);

    %Generate the world coordinates of the checkerboard corners in the
    %pattern-centric coordinate system, with the upper-left corner at (0,0)
    squareSize = 35; %[mm]
    worldPoints = generateCheckerboardPoints(boardSize, squareSize);

    %Calibrate camera
    I = imread(calib_imgs{1});
    imgSize = [size(I, 1), size(I, 2)];
    cameraParams = estimateCameraParameters(imgPoints, worldPoints, 'ImageSize', imgSize);
    
    %% Figures
    %Evaluate calibration accuracy
    figure(1); showReprojectionErrors(cameraParams);
    title('Reprojection Errors');
    
    %Visualize camera extrinsics
    figure(2);
    showExtrinsics(cameraParams);
    drawnow;

    %Plot detected and reprojected points
    figure(3); 
    imshow(calib_imgs{1}); 
    hold on;
    plot(imgPoints(:,1,1), imgPoints(:,2,1),'go');
    plot(cameraParams.ReprojectedPoints(:,1,1),cameraParams.ReprojectedPoints(:,2,1),'r+');
    legend('Detected Points','Reprojected Points');
    hold off;
end

function calib_imgs = load_calibration_imgs(folder)
    %Number of images in directory
    numImgs = numel(dir(fullfile(folder, '*.png')));
    %Create cell array for holding calibration images
    calib_imgs = cell(1, numImgs);
    for i = 1:numImgs
        calib_imgs{i} = fullfile(folder, sprintf('Image #%d.png', i));
    end
end