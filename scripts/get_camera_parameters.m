function [cameraParams, worldPoints] = get_camera_parameters(folder)
    global BOARD_SIZE
    
    calib_imgs = load_imgs(folder);
    
    %Detect checkerboard corners in images
    [imgPoints, boardSize] = detectCheckerboardPoints(calib_imgs);
    
    if ~isequal(boardSize, BOARD_SIZE)
        error("Board size seems weird. Detected: %dx%d, expected: %dx%d.", ...
            boardSize(1), boardSize(2), BOARD_SIZE(1), BOARD_SIZE(2))
    end
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

function imgArray = load_imgs(folder)
    %Number of images in directory
    numImgs = numel(dir(fullfile(folder, '*.png')));
    %Create cell array for holding calibration images
    imgArray = cell(1, numImgs);
    for i = 1:numImgs
        imgArray{i} = fullfile(folder, sprintf('image%d.png', i));
    end
end