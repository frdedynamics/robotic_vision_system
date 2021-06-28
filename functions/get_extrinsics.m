function [R, t] = get_extrinsics(params, imgUndistorted, newOrigin, worldPoints)
    [imagePoints, boardSize] = detectCheckerboardPoints(imgUndistorted);
    squareSize = 35; %[mm]
    imagePoints = imagePoints + newOrigin;
    [R, t] = extrinsics(imagePoints, worldPoints, params);
end
