function filter_recognition_image(folder, recogImg, params)
    %Load image
    imgPath = fullfile(folder, sprintf('image%d.png', recogImg));
    img = im2double(imread(imgPath));
    %Correct image for lens distortion
    [imgUndistorted, newOrigin] = undistortImage(img, params); %'OutputView', 'full'
    
    %Filter image such that objects are easily seen
end