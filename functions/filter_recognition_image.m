function [imgUndistorted, newOrigin] = filter_recognition_image(folder, recogImg, params)
    %Load image
    imgPath = fullfile(folder, sprintf('image%d.png', recogImg));
    img = im2double(imread(imgPath));
    %Correct image for lens distortion
    [imgUndistorted, newOrigin] = undistortImage(img, params); %'OutputView', 'full'
    
    %Filter image such that objects are easily seen (red objects)
    %Brute force filter for image segmentation - to be modified!
    imgHSV = rgb2hsv(img);
    hueValues = imgHSV(:, :, 1);
    threshold = 0.05;
    imgFiltered = (hueValues > threshold);
    
    %Plot
    figure(4); imshow(imgFiltered);
    title('Filtered image for object detection');
end