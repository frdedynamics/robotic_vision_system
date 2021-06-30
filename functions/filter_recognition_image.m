function [imgSegmented, imgUndistorted, newOrigin] = filter_recognition_image(folder, recogImg, params)
    %Load image
    imgPath = fullfile(folder, sprintf('image%d.png', recogImg));
    img = im2double(imread(imgPath));
    %Correct image for lens distortion
    [imgUndistorted, newOrigin] = undistortImage(img, params);
    
    %Segment image such that objects are easily seen
    I = rgb2gray(imgUndistorted);
    [~, threshold] = edge(I, 'sobel');
    I = edge(I, 'sobel', threshold * 0.5);

    %Ensure object edges are enclosed
    se90 = strel('line', 2, 90);
    se0 = strel('line', 2, 0);
    I = imdilate(I, [se90 se0]);
    %Fill object 
    I = imfill(I, 'holes');
    
    seDiamond = strel('diamond',2);
    imgSegmented = imerode(I, seDiamond);
    figure(9), imshow(imgSegmented), title('Filtered image');
end