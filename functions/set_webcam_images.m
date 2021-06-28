function set_webcam_images(numImgs, folder)
    camDeviceNum = connect_webcam()
    
    for i=1:numImgs
        %Capture image
        img = get_webcam_img(camDeviceNum);
        userInput = input('Satisfied with the captured image? (1/0)');

        while userInput == 0
            img = get_webcam_img(camDeviceNum);
            userInput = input('Satisfied with the captured image? (1/0)');
        end

        save_img(folder, img, i)

        if i < numImgs
            userInput = input('Ready to capture the next image? (1/0)');
            if userInput == 0
                %Preview livestream for setting correct pattern pose
                camObj = webcam(camDeviceNum);
                preview(camObj)

                disp('Press any key to continue')
                pause
                closePreview(camObj)
            end
        end
    end
end

function [camDeviceNum] = connect_webcam()
    %Define webcamlist
    myWebcamlist = webcamlist;
    %Display connected web cameras
    camNum = length(myWebcamlist);
    for cam = 1:camNum
        disp(string(myWebcamlist(cam)))
    end
    
    %Select camera by index of desired camera in list
    camDeviceNum = input('Which camera index should be used?');
    disp('Selected web camera: ' + string(myWebcamlist(camDeviceNum)))
end

function [img] = get_webcam_img(camDeviceNum)
    %Construct web camera object
    camObj = webcam(camDeviceNum);
    %Preview a stream of image frames
    preview(camObj);
    %Acquire and display a single image frame
    img = snapshot(camObj);
    imshow(img);
    %Stop using camera
    camObj.closePreview
    clear('camObj');
end

function save_img(folder, img, imgNum)
    %Save image to path
    imgName = sprintf('Image #%d.png', imgNum);
    fullFileName = fullfile(folder, imgName);
    imwrite(img, fullFileName);
end

