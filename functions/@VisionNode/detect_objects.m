function [centroid, boxes] = detect_objects(node, imgSegmented, imgUndistorted)
    %Use segmented image to find connected regions
    %Set Blob Analysis object properties
    blobAnalysis = vision.BlobAnalysis('AreaOutputPort', true,...
        'CentroidOutputPort', true,...
        'BoundingBoxOutputPort', true,...
        'MaximumBlobArea', 50000, 'MinimumBlobArea', 100, 'ExcludeBorderBlobs', true);
    [areas, centroid, boxes] = blobAnalysis(imgSegmented);

    %Label detected objects and mark the center
    imgDetectedObj = insertObjectAnnotation(imgUndistorted, 'rectangle', boxes, 'object');
    imgDetectedObj = insertMarker(imgDetectedObj, centroid, 'color', 'blue', 'size', 3);
    %Plot result
    figure(10); imshow(imgDetectedObj); title('Detected Objects');
end