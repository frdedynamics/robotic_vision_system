function [objLocation, objWidth] = get_obj_measurements(node, params, R, t, origin, centroids, boundingBoxes)
    [numObj, ~] = size(boundingBoxes);
    objWidth = zeros(numObj, 1);
    objLocation = zeros(numObj, 2);
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
        objLocation(i,:) = pointsToWorld(params, R, t, centroids(i,:));
    end
end