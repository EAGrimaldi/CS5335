% Localize a sphere in the point cloud. Given a point cloud as input, this
% function should locate the position and radius of a sphere.
% input: ptCloud -> a pointCloud object that contains the point cloud (see
%                   Matlab documentation)
% output: center -> 3x1 vector denoting sphere center
%         radius -> scalar radius of sphere
function [best_center,best_radius] = Q2(ptCloud)
    rmin = 0.05;
    rmax = 0.11;
    rdiff = rmax-rmin;
    points = ptCloud.Location;
    num_points = size(points,1);
    if num_points > 20000
        ratio = 20000/num_points;
        dsCloud = pcdownsample(ptCloud,'random',ratio);
        num_points = ratio*num_points;
        points = dsCloud.Location;
        normals = pcnormals(dsCloud);
    else
        normals = pcnormals(ptCloud);
    end
    delta = 0.0002;
    best_center = zeros(1,3);
    best_radius = 0;
    best_score = 0;
    num_samples = 10000;
    for i=randi(num_points,1,num_samples)
        point = points(i,:);
        normal = normals(i,:);
        radius = rand*(rdiff)+rmin;
        center = point-normal*radius;
        score = 0;
        for j=1:num_points
            dist = norm(points(j,:)-center);
            if (dist<(radius+delta)) && (dist>(radius-delta))
                score = score+1;
            end
        end
        if score>best_score
            best_center = center.';
            best_radius = radius;
            best_score = score;
        end
    end
end
