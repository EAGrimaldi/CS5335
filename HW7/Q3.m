% Localize a cylinder in the point cloud. Given a point cloud as input, this
% function should locate the position and orientation, and radius of the
% cylinder.
% input: ptCloud -> a pointCloud object that contains the point cloud (see
%                   Matlab documentation)
% output: center -> 3x1 vector denoting cylinder center
%         axis -> 3x1 unit vector pointing along cylinder axis
%         radius -> scalar radius of cylinder
function [best_center,best_axis,best_radius] = Q3(ptCloud)
    rmin = 0.05;
    rmax = 0.10;
    rdiff = rmax-rmin;
    normals = pcnormals(ptCloud);
    points = ptCloud.Location;
    num_points = size(points,1);
    delta = 0.0002;
    best_center = zeros(3,1);
    best_axis = zeros(3,1);
    best_radius = 0;
    best_score = 0;
    num_samples = 50000;
    for i=1:num_samples
        [sample_points, sample_index] = datasample(points,2,1,"Replace",false);
        axis = (cross(normals(sample_index(1),:), normals(sample_index(2),:))).';
        radius = rand*(rdiff)+rmin;
        center = (sample_points(1,:)-normals(sample_index(1),:)*radius).';
        proj = eye(3)-axis*axis.';
        proj_center = proj*center;
        proj_points = proj*points.';
        score = 0;
        for j=1:num_points
            dist = norm(proj_points(:,j)-proj_center);
            if (dist<(radius+delta)) && (dist>(radius-delta))
                score = score+1;
            end
        end
        if score>best_score
            best_center = center;
            best_axis = axis;
            best_radius = radius;
            best_score = score;
        end
    end
end