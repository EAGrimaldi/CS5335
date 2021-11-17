% Fit a plane using RANSAC
% input        P ->  3x100 matrix denoting 100 points in 3D space
% output: normal -> 1x3 vector denoting surface normal of the fitting plane
%         center -> 1x3 vector denoting center of the points
function [best_normal,best_center] = Q1_c(P)
    num_points = size(P,2);
    num_samples = 100;
    delta = 0.05;
    best_normal = zeros(1,3);
    best_center = zeros(1,3);
    best_score = 0;
    for i=1:num_samples
        choice = datasample(P,3,2,"Replace",false);
        % choose 3 columns from P uniformly randomly without replacement
        [normal, center] = Q1_a(choice);
        score = 0;
        for j=1:num_points
            dist = abs(dot((P(:,j).'-center),normal));
            if dist<delta
                score = score+1;
            end
        end
        if score>best_score
            best_normal = normal;
            best_center = center;
            best_score = score;
        end
    end
end
