% Fit a least squares plane by taking the Eigen values and vectors of the
% sample covariance matrix.
% input: P       ->  3x100 matrix denoting 100 points in 3D space
% output: normal -> 1x3 vector denoting surface normal of the fitting plane
%         center -> 1x3 vector denoting center of the points
function [normal,center] = Q1_a(P)
    center = mean(P.');
    covp = cov(P.');
    [eigvec, eigval] = eig(covp);
    [minval, minind] = min([eigval(1,1) eigval(2,2) eigval(3,3)]);
    normal = eigvec(:,minind).';
end
