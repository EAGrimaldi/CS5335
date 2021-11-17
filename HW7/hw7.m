% install MVTB-4.3 https://petercorke.com/toolboxes/machine-vision-toolbox/
function hw7(questionNum)
    % ************* Question 1.a *******************
    if questionNum == 1
        close all;
        T = SE3(1,2,3) * SE3.rpy(0.3,0.4,0.5);
        P = mkgrid(10,1,'pose',T);
        P = P + 0.02*randn(size(P));
        scatter3(transpose(P(1,:)),transpose(P(2,:)),transpose(P(3,:)),'x','r');
        hold on;
        [normal,center] = Q1_a(P);
        plot_plane(normal,center);
   
    end
    % ************* Question 1.b *******************
    if questionNum == 2
        close all;
        T = SE3(1,2,3) * SE3.rpy(0.3,0.4,0.5);
        P = mkgrid(10,1,'pose',T);
        P = P + 0.02*randn(size(P));
        column_index = [1,2,3,4,5,6,7,8];
        P(:,column_index) = P(:,column_index)+ 1*randn(3,8);
        scatter3(transpose(P(1,:)),transpose(P(2,:)),transpose(P(3,:)),'x','r');
        hold on;
        [normal,center] = Q1_a(P);
        plot_plane(normal,center);
    end
    % ************* Question 1.c *******************
    if questionNum == 3
        close all;
        T = SE3(1,2,3) * SE3.rpy(0.3,0.4,0.5);
        P = mkgrid(10,1,'pose',T);
        P = P + 0.02*randn(size(P));
        column_index = [1,2,3,4,5,6,7,8];
        P(:,column_index) = P(:,column_index)+ 1*randn(3,8);
        scatter3(transpose(P(1,:)),transpose(P(2,:)),transpose(P(3,:)),'x','r');
        hold on;
        [normal,center] = Q1_c(P);
        plot_plane(normal,center);
    end

     
    % ************* Question 2 *******************
    if questionNum == 4
        close all;

        % load point cloud
        load('object3d.mat');
        ptCloudOrig = ptCloud;
        % approximately segment object from background
        % roi = [-inf,0.5,0.2,0.4,0.1,inf]; % segment sphere
        roi = [-inf,inf,-inf,inf,-inf,inf]; % no segmentation 
        indices = findPointsInROI(ptCloud,roi);
        ptCloudB = select(ptCloudOrig,indices);
        
        % locate sphere
        [center,radius] = Q2(ptCloudB);

        % display cloud
        figure;
        pcshow(ptCloudOrig);
        hold on;

        % plot sphere
        [X,Y,Z] = sphere;
        X = X * radius + center(1);
        Y = Y * radius + center(2);
        Z = Z * radius + center(3);
        surf(X,Y,Z);        
    end
    
    % ************* Question 3 *******************
    if questionNum == 5
        close all;
        % load point cloud
        load('object3d.mat');
        ptCloudOrig = ptCloud;        
        % approximately segment object from background
        roi = [0.4,0.6,-inf,0.2,0.1,inf]; % cylinder
        indices = findPointsInROI(ptCloud,roi);
        ptCloudB = select(ptCloudOrig,indices);

        % display cloud
        figure;
        pcshow(ptCloudOrig);
        hold on;

        % locate cylinder
        [center,axis,radius] = Q3(ptCloudB);
        axis = axis / norm(axis); % normalize just to be sure...
        
        % plot cylinder
        nn = (eye(3) - axis * axis') * [1;0;0]; % get an arbitrary vector orthogonal to axis
        nn = nn / norm(nn);
        R = [cross(nn,axis) nn axis]; % create rotation matrix
        [X,Y,Z] = cylinder;
        Z=Z*5 - 2.5; % lengthen cylinder
        m = size(X,1)*size(X,2);
        rotXYZ = R * ([reshape(X,1,m); reshape(Y,1,m); reshape(Z,1,m)]*radius) + repmat(center,1,m); % rotate cylinder and add offset
        surf(reshape(rotXYZ(1,:),2,m/2),reshape(rotXYZ(2,:),2,m/2),reshape(rotXYZ(3,:),2,m/2)); % plot it

    end

end



function plot_plane(normal,center)
   w = null(normal); % Find two orthonormal vectors which are orthogonal to v
   [P,Q] = meshgrid(-0.5:0.5); % Provide a gridwork (you choose the size)
   X = center(1,1)+w(1,1)*P+w(1,2)*Q; % Compute the corresponding cartesian coordinates
   Y = center(1,2)+w(2,1)*P+w(2,2)*Q; %   using the two vectors in w
   Z = center(1,3)+w(3,1)*P+w(3,2)*Q;
   surf(X,Y,Z)  
end

