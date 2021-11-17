function p = mkgrid(N, s, varargin)
    
    opt.pose = [];


    [opt,args] = tb_optparse(opt, varargin);
    
    if length(args) > 0 && ishomog(args{1})
        % compatible with legacy call
        opt.pose = args{1};
    end
    if length(s) == 1
        sx = s; sy = s;
    else
        sx = s(1); sy = s(2);
    end

    if length(N) == 1
        nx = N; ny = N;
    else
        nx = N(1); ny = N(2);
    end


    if N == 2
        % special case, we want the points in specific order
        p = [-sx -sy 0
             -sx  sy 0
              sx  sy 0
              sx -sy 0]'/2;
    else
        [X, Y] = meshgrid(1:nx, 1:ny);
        X = ( (X-1) / (nx-1) - 0.5 ) * sx;
        Y = ( (Y-1) / (ny-1) - 0.5 ) * sy;
        Z = zeros(size(X));
        p = [X(:) Y(:) Z(:)]';
    end
    
    % optionally transform the points
    if ~isempty(opt.pose)
        p = SE3.convert(opt.pose) * p;
    end