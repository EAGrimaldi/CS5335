% Input: distances -> NxN matrix containing the distance transform from
%                      the goal configuration
%                      == 0 if cell is unreachable
%                      == 1 if cell is an obstacle
%                      == 2 if cell is the goal
%                      >  2 otherwise
%        q_grid -> 1xN vector of angles between 0 and 2*pi, discretizing
%                  each dimension of configuration space
%        q_start -> 2x1 vector denoting the start configuration
% Output: path -> Mx2 matrix containing a collision-free path from q_start
%                 to q_goal (as computed in C3, embedded in distances).
%                 The entries of path should be grid cell indices, i.e.,
%                 integers between 1 and N. The first row should be the
%                 grid cell containing q_start, the final row should be
%                 the grid cell containing q_goal.

function path = C4(distances, q_grid, q_start)
    [~, res] = size(distances);
    start = [dsearchn(q_grid.', q_start(1,1)) dsearchn(q_grid.', q_start(2,1))];
    path = [start];
    edge = start;
    while distances(edge(1), edge(2))>2
        x = edge(1); y = edge(2); % for convenient typing
        neighbors = [x-1 x-1 x-1 x x x+1 x+1 x+1; y-1 y y+1 y-1 y+1 y-1 y y+1];
        for n=1:8
            dx = neighbors(1,n); dy = neighbors(2,n); % for convenient typing
            if ~(dx<1 || dy<1 || dx>res || dy>res) && distances(dx,dy)~=1 && distances(dx,dy)<distances(edge(1),edge(2))
                edge = [dx dy];
            end
        end
        path = cat(1, path, edge);
    end
end