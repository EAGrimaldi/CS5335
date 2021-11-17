% Input: cspace -> NxN matrix: cspace(i,j)
%                   == 1 if [q_grid(i); q_grid(j)] is in collision,
%                   == 0 otherwise
%        q_grid -> 1xN vector of angles between 0 and 2*pi, discretizing
%                  each dimension of configuration space
%        q_goal -> 2x1 vector denoting the goal configuration
% Output: distances -> NxN matrix containing the distance transform from
%                      the goal configuration
%                      == 0 if cell is unreachable
%                      == 1 if cell is an obstacle
%                      == 2 if cell is the goal
%                      >  2 otherwise

function distances = C3(cspace, q_grid, q_goal)
    [~, res] = size(cspace);
    distances = zeros(res, res);
    goal = [dsearchn(q_grid.', q_goal(1,1)); dsearchn(q_grid.', q_goal(2,1))];
    distances(goal(1, 1), goal(2, 1)) = 2;
    edges = [goal]; % an array of 2x1 arrays
    new_edges = [];
    while ~isempty(edges)
        for edge = edges
            x = edge(1); y = edge(2); % for convenient typing
            neighbors = [x-1 x-1 x-1 x x x+1 x+1 x+1; y-1 y y+1 y-1 y+1 y-1 y y+1];
            for n=1:8
                dx = neighbors(1,n); dy = neighbors(2,n); % for convenient typing
                if ~(dx<1 || dy<1 || dx>res || dy>res)                    
                    if cspace(dx,dy)==1
                        distances(dx,dy)=1;
                    elseif distances(dx,dy)==0
                        new_edges = cat(2, new_edges, [dx; dy]);
                        distances(dx,dy)=distances(x,y)+1;
                    end
                end
            end
        end
        edges = new_edges;
        new_edges = [];
    end
end