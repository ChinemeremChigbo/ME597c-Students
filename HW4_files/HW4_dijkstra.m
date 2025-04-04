%%%% Code for ME597c - HW3
%%%% Complete the code where indicated
%%%% The expected outcome is: given an occupancy map, the shortest path from given start to given goal
clear;
close all;

% Load the given occupancy map
load occupancy_map.mat
disp(["Size of the map: ", size(omap)]);
disp(["Number of cells: ", numel(omap)]);

% Coordinates of the starting cell (try to set different starts and goals and see what happens)
start = [40,15];
% Coordinates of the goal cell 
goal = [21,37];
% Plot the occupancy map with the start and goal cells
plot_map(omap, start, goal)

% Matrix to store the cost values for each cell
% Initialize with infinity
costs = ones(size(omap)) * Inf;

% Matrix to store cells that have already been visited -> closed set
% Initialize with zeros (1 means that the cell is part of the closed list)
closed = zeros(size(omap));

% Cell array to store the coordinates and cost of priors for each visited cell
priors = cell(size(omap) + [2,2]);
% Each element of the cell array element is defined as: [x,y,cost]
% Initialize coordinates to Inf and costs to -1
priors(:,:) = {[Inf,Inf,-1]};

% Initial condition: first prior_node is set to be the starting cell, with cost set to zero
prior_node = start;
costs(start(1), start(2)) = 0;
while ~isequal(prior_node, goal)
    % Initialize open_cost as costs
    open_costs = costs;
    % Find all those cells that have already been expanded (in the closed set) and set their cost to Inf
    open_costs(find(closed==1)) = Inf;

    % Find the cell with the minimum cost in the open list
    [mx,my] = find(open_costs==min(min(open_costs)));
    mincost = ind2sub(size(open_costs), [mx ,my]);
    % Take the first found
    x = mincost(1,1);
    y = mincost(1,2);

    % Break the loop if there are no more open cells
    if open_costs(x,y) == Inf
      break
    end

    % Set the minimum cost cell as prior_node and assign it in the closed list
    prior_node = [x,y];
    closed(x,y) = 1;

    % Update costs and prior nodes for the neighbors
    neighbors = {};
    %%%%%%% Complete code here (c) %%%%%%%%%
    
    % Visualize the cells that have already been expanded
    plot_expanded(prior_node, start, goal)
end

% When done, rewind the path from goal to start
if isequal(prior_node, goal)
    disp(["N. of cells expanded : ", nnz(closed)])
    disp(["Cost of the path : ", costs(goal(1), goal(2))])
    path_length = 0;
    while priors{prior_node(1), prior_node(2)}(3) >= 0
      plot_path(prior_node, goal)
      previous = priors{prior_node(1), prior_node(2)}(1:2);
      path_length = path_length + norm(prior_node - previous);
      prior_node = previous;
    end
    disp(["Length of the path: ", path_length])
else
    disp("No feasible path could be found.")
end

% Visualize the costs as a colored map
plot_costs(costs)

%   Find all the neighbors of the given cell.
%   Arguments:
%   current_cell: coordinates of a cell as [x, y]
%   omap_size: size of the occupancy map (nx, ny)
% 
%   Output:
%   neighbors: list of up to eight neighbor coordinate as cell array {(x1, y1), (x2, y2), ...}
function neighbors = get_neighbors(current_cell, omap_size)
    neighbors = {};
    %%%%%%% Complete code here (a) %%%%%%%%%
end

%   Calculate the cost to move from prior_node to current_node.
%   Arguments:
%   prior_node, current_node: coordinates of a cell as [x, y]
%   omap: occupancy map
% 
%   Output:
%   edge_cost: calculated cost
function edge_cost = get_edge_cost(prior_node, current_node, omap)  
    %%%%%%% Complete code here (b) %%%%%%%%%
end

%%%%%% Plotting functions %%%%%%
function plot_map(omap, start, goal)
  imshow(omap', 'InitialMagnification', 1000)
  colorbar
  hold on
  plot(start(1), start(2), 'gp');
  plot(goal(1), goal(2), 'rp');
  xlabel('x')
  ylabel('y')
end

function plot_expanded(expanded, start, goal)
  if isequal(expanded, start) || isequal(expanded, goal)
    return
  end
  plot(expanded(1), expanded(2), 'yx');
  pause(1e-6)
end

function plot_path(path, goal)
  if isequal(path, goal)
    return
  end
  plot(path(1), path(2), 'bx')
  pause(1e-6)
end
  
function plot_costs(cost)
  figure;
  imagesc(cost')
  colorbar
  xlabel('x')
  ylabel('y')
end
