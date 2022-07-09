function [route] = greedy2D(map, start, end_, length_cost)
    
    % Check if length cost was given
    if ~exist('length_cost', 'var')
        length_cost = 1;
    end
    
    % Define the limits of map
    max_x = length(map(:,1,1));
    max_y = length(map(1,:,1));

    first_closed = 1;
    first_children = 1;
    closed = [];
    children = [];

    % create the first node at the start position
    parent_node = mynode;
    parent_node.position = start;
    parent_node.h = parent_node.calc_dist(end_);
    parent_node.f = parent_node.h;

    % Flag used to skip nodes which is already added
    continue_flag = 0;

    % Slow the calculation down, so it can be followed in real time
    pause on;
    
    % Keep running until the end point is reached
    while ~(parent_node.position(1) == end_(1) && ...
            parent_node.position(2) == end_(2))
       % Run through the surrouding squares(from one specific node)
       for x= -1:1
           for y = -1:1
               % skip the node itself
               % Don't allow diagonal movement
               if ~((x == 0 && y == 0) ||...
                       (abs(x) + abs(y) > 1))
                   % get the temp child node
                   node_pos = [parent_node.position(1) + x, parent_node.position(2) + y];
    
                   % Check if the children is in the map
                   if ~(node_pos(1) < 1 || node_pos(1) > max_x || node_pos(2) < 1 || node_pos(2) > max_y)
                       % Check if the children is an obstacle
                       if ~(map(node_pos(1), node_pos(2)) == 1)
                           % Check if the children has been visited
                           for closed_i = 1:length(closed)
                               if node_pos == closed(closed_i).position
                                   continue_flag = 1;
                               end
                           end
                           % Check if the node is already a child
                           for child_i = 1:length(children)
                               if node_pos == children(child_i).position
                                   continue_flag = 1;
                               end
                           end
                           % Check if the node should be skipped
                           if continue_flag == 1
                               continue_flag = 0;
                               continue
                           end
                           
                           % Define the child node
                           temp_node = mynode;
                           % Note its parent
                           % (when this child node is
                           % selected as the next parent node,
                           % this will be its parent node information
                           % for trakcing the route)
                           temp_node.parent = parent_node;
                           % Note its position
                           temp_node.position = node_pos;
                           % Calculate the distance from the node to the
                           % end point
                           temp_node.h = temp_node.calc_dist(end_);
                           % Calculate the total cost of the node
                           temp_node.f = temp_node.h;
    
                           % Add the node to the children array
                           % Check if it is the first child being added
                           if first_children == 1
                               first_children = 0;
                               children = [temp_node];
                           else
                               % Otherwise expand the children array
                               children(end+1) = temp_node;
                           end
                        end
                    end
                end
            end
       end

       % Add the parent node to the list of closed nodes
       if first_closed == 1
           first_closed = 0;
           closed = [parent_node];
       else
           closed(end+1) = parent_node;
       end
       
       % choose the child node with the lowest f value
       lowest_f = 999999;
       lowest_child_i = -1;
       for child_i = 1:length(children)
           if children(child_i).f < lowest_f
               lowest_f = children(child_i).f;
               lowest_child_i = child_i;
           end
       end  
       
       % Check if there still is routes available
       if length(children) == 0
           route = NaN;
           return
       end
       
        % update the parent to the children
        parent_node = children(lowest_child_i);
        % delete the selected child node from children
        children(lowest_child_i) = [];

    end
    
    % Find the route that the algorithm took
    % Init the route array
    route = [parent_node.position];

    % Keep going until the route is back at the start position
    while ~(parent_node.position(1) == start(1) && parent_node.position(2) == start(2))
        % Update the route by going backwards through the parents
        parent_node = parent_node.parent;
        route = cat(1, route, parent_node.position);
    end
    route = flip(route);
end
