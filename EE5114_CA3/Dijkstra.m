function [path, success] = Dijkstra(Map, startNode, goalNode)
    numRows = size(Map, 1);
    numCols = size(Map, 2);
    dist = inf(numRows, numCols);
    dist(startNode(1), startNode(2)) = 0;
    visited = false(numRows, numCols);
    parentMap = containers.Map('KeyType', 'char', 'ValueType', 'any');
    
    % Priority queue: [row, col, distance]
    priorityQueue = [startNode, 0]; 
    
    % Neighbor offsets
    neighbors = [0, 1; 1, 0; 0, -1; -1, 0]; % Right, Down, Left, Up
    
    while ~isempty(priorityQueue)
        % Sort the queue by distance (priority)
        [~, idx] = min(priorityQueue(:, 3));
        current = priorityQueue(idx, 1:2);
        currentDist = priorityQueue(idx, 3);
        priorityQueue(idx, :) = [];
        
        % Mark current node as visited
        if visited(current(1), current(2))
            continue;
        end
        visited(current(1), current(2)) = true;
        
        % Check if goal is reached
        if isequal(current, goalNode)
            path = reconstructPath(parentMap, current);
            success = true;
            return;
        end
        
        % Explore neighbors
        for i = 1:size(neighbors, 1)
            neighbor = current + neighbors(i, :);
            
            % Skip invalid neighbors
            if neighbor(1) < 1 || neighbor(2) < 1 || ...
                    neighbor(1) > numRows || neighbor(2) > numCols
                continue;
            end
            
            % Skip occupied or visited nodes
            if Map(neighbor(1), neighbor(2)) || visited(neighbor(1), neighbor(2))
                continue;
            end
            
            % Calculate tentative distance
            tentativeDist = currentDist + 1;
            
            if tentativeDist < dist(neighbor(1), neighbor(2))
                % Update distance
                dist(neighbor(1), neighbor(2)) = tentativeDist;
                
                % Update parent map
                parentMap(mat2str(neighbor)) = current;
                
                % Add to priority queue
                priorityQueue = [priorityQueue; neighbor, tentativeDist];
            end
        end
    end
    
    % No path found
    path = [];
    success = false;
end

function path = reconstructPath(parentMap, current)
    path = current;
    while isKey(parentMap, mat2str(current))
        current = parentMap(mat2str(current));
        path = [current; path];
    end
end
