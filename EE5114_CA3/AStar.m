function [path, success] = AStar(Map, startNode, goalNode)
    openList = [];
    closedList = [];
    
    % Define the heuristic function 
    % Manhattan distance
    heuristic = @(node) abs(node(1) - goalNode(1)) + abs(node(2) - goalNode(2));
    
    % Add startNode to open list
    gScore = inf(size(Map));
    gScore(startNode(1), startNode(2)) = 0;
    fScore = inf(size(Map));
    fScore(startNode(1), startNode(2)) = heuristic(startNode);
    openList = [startNode, fScore(startNode(1), startNode(2))];
    
    % Parent map to reconstruct the path
    parentMap = containers.Map('KeyType', 'char', 'ValueType', 'any');
    
    % Neighbor offsets
    neighbors = [0, 1; 1, 0; 0, -1; -1, 0]; % Right, Down, Left, Up
    
    while ~isempty(openList)
        % Extract node with lowest fScore from the openList
        [~, idx] = min(openList(:, 3));
        current = openList(idx, 1:2);
        openList(idx, :) = [];
        
        % Check if goal is reached
        if isequal(current, goalNode)
            path = reconstructPath(parentMap, current);
            success = true;
            return;
        end
        
        % Add current to closed list
        closedList = [closedList; current];
        
        % Explore neighbors
        for i = 1:size(neighbors, 1)
            neighbor = current + neighbors(i, :);
            
            % Skip invalid neighbors
            if neighbor(1) < 1 || neighbor(2) < 1 || ...
                    neighbor(1) > size(Map, 1) || neighbor(2) > size(Map, 2)
                continue;
            end
            
            % Skip occupied or already visited nodes
            if Map(neighbor(1), neighbor(2)) || ismember(neighbor, closedList, 'rows')
                continue;
            end
            
            % Calculate tentative gScore
            tentativeGScore = gScore(current(1), current(2)) + 1;
            
            if tentativeGScore < gScore(neighbor(1), neighbor(2))
                % Update parent map
                parentMap(mat2str(neighbor)) = current;
                
                % Update scores
                gScore(neighbor(1), neighbor(2)) = tentativeGScore;
                fScore(neighbor(1), neighbor(2)) = tentativeGScore + heuristic(neighbor);
                
                % Add to open list if not already there
                if ~ismember(neighbor, openList(:, 1:2), 'rows')
                    openList = [openList; neighbor, fScore(neighbor(1), neighbor(2))];
                end
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
