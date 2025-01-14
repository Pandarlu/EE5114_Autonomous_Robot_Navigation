function Main()
    % Load the map
    load('Map.mat', 'Map');
    
    % Ensure the map is logical
    if ~islogical(Map)
        Map = logical(Map);
    end
    
    % PART 1: Fixed Start and Goal Points
    startNodeFixed = [1, 1];
    goalNodeFixed = [10, 10];
    fprintf('\n--- Fixed Points ---\n');
    fprintf('Start: (%d, %d), Goal: (%d, %d)\n', startNodeFixed(1), startNodeFixed(2), goalNodeFixed(1), goalNodeFixed(2));
    
    % Run A* and Dijkstra for fixed points
    [pathAStarFixed, successAStarFixed] = AStar(Map, startNodeFixed, goalNodeFixed);
    [pathDijkstraFixed, successDijkstraFixed] = Dijkstra(Map, startNodeFixed, goalNodeFixed);
    
    % Visualize fixed point results
    if successAStarFixed
        visualizePath(Map, pathAStarFixed, 'A* Algorithm (Fixed Points)', 1);
    else
        fprintf('Failure: A* could not find a path.\n');
    end
    
    if successDijkstraFixed
        visualizePath(Map, pathDijkstraFixed, 'Dijkstra Algorithm (Fixed Points)', 2);
    else
        fprintf('Failure: Dijkstra could not find a path.\n');
    end
    
    % PART 2: Random Start and Goal Points
    [startNodeRandom, goalNodeRandom] = generateRandomPoints(Map);
    fprintf('\n--- Random Points ---\n');
    fprintf('Start: (%d, %d), Goal: (%d, %d)\n', startNodeRandom(1), startNodeRandom(2), goalNodeRandom(1), goalNodeRandom(2));
    
    % Run A* and Dijkstra for random points
    [pathAStarRandom, successAStarRandom] = AStar(Map, startNodeRandom, goalNodeRandom);
    [pathDijkstraRandom, successDijkstraRandom] = Dijkstra(Map, startNodeRandom, goalNodeRandom);
    
    % Visualize random point results
    if successAStarRandom
        visualizePath(Map, pathAStarRandom, 'A* Algorithm (Random Points)', 3);
    else
        fprintf('Failure: A* could not find a path.\n');
    end
    
    if successDijkstraRandom
        visualizePath(Map, pathDijkstraRandom, 'Dijkstra Algorithm (Random Points)', 4);
    else
        fprintf('Failure: Dijkstra could not find a path.\n');
    end
end

function [startNode, goalNode] = generateRandomPoints(Map)
    % Generate random start and goal points that are not occupied
    freeCells = find(~Map);
    if numel(freeCells) < 2
        error('Failure: Random points could not be generated.\n');
    end
    
    % Randomly select two distinct free cells using randperm
    selectedIndices = randperm(numel(freeCells), 2);
    [startRow, startCol] = ind2sub(size(Map), freeCells(selectedIndices(1)));
    [goalRow, goalCol] = ind2sub(size(Map), freeCells(selectedIndices(2)));
    
    % Assign the start and goal nodes
    startNode = [startRow, startCol];
    goalNode = [goalRow, goalCol];
end

function visualizePath(Map, path, titleText, figureID)
    figure(figureID);
    imagesc(~Map); 
    colormap(gray);
    hold on;
    plot(path(:, 2), path(:, 1), 'r', 'LineWidth', 2);
    plot(path(1, 2), path(1, 1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g'); 
    plot(path(end, 2), path(end, 1), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
    title(titleText);
    hold off;
end
