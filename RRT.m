%% Clearing the command window and workspace
clc
clear

%% Defining obstacles
obstacles = readmatrix('obstacles.csv');
centerX = obstacles(:,1);
centerY = obstacles(:,2);
radius = obstacles(:,3)/2;

obstacleCenterX = zeros(1, length(obstacles));
obstacleCenterY = zeros(1, length(obstacles));
obstacleRadius = zeros(1, length(obstacles));

for i = 1 : length(obstacles)
    obstacleCenterX(i) = centerX(i);
    obstacleCenterY(i) = centerY(i);
    obstacleRadius(i) = radius(i);
end

%% Setting up the PRM Aims and Parameters:
startNode = [-0.5 -0.5];
endNode = [0.5 0.5];
maxTreeSize = 1000;

T{1} = startNode;
rangeX = startNode(1) : 0.01 : endNode(1);
rangeY = startNode(2) : 0.01 : endNode(2);
nodeNumber = 1;
edge = [];

%% Starting the RRT Algorithm:
while length(T) < maxTreeSize
    
    if(length(rangeX) ~= 0)
    xAxis = randsample(rangeX*0.1, 1);
    end
    
    if(length(rangeY) ~= 0)
    yAxis = randsample(rangeY*0.1, 1);   
    end
    
    xSamp = [xAxis yAxis];
    
    for i = 1 : length(T)
        distances = sqrt((T{1}(1) - xAxis)^2 + (T{1}(2) - yAxis)^2);
        index = find(distances == min(distances));
        xNearest = T{i};
    end
    
    dRange = 0 : 0.005: 0.1;
    distanceX = randsample(dRange, 1);
    distanceY = randsample(dRange, 1);
    
    xNewAxisX = xNearest(1) + distanceX;
    xNewAxisY = xNearest(2) + distanceY;
    xNew = [xNewAxisX xNewAxisY];
    
    if (xNew(1) - xNearest(1) == 0)
        continue
    end
    
    lineCoeffs = polyfit([xNearest(1) xNew(1)], [xNearest(2) xNew(2)], 1);
    slope = lineCoeffs(1);
    yIntercept = lineCoeffs(2);
    
    % Checking if the line intersects any of the obstacles:
    for i = 1 : length(obstacles)
        a = linecirc(slope,yIntercept,obstacleCenterX(i),obstacleCenterY(i),obstacleRadius(i));
        % If a is not a NaN array (1 by 2), this means collision and loop
        % is terminated:
        if (~isnan(a))
            addCondition = 0;
            break
        else
            addCondition = 1;
        end
    
    end
    
    if xNew(1)>= 0.5
        xNew(1) = 0.5;
    end
    
    if xNew(2)>= 0.5
        xNew(2) = 0.5;
    end
    
    % If the line does not intersect the obstacles, xNew is added to the
    % tree and vertex/edge is created between xNearest and xNew:
    if (addCondition ~= 0)
        T{length(T) + 1} = xNew;
        nodeDistance = sqrt((xNearest(1) - xNew(1))^2 + (xNearest(2) - xNew(2))^2);
        edge{length(edge) + 1} = [nodeNumber nodeNumber+1 nodeDistance];
        nodeNumber = nodeNumber + 1;
        % Narrowing down the range of sample in order to get closer to the
        % goal node:
        narrowRangeX = rangeX < xNew(1);
        narrowRangeY = rangeY < xNew(2);
        rangeX(narrowRangeX) = [];
        rangeY(narrowRangeY) = [];
        
        if xNew == endNode
            disp('RRT is completed successfully!')
            for j = 1 : length(T)
                nodes(j, :) = [j T{j}];
            end
            
            for j = 1 : length(edge)
                edges(j, :) = edge{j};
            end           
            
            writematrix(edges, 'edges.csv');
            writematrix(nodes, 'nodes.csv');
            writematrix(nodes(:,1)', 'path.csv')
            return
        end
    end
    
end
disp('RRT is not completed successfully!')    