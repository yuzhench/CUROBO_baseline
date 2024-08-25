
function draw_obstacles(obstacleData, ax)
    axes(ax);
    hold on;
    axis equal;
    grid on;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    view(3);  % Set view to 3D
    
    obstacleNames = fieldnames(obstacleData);  % Get all obstacle names
    
    for i = 1:length(obstacleNames)
        obstacle = obstacleData.(obstacleNames{i});
        dims = obstacle.dims;  % Already in numeric form
        pose = obstacle.pose;  % Already in numeric form

        x = pose(1);
        y = pose(2);
        z = pose(3);
        dx = dims(1);
        dy = dims(2);
        dz = dims(3);

        % Calculate vertices of the cuboid
        X = [x-dx/2, x+dx/2, x+dx/2, x-dx/2, x-dx/2, x+dx/2, x+dx/2, x-dx/2];
        Y = [y-dy/2, y-dy/2, y+dy/2, y+dy/2, y-dy/2, y-dy/2, y+dy/2, y+dy/2];
        Z = [z-dz/2, z-dz/2, z-dz/2, z-dz/2, z+dz/2, z+dz/2, z+dz/2, z+dz/2];

        % Draw the cuboid
        for j = 1:4
            % Vertical edges
            plot3([X(j), X(j+4)], [Y(j), Y(j+4)], [Z(j), Z(j+4)], 'k-');
            % Top face
            k = j + 4;  % Index offset for top vertices
            if j < 4
                % Horizontal edges (bottom and top)
                plot3([X(j), X(j+1)], [Y(j), Y(j+1)], [Z(j), Z(j+1)], 'k-');  % Bottom face
                plot3([X(k), X(k+1)], [Y(k), Y(k+1)], [Z(k), Z(k+1)], 'k-');  % Top face
            else
                % Closing the loop on the faces
                plot3([X(j), X(1)], [Y(j), Y(1)], [Z(j), Z(1)], 'k-');  % Bottom face
                plot3([X(k), X(5)], [Y(k), Y(5)], [Z(k), Z(5)], 'k-');  % Top face
            end
        end
    end
    % 
    % hold off;
end


 
% draw_obstacles(obstacleData);

 



% grid off; axis equal; hold on;

% % Visualize the robot configurations
% for i = 1:size(configurations, 1)
%     show(robot, configurations(i, :)', 'PreservePlot', false);
%     draw_obstacles(obstacleData);
%     pause(0.02); % Adjust the pause for animation speed
%     fprintf("%.2d",i);
% end


close all;
clear;
clc;

% 初始化图形环境
fig = figure;
ax = axes('Parent', fig);

hold on;
axis equal;
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
view(3);  % 3D视角

% 绘制障碍物

jsonData = fileread('/home/yuzhen/Desktop/CROWS_paper_baselines/csv_environment_sources/json_result/json_version_sparrow_world2');  % Provide the correct path
obstacles = jsondecode(jsonData);
obstacleData = obstacles.cuboid;
 
% 导入机械臂模型
% Load the configurations from the .mat file
data = load('examples/matlab_mat_result/curobo_trajectory2.mat');  % replace 'path_to_your_mat_file.mat' with the actual path
% Import the robot model from a URDF file
robot = importrobot('/home/yuzhen/Desktop/CROWS_paper_baselines/curobo/src/curobo/content/assets/robot/kinova-gen3/kinova.urdf'); % Replace 'your_urdf.urdf' with your actual URDF file name
robot.DataFormat = 'column';



draw_obstacles(obstacleData, ax);

% % Assuming 'configurations' is the variable in your .mat file that contains joint configurations
configurations = double(data.q);

% 动画显示机械臂的移动

% Visualize the robot configurations
for i = 1:size(configurations, 1)
    show(robot, configurations(i, :)', 'Parent', ax, 'PreservePlot', false);
    % drawnow;
    pause(0.02); % Adjust the pause for animation speed
    fprintf("%.2d",i);
end

hold off;