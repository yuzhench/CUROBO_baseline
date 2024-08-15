close all; clear; clc;

% Load the configurations from the .mat file
data = load('examples/curobo_trajectory.mat');  % replace 'path_to_your_mat_file.mat' with the actual path

% Import the robot model from a URDF file
robot = importrobot('/home/yuzhen/Desktop/CROWS_paper_baselines/curobo/src/curobo/content/assets/robot/kinova-gen3/kinova.urdf'); % Replace 'your_urdf.urdf' with your actual URDF file name

 

robot.DataFormat = 'column';


 


 


 

% Assuming 'configurations' is the variable in your .mat file that contains joint configurations
configurations = double(data.q);

% grid off; axis equal; hold on;

% Visualize the robot configurations
for i = 1:size(configurations, 1)
    show(robot, configurations(i, :)', 'PreservePlot', false);
    pause(0.02); % Adjust the pause for animation speed
    fprintf("%.2d",i);
end

 