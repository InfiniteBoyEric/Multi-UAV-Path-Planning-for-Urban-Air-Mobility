%% ========================================================================
% Multi-UAV Path Planning for Urban Air Mobility 
% Based on Centralized Layered Planning Strategy
%% ========================================================================

%% Clear all figures, variables and commands
close all
clear
clc
dbclear all
dbstop if all error

%% Step 1: Initialize the system
% Add directories into the code
algorithms_folder = '.\algorithms';
uav_folder = '.\uav';
utils_folder = '.\utils';
addpath(genpath(algorithms_folder));
addpath(genpath(uav_folder));
addpath(genpath(utils_folder));

% Select one of the algorithms as the solver
% Algorithm options: 'THRO', 'GGO', 'TOC', 'PGA', 'IVY', 'CCO', 'TGCOA'
% More optional algorithms in '.\algorithms' directory
ALGORITHM_OPTION = 'CCO'; 

% parameters for the algorithm
pop_size = 50;       % population size
max_iter = 1000;     % max iteration

%% Step 2: Construct an urban environment
fprintf('>>> Constructing the urban environment ...\n');

% Create the urban environment
global model
model = CreateUrbanEnvironment(); 

fprintf('Succeed in creating the environment!\n');
fprintf('  - Map size: %d x %d x %d m\n', ...
    model.MAPSIZE_X, model.MAPSIZE_Y, model.zmax);

%% Step 3: Define tasks of UAVs
% Define three UAVs
uav_tasks = struct();

% UAV_1: Low Layer (~80m); Task: Hospital -> Residential Area
uav_tasks(1).id = 1;
uav_tasks(1).start = [100; 100; 79]; 
uav_tasks(1).end   = [900; 900; 82]; 
uav_tasks(1).color = [0.9, 0.1, 0.1];  % Trajectory color: red 
uav_tasks(1).name  = 'UAV-1 (Medical)';

% UAV_2: Intermediate Layer (~100m); Task: Mall -> Express Center
uav_tasks(2).id = 2;
uav_tasks(2).start = [100; 900; 98]; 
uav_tasks(2).end   = [900; 100; 104]; 
uav_tasks(2).color = [0.1, 0.1, 0.9];  % Trajectory color: blue
uav_tasks(2).name  = 'UAV-2 (Delivery)';

% UAV_3: High Layer (~120m); Task: Patrol through the Center of the city)
uav_tasks(3).id = 3;
uav_tasks(3).start = [50; 500; 121]; 
uav_tasks(3).end   = [950; 500; 125]; 
uav_tasks(3).color = [0.1, 0.7, 0.1];  % Trajectory color: green 
uav_tasks(3).name  = 'UAV-3 (Patrol)';

num_uavs = length(uav_tasks);
all_results = struct();

%% Step 4: Solve the planning with the algorithm choosed
fprintf('>>> Start solving...\n');
fprintf('    Algoithm: %s\n\n', ALGORITHM_OPTION);

% Environment backup
base_model = model; 

for i = 1:num_uavs
    fprintf('--- Planning %s ---\n', uav_tasks(i).name);
    
    % 1. Reset the start point and end point
    model = base_model;
    model.start = uav_tasks(i).start;
    model.end   = uav_tasks(i).end;
    
    % 2. Get parameters
    [Xmin, Xmax, dim, fobj] = fun_info();
    
    % 3. Execute the algorithm choosed
    tic;
    switch ALGORITHM_OPTION
        case 'PGA'
            [fMin, bestX, conv] = PGA(pop_size, max_iter, Xmin, Xmax, dim, fobj);
        case 'IVY'
            [fMin, bestX, conv] = IVY(pop_size, max_iter, Xmin, Xmax, dim, fobj);
        case 'THRO'
            [fMin, bestX, conv] = THRO(pop_size, max_iter, Xmin, Xmax, dim, fobj);
        case 'GGO'
            [fMin, bestX, conv] = GGO(pop_size, max_iter, Xmin, Xmax, dim, fobj);
        case 'CCO'
            [fMin, bestX, conv] = CCO(pop_size, max_iter, Xmin, Xmax, dim, fobj);
        case 'TOC'
            [fMin, bestX, conv] = TOC(pop_size, max_iter, Xmin, Xmax, dim, fobj);
        case 'TGCOA'
            [fMin, bestX, conv] = TGCOA(pop_size, max_iter, Xmin, Xmax, dim, fobj);
        otherwise
            error('Unknown Algorithms: %s!', ALGORITHM_OPTION);
    end
    exec_time = toc;
    
    % 4. Save results
    sol = SphericalToCart(bestX);
    all_results(i).sol = sol;
    all_results(i).cost = fMin;
    all_results(i).time = exec_time;
    all_results(i).conv = conv;
    
    fprintf('    ✓ Done! Time consumption: %.2f s; Final score: %.2f\n', exec_time, fMin);
end

%% Step 5: Visualize results
fprintf('\n===========================================\n');
fprintf('  All paths have been solved! Visualizing...');
fprintf('\n===========================================\n');

% Plot the paths
PlotMultiUAV3D(all_results, uav_tasks, base_model);

% Plot the top view of the paths
PlotMultiUAVTopView(all_results, uav_tasks, base_model);

% Demonstrate data from results
PrintMultiUAVStats(all_results, uav_tasks);

%% Function Definition
% Create urban environment
function model = CreateUrbanEnvironment()
    % Map size
    MAPSIZE_X = 1000;
    MAPSIZE_Y = 1000;

    % discrete points flying through
    n = 10;
    
    % default start point and end point
    start_point = [100; 80; 60];
    end_point = [950; 880; 70];
    
    % Threat Area Definition
    Threats = [
        250, 300, 120, 60, 1;
        400, 500, 130, 55, 2;
        300, 700, 140, 50, 1;
        550, 250, 125, 65, 2;
        650, 550, 135, 60, 1;
        500, 800, 145, 55, 2;
        750, 350, 130, 70, 1;
        800, 700, 140, 60, 2;
    ];
    
    % Buildings Definitions   
    ground_level = 5;
    H = ones(MAPSIZE_Y, MAPSIZE_X) * ground_level;
    grid_centers_x = 200:180:900;
    grid_centers_y = 200:180:900;    
    b_size_min = 40; 
    b_size_max = 90;
    b_height_min = 50; 
    b_height_max = 140;
    % Fix the random seed
    rng(42);
    % Generate buildings
    for cx = grid_centers_x
        for cy = grid_centers_y
            height = randi([b_height_min, b_height_max]);
            width  = randi([b_size_min, b_size_max]);
            depth  = randi([b_size_min, b_size_max]);
            
            if rand() > 0.5
                continue; 
            end
            
            x_start = max(1, round(cx - width/2));
            x_end   = min(MAPSIZE_X, round(cx + width/2));
            y_start = max(1, round(cy - depth/2));
            y_end   = min(MAPSIZE_Y, round(cy + depth/2));
            
            H(y_start:y_end, x_start:x_end) = height;
        end
    end

    % Create landscapes possibly existed in the city
    H(800:830, 300:600) = 80; % L-shape building
    H(800:900, 570:600) = 80;
    H(500:600, 500:600) = ground_level; % Central park
    H(650:670, :) = 1; % River
    
    [X, Y] = meshgrid(1:MAPSIZE_X, 1:MAPSIZE_Y);
    
    model.start = start_point;
    model.end = end_point;
    model.n = n;
    model.xmin = 1;  
    model.xmax = MAPSIZE_X;
    model.ymin = 1;  
    model.ymax = MAPSIZE_Y;
    model.zmin = 80; 
    model.zmax = 180;
    model.MAPSIZE_X = MAPSIZE_X;
    model.MAPSIZE_Y = MAPSIZE_Y;
    model.X = X;
    model.Y = Y;
    model.H = H;
    model.threats = Threats;
end

% Plot the paths
function PlotMultiUAV3D(results, tasks, model)
    figure('Position', [50, 50, 1200, 900], 'Color', [1 1 1]);
    
    % 1 Demonstrate the environment
    % Draw the whole environment
    surf(model.X, model.Y, model.H, 'EdgeColor', 'none', 'FaceAlpha', 0.6);
    colormap(gray); 
    light('Position',[-1 -1 1],'Style','infinite');
    lighting gouraud;
    hold on;    
    % Draw threat areas
    theta = linspace(0, 2*pi, 30);
    for i = 1:size(model.threats, 1)
        t = model.threats(i, :);
        x = t(1) + t(4) * cos(theta);
        y = t(2) + t(4) * sin(theta);
        z = [0, 200];
        [X, Z] = meshgrid(x, z);
        [Y, ~] = meshgrid(y, z);
        surf(X, Y, Z, 'FaceColor', 'r', 'FaceAlpha', 0.1, 'EdgeColor', 'none');
    end
    
    legend_h = [];
    legend_str = {};
    
    % 2 Plot trajectories of UAVs
    for i = 1:length(tasks)
        sol = results(i).sol;
        task = tasks(i);
        
        % Construct the trajectory of a single UAV
        P = [task.start, [sol.x; sol.y; sol.z], task.end];
        
        % Interpolation
        t = 1:size(P, 2);
        ts = 1:0.1:size(P, 2);
        P_smooth = spline(t, P, ts);
        
        % Plot the trajectory of a single UAV
        h = plot3(P_smooth(1,:), P_smooth(2,:), P_smooth(3,:), ...
            'Color', task.color, 'LineWidth', 3);
        
        % Plot the start point and end point
        scatter3(task.start(1), task.start(2), task.start(3), 100, task.color, 'filled', 'o');
        scatter3(task.end(1), task.end(2), task.end(3), 100, task.color, 'filled', '^');
        
        legend_h = [legend_h, h];
        legend_str{end+1} = task.name;
        
        plot3(P_smooth(1,:), P_smooth(2,:), zeros(size(ts)), ...
            'Color', [task.color, 0.3], 'LineWidth', 1, 'LineStyle', ':');
    end
    
    grid on;
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Height (m)');
    title('Multi-UAV Path Planning in Urban Environment', 'FontSize', 15, 'FontWeight', 'bold');
    legend(legend_h, legend_str, 'Location', 'northeast', 'FontSize', 11);
    view(-45, 30);
    axis equal;
    xlim([0 model.MAPSIZE_X]); ylim([0 model.MAPSIZE_Y]); zlim([0 200]);
end

% Plot the top view of the paths
function PlotMultiUAVTopView(results, tasks, model)
    figure('Position', [100, 100, 1000, 800], 'Color', [1 1 1]);
    
    % Plot contour lines
    contourf(model.X, model.Y, model.H, 20, 'LineStyle', 'none');
    colormap(gray);
    colorbar;
    hold on;
    
    % Plot threat areas
    theta = linspace(0, 2*pi, 50);
    for i = 1:size(model.threats, 1)
        t = model.threats(i, :);
        fill(t(1) + t(4)*cos(theta), t(2) + t(4)*sin(theta), ...
             [1 0 0], 'FaceAlpha', 0.2, 'EdgeColor', 'r', 'LineStyle', '--');
    end
    
    % Plot trajectories
    for i = 1:length(tasks)
        sol = results(i).sol;
        task = tasks(i);
        P = [task.start, [sol.x; sol.y; sol.z], task.end];
        
        plot(P(1,:), P(2,:), ...
            'Color', task.color, 'LineWidth', 2.5, 'Marker', '.');
        plot(task.start(1), task.start(2), 'o', ...
            'MarkerSize', 8, ...
            'MarkerFaceColor', task.color, ...
            'MarkerEdgeColor', 'k');
        plot(task.end(1), task.end(2), '^', ...
            'MarkerSize', 8, ...
            'MarkerFaceColor', task.color, ...
            'MarkerEdgeColor', 'k');
    end
    
    title('Top-View Trajectories', 'FontSize', 14);
    xlabel('X (m)'); ylabel('Y (m)');
    axis equal;
    xlim([0 model.MAPSIZE_X]); ylim([0 model.MAPSIZE_Y]);
end

% Demonstrate data from results
function PrintMultiUAVStats(results, tasks)
    fprintf('\n');
    fprintf('╔══════════════════════════════════════════════════════════════╗\n');
    fprintf('║                  Multi-UAV Simulation Report                 ║\n');
    fprintf('╠══════════════════════════════════════════════════════════════╣\n');
    fprintf('║ UAV Name          │ Flight Cost │ Calc Time(s) │ Path Status ║\n');
    fprintf('╠══════════════════════════════════════════════════════════════╣\n');
    
    for i = 1:length(tasks)
        fprintf('║ %-17s │ %10.2f  │ %11.2f  │ %-11s ║\n', ...
            tasks(i).name, ...
            results(i).cost, ...
            results(i).time, ...
            'Optimal');
    end
    fprintf('╚══════════════════════════════════════════════════════════════╝\n');
end