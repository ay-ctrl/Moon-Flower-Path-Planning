clc; clear; close all;

%% 1. Parameters
params = struct();
params.dt = 0.2;            
params.v = 5;               
params.sun_radius = 4500;
params.sun_omega = 0.0015;   
params.alpha = 0.6;         
params.beta = 0.4;          
params.base = [0, 0];
params.R_TARGETS = [1100, 2000, 2900]; 
params.num_layers = length(params.R_TARGETS);

%% 2. Grafic Setting
fig = figure('Color', 'w', 'Position', [100 100 850 850]);
hold on; axis equal; grid on;
title('Ghost Rover: Katman Geçişli Mod Yapısı');

theta_c = linspace(0, 2*pi, 300);
for r = params.R_TARGETS
    plot(r*cos(theta_c), r*sin(theta_c), '-', 'Color', [0.8 0.8 0.8]);
end
h.layer_colors = lines(params.num_layers);

for k = 1:params.num_layers
    h.Paths(k) = plot(nan, nan, 'LineWidth', 2.0, 'Color', h.layer_colors(k, :));
    h.GhostPaths(k) = plot(nan, nan, '--', 'LineWidth', 1.0, 'Color', [h.layer_colors(k, :), 0.6]); 
end

h.GhostRover = plot(0,0,'go','MarkerFaceColor','none','MarkerEdgeColor','g','MarkerSize',10); 
h.Rover = plot(0,0,'bo','MarkerFaceColor','b','MarkerSize',8);
h.Sun = plot(0,0,'ro','MarkerFaceColor','r','MarkerSize',10);
h.SunLine = plot([0 0],[0 0],'r--');
h.LayerText = title('Başlatılıyor...');

%% 3. State Variables
state = struct();
state.sun_angle = pi/4;     
state.current_layer = 1;    
state.mode = 3;            
state.rover_pos = [0, 0];   
state.ghost_pos = [0, 0];   
state.sim_on = true;
state.total_covered_angle = 0; 
state.last_leaf_start_angle = state.sun_angle;
state.layer_start_sun_angle = state.sun_angle; 
state.leaf_angle_width = 0; 
state.layer_start_iter = 1;
state.is_first_leaf_of_layer = true; 

max_iter = 300000;
trajectory_data = zeros(max_iter, 2);
ghost_trajectory = zeros(max_iter, 2);
iter_count = 0;
skip_frames = 25; 

%% 4. Simulation Loop
while state.sim_on && iter_count < max_iter
    iter_count = iter_count + 1;
    
    state.sun_angle = state.sun_angle + params.sun_omega * params.dt;
    sun_pos = [params.sun_radius * cos(state.sun_angle), params.sun_radius * sin(state.sun_angle)];
    
    current_dist_ghost = norm(state.ghost_pos - params.base);
    target_r = params.R_TARGETS(state.current_layer);
    
    % --- GHOST DECISION MECHANISM ---
    
    if state.mode == 3 %% First climb to the layer
        dir_sun = (sun_pos - state.ghost_pos) / (norm(sun_pos - state.ghost_pos) + 1e-6);
        state.ghost_pos = state.ghost_pos + params.v * dir_sun * params.dt;
        state.rover_pos = state.ghost_pos;
        
        if current_dist_ghost >= target_r
            state.mode = 2; 
            state.is_first_leaf_of_layer = false; 
        end

    elseif state.mode == 1 %% MOD 1: Normal climb to outside (other leaves than first one)
        dir_sun = (sun_pos - state.ghost_pos) / (norm(sun_pos - state.ghost_pos) + 1e-6);
        state.ghost_pos = state.ghost_pos + params.v * dir_sun * params.dt;
        state.rover_pos = state.ghost_pos;
        
        if current_dist_ghost >= target_r
            state.mode = 2; 
        end
        
    elseif state.mode == 2 %% MOD 2: Return inside
        dir_base = (params.base - state.ghost_pos) / (norm(params.base - state.ghost_pos) + 1e-6);
        dir_sun = (sun_pos - state.ghost_pos) / (norm(sun_pos - state.ghost_pos) + 1e-6);
        move_vec = params.alpha * dir_base + params.beta * dir_sun;
        state.ghost_pos = state.ghost_pos + params.v * (move_vec/norm(move_vec)) * params.dt;
        
        if current_dist_ghost <= 35 
            state.ghost_pos = [0, 0]; 
            angle_diff = state.sun_angle - state.last_leaf_start_angle;
            if state.leaf_angle_width == 0, state.leaf_angle_width = angle_diff; end
            state.total_covered_angle = state.total_covered_angle + angle_diff;
            state.last_leaf_start_angle = state.sun_angle;
            
            if (2*pi - state.total_covered_angle) < state.leaf_angle_width
                state.mode = 4; % Wait for sun to come in the beginning position
            else
                state.mode = 1; % Continue to draw leaves
            end
        end
        
    elseif state.mode == 4 
        state.ghost_pos = [0, 0]; 
        angle_to_start = mod(state.sun_angle - state.layer_start_sun_angle, 2*pi);
        
        if angle_to_start < 0.05 && state.total_covered_angle > 1.0
            if state.current_layer < params.num_layers
                state.current_layer = state.current_layer + 1;
                state.total_covered_angle = 0;
                state.layer_start_iter = iter_count + 1;
                state.leaf_angle_width = 0;
                state.layer_start_sun_angle = state.sun_angle;
                state.last_leaf_start_angle = state.sun_angle;
                state.is_first_leaf_of_layer = true; % New layer started
                state.mode = 3; % Turn mode 3 for new layer
            else
                state.sim_on = false;
            end
        end
    end

    % Save path to visualize
    ghost_trajectory(iter_count, :) = state.ghost_pos;
    
    % Visualization
    if mod(iter_count, skip_frames) == 0
        set(h.Rover, 'XData', state.rover_pos(1), 'YData', state.rover_pos(2));
        set(h.GhostRover, 'XData', state.ghost_pos(1), 'YData', state.ghost_pos(2));
        set(h.Sun, 'XData', sun_pos(1), 'YData', sun_pos(2));
        set(h.SunLine, 'XData', [0 sun_pos(1)], 'YData', [0 sun_pos(2)]);
        
        curr_idx = state.layer_start_iter:iter_count;
        set(h.Paths(state.current_layer), 'XData', trajectory_data(curr_idx, 1), 'YData', trajectory_data(curr_idx, 2));
        set(h.GhostPaths(state.current_layer), 'XData', ghost_trajectory(curr_idx, 1), 'YData', ghost_trajectory(curr_idx, 2));
        
        mode_str = sprintf('MOD: %d', state.mode);
        set(h.LayerText, 'String', sprintf('Katman: %d | %s', state.current_layer, mode_str));
        
        drawnow limitrate;
    end
end
    
    
    
   