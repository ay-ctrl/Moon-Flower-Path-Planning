clc; clear; close all;

%% 1. Parameters
params = struct();
params.dt = 0.01; % how fast we calculate

% --- Time Scaling ---
% Since the lunar day is very long (approximately 708 hours), we must accelerate 
% time by at least a factor of 1000 so that the simulation runs at a visible speed on the screen.
params.timeScale = 3000;            

% --- Rover Speed ​​---
% The Lunar Roving Vehicle (LRV)—the first crewed vehicle to land on the Moon—was fast, but
% autonomous/uncrewed rovers (e.g., China's Yutu-2 rover) move at a
% snail's pace to avoid hazards. An average speed of 0.05 m/s (180 meters per hour) is very realistic.
params.v = 0.05;                    % m/s
         
params.sun_radius = 1700;

% Moon's actual rotation period: ~29.53 Earth days.
% In seconds: 29.53 * 24 * 3600 = 2,551,392 seconds.
% Angular velocity (rad/s):
params.sun_omega = 2 * pi / (29.53 * 24 * 3600); % ~2.46e-6 rad/s

params.alpha = 0.6;         
params.beta = 0.4;          
params.base = [0, 0];

% --- Layer Radii ---
% Reasonable exploration layers (in meters) extending from the center of the lunar base
params.R_TARGETS = [500, 1000, 1300]; 
params.num_layers = length(params.R_TARGETS);
params.num_layers = length(params.R_TARGETS);

%% 2. Graphics Setup
fig = figure('Color', 'w'); 
axSim = axes('Parent',fig);
hold(axSim, 'on'); axis(axSim, 'equal'); grid(axSim, 'on');
title(axSim, 'Ghost Rover: Layered Transition Mode Structure');

theta_c = linspace(0, 2*pi, 300);
for r = params.R_TARGETS
    plot(axSim, r*cos(theta_c), r*sin(theta_c), '-', 'Color', [0.8 0.8 0.8]);
end

h.layer_colors = lines(params.num_layers);
for k = 1:params.num_layers
    h.Paths(k) = plot(axSim, nan, nan, 'LineWidth', 2.0, 'Color', h.layer_colors(k, :));
    h.GhostPaths(k) = plot(axSim, nan, nan, '--', 'LineWidth', 1.0, 'Color', [h.layer_colors(k, :), 0.6]); 
end

h.GhostRover = plot(axSim, 0,0,'go','MarkerFaceColor','none','MarkerEdgeColor','g','MarkerSize',10); 
h.Rover = plot(axSim, 0,0,'bo','MarkerFaceColor','b','MarkerSize',8);
h.Sun = plot(axSim, 0,0,'ro','MarkerFaceColor','r','MarkerSize',10);
h.SunLine = plot(axSim, [0 0],[0 0],'r--');
h.LayerText = title(axSim, 'Initializing...');

%% 3. State Variables
state = struct();
state.sun_angle = pi/4;     
state.current_layer = 1;    
state.mode = 1;             
state.rover_pos = [0, 0];       % r, theta
state.polar_cords = [0, 0];     % r, theta of ghost
state.ghost_pos = [0, 0];
state.sim_on = true;
state.total_covered_angle = 0; 
state.last_leaf_start_angle = state.sun_angle;
state.layer_start_sun_angle = state.sun_angle; 
state.leaf_angle_width = 0; 
state.layer_start_iter = 1;
state.is_first_leaf_of_layer = true;
state.is_first_climb = false;

max_iter = 300000;
trajectory_data = zeros(max_iter, 2);
ghost_trajectory = zeros(max_iter, 2);
iter_count = 0;
skip_frames = 25; 

%% 4. Simulation Loop
while state.sim_on && iter_count < max_iter
    iter_count = iter_count + 1;
    
    % Accelerated time zone
    dtSun = params.dt * params.timeScale;
    
    state.sun_angle = state.sun_angle + params.sun_omega * dtSun;
    sun_pos = [params.sun_radius * cos(state.sun_angle), params.sun_radius * sin(state.sun_angle)];
    
    current_dist_ghost = norm(state.ghost_pos - params.base);
    target_r = params.R_TARGETS(state.current_layer);
    
    if state.current_layer == 1
        inner_r = 0;
    else
        inner_r = params.R_TARGETS(state.current_layer - 1);
    end
    
    %% --- GHOST MOVEMENT ---
    if state.mode == 1
        dir_sun = (sun_pos - state.ghost_pos) / (norm(sun_pos - state.ghost_pos) + 1e-6);
        state.ghost_pos = state.ghost_pos + params.v * dir_sun * dtSun;
        [t, r] = cart2pol(state.ghost_pos(1), state.ghost_pos(2));
        state.polar_cords = [r, t];
        if current_dist_ghost >= inner_r
            state.is_first_climb = false;
        end
        if current_dist_ghost >= target_r
            state.mode = 2;
        end
        
    elseif state.mode == 2
        dir_base = (params.base - state.ghost_pos) / (norm(params.base - state.ghost_pos) + 1e-6);
        dir_sun = (sun_pos - state.ghost_pos) / (norm(sun_pos - state.ghost_pos) + 1e-6);
        move_vec = params.alpha * dir_base + params.beta * dir_sun;
        state.ghost_pos = state.ghost_pos + params.v * (move_vec/norm(move_vec)) * dtSun;
        [t, r] = cart2pol(state.ghost_pos(1), state.ghost_pos(2));
        state.polar_cords = [r, t];
        
        if current_dist_ghost <= 15 
            state.ghost_pos = [0, 0]; 
            angle_diff = state.sun_angle - state.last_leaf_start_angle;
            
            if state.leaf_angle_width == 0
                state.leaf_angle_width = angle_diff;
            end
            
            state.total_covered_angle = state.total_covered_angle + angle_diff;
            state.last_leaf_start_angle = state.sun_angle;
            
            if (2*pi - state.total_covered_angle) < state.leaf_angle_width
                state.mode = 4;
            else
                state.mode = 1;
            end
        end
        
    elseif state.mode == 4
        state.ghost_pos = [0, 0]; 
        state.polar_cords = [0, state.sun_angle];
        angle_to_start = mod(state.sun_angle - state.layer_start_sun_angle, 2*pi);
        
        if angle_to_start < 0.05 && state.total_covered_angle > 1.0
            if state.current_layer < params.num_layers
                state.current_layer = state.current_layer + 1;
                state.total_covered_angle = 0;
                state.layer_start_iter = iter_count + 1;
                state.leaf_angle_width = 0;
                state.layer_start_sun_angle = state.sun_angle;
                state.last_leaf_start_angle = state.sun_angle;
                state.mode = 1;
                state.is_first_climb = true;
            else
                state.sim_on = false;
            end
        end
    end
    
    %% --- REAL ROVER LOGIC ---
    dist_ghost = state.polar_cords(1); 
    
    if state.current_layer == 1 || dist_ghost >= inner_r || (state.current_layer==2 && state.is_first_climb) || (state.is_first_climb && dist_ghost >= params.R_TARGETS(state.current_layer-2))
        r_eff = state.polar_cords(1);
    elseif dist_ghost <= inner_r &&  state.is_first_climb
        r_eff = params.R_TARGETS(state.current_layer-2);
    else 
        r_eff = inner_r;
    end
    theta_eff = state.polar_cords(2);
    
    [rx, ry] = pol2cart(theta_eff, r_eff);
    state.rover_pos = [rx, ry];
    
    trajectory_data(iter_count, :) = state.rover_pos;
    ghost_trajectory(iter_count, :) = state.ghost_pos;
    
    %% Visualization
    if mod(iter_count, skip_frames) == 0
        set(h.Rover, 'XData', state.rover_pos(1), 'YData', state.rover_pos(2));
        set(h.GhostRover, 'XData', state.ghost_pos(1), 'YData', state.ghost_pos(2));
        set(h.Sun, 'XData', sun_pos(1), 'YData', sun_pos(2));
        set(h.SunLine, 'XData', [0, sun_pos(1)], 'YData', [0, sun_pos(2)]);
        
        curr_idx = state.layer_start_iter:iter_count;
        set(h.Paths(state.current_layer), 'XData', trajectory_data(curr_idx, 1), 'YData', trajectory_data(curr_idx, 2));
        set(h.GhostPaths(state.current_layer), 'XData', ghost_trajectory(curr_idx, 1), 'YData', ghost_trajectory(curr_idx, 2));
        
        mode_str = sprintf('MODE: %d', state.mode);
        set(h.LayerText, 'String', sprintf('Layer: %d | %s | Speed: %dx', state.current_layer, mode_str, params.timeScale));
        
        drawnow;
    end
end