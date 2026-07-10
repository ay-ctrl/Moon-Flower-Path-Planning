clc; clear; close all;

%% =========================================================================
%% 1. MAIN SIMULATION EXECUTION
%% =========================================================================

% Initialize params, states and graphics
[params, state] = initializeParameters();
[axSim, h] = initializeGraphics(params);

% Generate obstacles
obstacles = generateObstacles(axSim, params);

% Memory allocation for speed
max_iter = 300000;
skip_frames = 2;
trajectory_data = zeros(max_iter, 2);
ghost_trajectory = zeros(max_iter, 2);
iter_count = 0;

% Main simulation loop
while state.sim_on && iter_count < max_iter
    iter_count = iter_count + 1;
    
    % Time and sun update
    state = updateSun(state, params);
    
    % Layer and Decision Mechanism + Ghost Rover Movement
    state = updateGhost(state, params, obstacles, iter_count);
    
    % GReal Rover Logic and Kinematic Limitations
    state = updateRealRover(state, params);
    
    % Past data recording
    trajectory_data(iter_count, :) = state.rover_pos;
    ghost_trajectory(iter_count, :) = state.ghost_pos;
    
    % Visualization Layer
    if mod(iter_count, skip_frames) == 0
        drawSimulation(axSim, h, state, params, trajectory_data, ghost_trajectory, iter_count);
    end
end

%% =========================================================================
%% 2. ARCHITECTURAL FUNCTIONS 
%% =========================================================================

function [params, state] = initializeParameters()
    % Parameter struct
    params = struct();
    params.dt = 0.2; % Moon-based simulation time step (dt = 0.2)
    params.timeScale = 3000;            
    params.v = 0.05; % m/s
    params.sun_radius = 1700;
    params.sun_omega = 5 * 2 * pi / (29.53 * 24 * 3600); 
    params.alpha = 0.6;         
    params.beta = 0.4;          
    params.base = [0, 100]; % Center shifted to (0, 100)
    params.R_TARGETS = [500, 1000, 1300]; 
    params.num_layers = length(params.R_TARGETS);
    
    % Obstacle parameters
    params.num_obstacles = 20;
    params.min_obstacle_radius = 50;     
    params.max_obstacle_radius = 100;     
    params.obstacle_field_radius = params.R_TARGETS(end);
    
    % State struct
    state = struct();
    state.sun_angle = pi/4;     
    state.current_layer = 1;    
    state.mode = 1;             
    state.rover_pos = [0, 100];       
    state.polar_cords = [0, 0];     
    state.ghost_pos = [0, 100];
    state.sim_on = true;
    state.total_covered_angle = 0; 
    state.last_leaf_start_angle = state.sun_angle;
    state.layer_start_sun_angle = state.sun_angle; 
    state.leaf_angle_width = 0; 
    state.layer_start_iter = 1;
    state.is_first_leaf_of_layer = true;
    state.is_first_climb = false;
end

function [axSim, h] = initializeGraphics(params)
    fig = figure('Color', 'w'); 
    axSim = axes('Parent',fig);
    hold(axSim, 'on'); axis(axSim, 'equal'); grid(axSim, 'on');
    title(axSim, 'Ghost Rover: Layered Transition Mode Structure');
    
    theta_c = linspace(0, 2*pi, 300);
    for r = params.R_TARGETS
        plot(axSim, params.base(1) + r*cos(theta_c), params.base(2) + r*sin(theta_c), '-', 'Color', [0.8 0.8 0.8]);
    end
    
    h.layer_colors = lines(params.num_layers);
    for k = 1:params.num_layers
        h.Paths(k) = plot(axSim, nan, nan, 'LineWidth', 2.0, 'Color', h.layer_colors(k, :));
        h.GhostPaths(k) = plot(axSim, nan, nan, '--', 'LineWidth', 1.0, 'Color', [h.layer_colors(k, :), 0.6]); 
    end
    h.GhostRover = plot(axSim, params.base(1), params.base(2), 'go', 'MarkerFaceColor', 'none', 'MarkerEdgeColor', 'g', 'MarkerSize', 10); 
    h.Rover = plot(axSim, params.base(1), params.base(2), 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 8);
    h.Sun = plot(axSim, 0, 0, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 10);
    h.SunLine = plot(axSim, [params.base(1) 0], [params.base(2) 0], 'r--');
    h.LayerText = title(axSim, 'Initializing...');

    % --- TO FOCUS ON GHOST, HIDE REAL ROVER FOR NOW ---
    set(h.Rover, 'Visible', 'off'); 
    set(h.Paths, 'Visible', 'off');
end

function obstacles = generateObstacles(axSim, params)
    obstacles = struct('x', {}, 'y', {}, 'r', {});
    rng(42); 
    
    for i = 1:params.num_obstacles
        alpha_obs = rand() * 2 * pi;
        r_obs = 200 + rand() * (params.obstacle_field_radius - 200);
        
        obstacles(i).x = params.base(1) + r_obs * cos(alpha_obs);
        obstacles(i).y = params.base(2) + r_obs * sin(alpha_obs);
        obstacles(i).r = params.min_obstacle_radius + rand()*(params.max_obstacle_radius - params.min_obstacle_radius);
        
        % Calculate the circle's edge points
        th = linspace(0, 2*pi, 50);
        x_circle = obstacles(i).x + obstacles(i).r * cos(th);
        y_circle = obstacles(i).y + obstacles(i).r * sin(th);
        
        fill(axSim, x_circle, y_circle, 'r', 'EdgeColor', [0.2 0.2 0.2], 'LineWidth', 1.0, 'FaceAlpha', 0.5);
    end
end

function state = updateSun(state, params)
    dtSun = params.dt * params.timeScale;
    state.sun_angle = state.sun_angle + params.sun_omega * dtSun;
end

function state = updateGhost(state, params, obstacles, iter_count)
    dtSun = params.dt * params.timeScale;
    sun_pos = [params.sun_radius * cos(state.sun_angle), params.sun_radius * sin(state.sun_angle)];
    current_dist_ghost = norm(state.ghost_pos - params.base);
    target_r = params.R_TARGETS(state.current_layer);
    
    if state.current_layer == 1
        inner_r = 0;
    else
        inner_r = params.R_TARGETS(state.current_layer - 1);
    end
    
    % DECISION LAYER
    switch state.mode
        case 1 % Movement towards outside
            move_vec = (sun_pos - state.ghost_pos) / (norm(sun_pos - state.ghost_pos) + 1e-6);
            state.ghost_pos = moveGhost(state.ghost_pos, move_vec, params, obstacles, dtSun);
            
            if current_dist_ghost >= inner_r
                state.is_first_climb = false;
            end
            if current_dist_ghost >= target_r
                state.mode = 2;
            end
            
        case 2 % Returning towards center
            dir_base = (params.base - state.ghost_pos) / (norm(params.base - state.ghost_pos) + 1e-6);
            dir_sun = (sun_pos - state.ghost_pos) / (norm(sun_pos - state.ghost_pos) + 1e-6);
            move_vec = params.alpha * dir_base + params.beta * dir_sun;
            
            state.ghost_pos = moveGhost(state.ghost_pos, move_vec, params, obstacles, dtSun);
            
            if current_dist_ghost <= 15 
                state.ghost_pos = params.base; 
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
            
        case 4 % Wait sun for layer change
            state.ghost_pos = params.base; 
            angle_to_start = mod(state.sun_angle - state.layer_start_sun_angle, 2*pi);
            
            if angle_to_start < 0.05 && state.total_covered_angle > 1.0
                if state.current_layer < params.num_layers
                    state.current_layer = state.current_layer + 1;
                    state.total_covered_angle = 0;
                    state.leaf_angle_width = 0;
                    state.layer_start_sun_angle = state.sun_angle;
                    state.last_leaf_start_angle = state.sun_angle;
                    state.mode = 1;
                    state.is_first_climb = true;
                    
                    state.layer_start_iter = iter_count + 1; 
                else
                    state.sim_on = false;
                end
            end
    end
    
    % Polar projection calculation according to the center (0,100) shift
    [t, r] = cart2pol(state.ghost_pos(1) - params.base(1), state.ghost_pos(2) - params.base(2));
    state.polar_cords = [r, t];
end

function newPos = moveGhost(currPos, moveVec, params, obstacles, dt)
    % MOTION LAYER & COLLISION CONTROL 
    if norm(moveVec) > 0
        moveVec = moveVec / norm(moveVec);
    end
    
    % Instant control for a simple and stable local escape approach
    collision = false;
    avoidObstacleIdx = 0;
    for i = 1:length(obstacles)
        dist_to_obs = norm(currPos - [obstacles(i).x, obstacles(i).y]);
        if dist_to_obs <= obstacles(i).r + 5 
            collision = true;
            avoidObstacleIdx = i;
            break;
        end
    end
    
    if collision
        % Escape vector generation in the clockwise (CW) direction
        obs = obstacles(avoidObstacleIdx);
        to_robot = currPos - [obs.x, obs.y];
        dist_to_obs = norm(to_robot);
        
        % Clockwise (CW) tangent: [y, -x]
        tangent_dir = [to_robot(2), -to_robot(1)] / (dist_to_obs + 1e-6);
        radial_push = to_robot / (dist_to_obs + 1e-6);
        
        avoid_dir = tangent_dir * 1.0 + radial_push * 0.2;
        avoid_dir = avoid_dir / norm(avoid_dir);
        
        newPos = currPos + params.v * avoid_dir * dt;
    else
        % Normal movement
        newPos = currPos + params.v * moveVec * dt;
    end
end

function state = updateRealRover(state, params)
    % KINEMATIC CONSTRAINT AND FOLLOW LAYER
    if state.current_layer == 1
        inner_r = 0;
    else
        inner_r = params.R_TARGETS(state.current_layer - 1);
    end
    
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
    % apply the center shift (0, 100) to the real robot too
    state.rover_pos = [rx + params.base(1), ry + params.base(2)];
end

function drawSimulation(~, h, state, params, trajectory_data, ghost_trajectory, iter_count)
    % VISUALIZATION LAYER 
    sun_pos = [params.sun_radius * cos(state.sun_angle), params.sun_radius * sin(state.sun_angle)];
    
    set(h.Rover, 'XData', state.rover_pos(1), 'YData', state.rover_pos(2));
    set(h.GhostRover, 'XData', state.ghost_pos(1), 'YData', state.ghost_pos(2));
    set(h.Sun, 'XData', sun_pos(1), 'YData', sun_pos(2));
    set(h.SunLine, 'XData', [params.base(1), sun_pos(1)], 'YData', [params.base(2), sun_pos(2)]);
    
    curr_idx = state.layer_start_iter:iter_count; 
    set(h.Paths(state.current_layer), 'XData', trajectory_data(curr_idx, 1), 'YData', trajectory_data(curr_idx, 2));
    set(h.GhostPaths(state.current_layer), 'XData', ghost_trajectory(curr_idx, 1), 'YData', ghost_trajectory(curr_idx, 2));
    
    mode_str = sprintf('MODE: %d', state.mode);
    set(h.LayerText, 'String', sprintf('Layer: %d | %s | Speed: %dx', state.current_layer, mode_str, params.timeScale));
    
    drawnow;
end