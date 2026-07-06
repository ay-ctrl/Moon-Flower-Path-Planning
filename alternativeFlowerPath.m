%% 
clc
clear
close all

%% Parameters
params = struct();
params.dt = 0.2;            
params.v = 5;               
params.sun_radius = 4500;
params.sun_omega = 0.0015;   
params.alpha = 0.6;         
params.beta = 0.4;          
params.base = [0 0];
params.R_TARGETS = [100, 1100, 2000, 2700, 3500];
params.num_targets = length(params.R_TARGETS);
params.sun_start_angle = pi/4;

%% Grafics
fig = setup_figure(params.R_TARGETS);

%% Memory
max_iter = 100000;
trajectory_data = zeros(max_iter, 2);
iter_count = 0;
skip_frames = 15;

%% State
state = struct();
state.sun_angle = params.sun_start_angle;
state.current_idx = 1;
state.inner_r = params.R_TARGETS(state.current_idx);
state.outer_r = params.R_TARGETS(state.current_idx + 1);

state.rover_pos = state.inner_r * [cos(state.sun_angle), sin(state.sun_angle)];
% from polar cordinate to cartesian

state.start_ref_angle = state.sun_angle;
state.has_left_start = false;
state.mode = 1;
state.sim_on = true;

state.theta_leaf = 0;
state.theta_leaf_computed = false;

% NEW ADDED
state.impact_angle = 0;
state.impact_angle_set = false;

%% Plot objects
h.Rover = plot(0,0,'bo','MarkerFaceColor','b','MarkerSize',8);
h.Sun = plot(0,0,'ro','MarkerFaceColor','r','MarkerSize',10);
h.Path = plot(0,0,'b','LineWidth',1.2);
h.SunLine = plot([0 0],[0 0],'r--');

%% Simulation
while state.sim_on
    iter_count = iter_count + 1;
    
    % Sun movement
    state.sun_angle = state.sun_angle + params.sun_omega * params.dt;
    sun_pos = params.base + params.sun_radius * [cos(state.sun_angle), sin(state.sun_angle)];
    
    current_dist = norm(state.rover_pos - params.base);
    
    % Rover update
    state = update_rover(state, params, sun_pos, current_dist);
    
    trajectory_data(iter_count, :) = state.rover_pos;
    
    if mod(iter_count, skip_frames) == 0
        update_plot(h, state, sun_pos, trajectory_data, iter_count);
    end
    
    if iter_count >= max_iter
        break;
    end
end

%% Functions

function fig = setup_figure(R_TARGETS)
    fig = figure('Color', 'w', 'Position', [100 100 800 800]);
    hold on; axis equal; grid on;
    title('Flower Rover Simulation');
    theta_c = linspace(0, 2*pi, 200);
    for r = R_TARGETS
        plot(r*cos(theta_c), r*sin(theta_c), 'k--', 'LineWidth', 0.5)
    end
end

function state = update_rover(state, params, sun_pos, current_dist)

    switch state.mode
        
        %% MODE 1 - Extend outside
        case 1
            dir = (sun_pos - state.rover_pos) / norm(sun_pos - state.rover_pos);
            state.rover_pos = state.rover_pos + params.v * dir * params.dt;
            
            if current_dist >= state.outer_r
                state.mode = 2;
                
                % RECORD IMPACT ANGLE FIRST TIME
                if ~state.impact_angle_set
                    state.impact_angle = atan2(state.rover_pos(2), state.rover_pos(1));
                    state.impact_angle_set = true;
                end
            end
            
        %% MODE 2 - Turn inside
        case 2
            dir_base = (params.base - state.rover_pos) / norm(params.base - state.rover_pos);
            dir_sun = (sun_pos - state.rover_pos) / norm(sun_pos - state.rover_pos);
            
            move = params.alpha*dir_base + params.beta*dir_sun;
            state.rover_pos = state.rover_pos + params.v * (move/norm(move)) * params.dt;
            
            if current_dist <= state.inner_r
                state.mode = 3;
            end
            
        %% MODE 3 - Catch sun
        case 3
            state = mode3_capture_sun(state, params, sun_pos);
            
        %% MODE 4 - Above floor
        case 4
            dir_sun = (sun_pos - state.rover_pos) / norm(sun_pos - state.rover_pos);
            state.rover_pos = state.rover_pos + params.v * dir_sun * params.dt;
            
            if current_dist >= state.outer_r
                state.current_idx = state.current_idx + 1;
                
                state.inner_r = params.R_TARGETS(state.current_idx);
                state.outer_r = params.R_TARGETS(state.current_idx + 1);
                
                state.start_ref_angle = atan2(state.rover_pos(2), state.rover_pos(1));
                state.has_left_start = false;
                state.theta_leaf_computed = false;
                
                % RESET
                state.impact_angle_set = false;
                
                state.mode = 1;
            end
            
        %% MODE 5 - Circle follow
        case 5
            r_ang = atan2(state.rover_pos(2), state.rover_pos(1));
            r_ang = r_ang + (params.v/state.inner_r)*params.dt;
            
            state.rover_pos = state.inner_r * [cos(r_ang), sin(r_ang)];
            
            remaining_angle = abs(wrapToPi(state.impact_angle - r_ang));
            
            if remaining_angle < 0.05
                if state.current_idx < params.num_targets - 1
                    state.mode = 4;
                else
                    state.sim_on = false;
                end
            end
    end
end

function state = mode3_capture_sun(state, params, sun_pos)
    
    r_ang = atan2(state.rover_pos(2), state.rover_pos(1));
    r_ang = r_ang + (params.v/state.inner_r)*params.dt;
    
    state.rover_pos = state.inner_r * [cos(r_ang), sin(r_ang)];
    
    s_ang_rel = atan2(sun_pos(2), sun_pos(1));
    
    angle_from_start = abs(wrapToPi(r_ang - state.start_ref_angle));
    
    if angle_from_start > 0.5
        state.has_left_start = true;
    end
    
    if abs(wrapToPi(r_ang - s_ang_rel)) < 0.1
        
        plot(state.rover_pos(1), state.rover_pos(2), 'g*', 'MarkerSize', 5);
        
        if ~state.theta_leaf_computed
            theta_rover = atan2(state.rover_pos(2), state.rover_pos(1));
            state.theta_leaf = wrapToPi(theta_rover - params.sun_start_angle);
            state.theta_leaf_computed = true;
        end
        
        remaining_angle = abs(wrapToPi(params.sun_start_angle - r_ang));
        
        if remaining_angle >= abs(state.theta_leaf)
            state.mode = 1;
        else
            if state.has_left_start && angle_from_start < 0.2
                if state.current_idx < params.num_targets - 1
                    state.mode = 4;
                else
                    state.sim_on = false;
                end
            else
                state.mode = 5;
            end
        end
    end
end

function update_plot(h, state, sun_pos, trajectory_data, iter_count)
    set(h.Rover, 'XData', state.rover_pos(1), 'YData', state.rover_pos(2));
    set(h.Sun, 'XData', sun_pos(1), 'YData', sun_pos(2));
    set(h.Path, 'XData', trajectory_data(1:iter_count,1), 'YData', trajectory_data(1:iter_count,2));
    set(h.SunLine, 'XData', [0 sun_pos(1)], 'YData', [0 sun_pos(2)]);
    drawnow limitrate;
    pause(0.001)
end