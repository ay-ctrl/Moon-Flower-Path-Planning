clc
clear
close all

%% Parameters
dt = 0.2;            
v = 5;               
sun_radius = 4500;
sun_omega = 0.0015;   
alpha = 0.6;         
beta = 0.4;          
base = [0 0];
sun_angle = pi/4;
R_TARGETS = [100, 1100, 2000, 2700,3500];
num_targets = length(R_TARGETS);
sun_start_angle = pi/4;

%% Video Record

% video_filename = 'sim_record.mp4';
% v_writer = VideoWriter(video_filename, 'MPEG-4');
% v_writer.FrameRate = 30; 
% open(v_writer);

%% Grafic Settings
fig = figure('Color', 'w', 'Position', [100 100 800 800]);
hold on; axis equal; grid on;
title('Optimize Edilmiş Rover Simülasyonu');
theta_c = linspace(0, 2*pi, 200);
for r = R_TARGETS
    plot(r*cos(theta_c), r*sin(theta_c), 'k--', 'LineWidth', 0.5)
end
hRover = plot(0,0,'bo','MarkerFaceColor','b','MarkerSize',8);
hSun = plot(0,0,'ro','MarkerFaceColor','r', 'MarkerSize',10);
hPath = plot(0,0,'b','LineWidth',1.2);
hSunLine = plot([0 0],[0 0],'r--');

%% Memory Management
max_iter = 100000;
trajectory_data = zeros(max_iter, 2); 
iter_count = 0;
skip_frames = 15; % Record 1 frame per 15 frames

%% Simulation Variables
current_idx = 1;
inner_r = R_TARGETS(current_idx);
outer_r = R_TARGETS(current_idx + 1);
rover_pos = base + inner_r * [cos(sun_angle) sin(sun_angle)];
start_ref_angle = sun_angle; 
has_left_start = false; 
mode = 1; 
sim_on = true;
theta_leaf = 0;
theta_leaf_computed = false;

while sim_on
    iter_count = iter_count + 1;
    
    % Physical Calculations
    sun_angle = sun_angle + sun_omega * dt;
    sun_pos = base + sun_radius * [cos(sun_angle) sin(sun_angle)];
    current_dist = norm(rover_pos - base);
    
    if mode == 1 % Extend outside
        dir = (sun_pos - rover_pos) / norm(sun_pos - rover_pos);
        rover_pos = rover_pos + v * dir * dt;
        if current_dist >= outer_r, mode = 2; end
    elseif mode == 2 % Turn inside
        dir_base = (base - rover_pos) / norm(base - rover_pos);
        dir_sun = (sun_pos - rover_pos) / norm(sun_pos - rover_pos);
        move = alpha*dir_base + beta*dir_sun;
        rover_pos = rover_pos + v * (move/norm(move)) * dt;
        if current_dist <= inner_r, mode = 3; end
    elseif mode == 3 % Catch sun
        r_ang = atan2(rover_pos(2), rover_pos(1));
        r_ang = r_ang + (v/inner_r)*dt;
        rover_pos = inner_r * [cos(r_ang) sin(r_ang)];
        s_ang_rel = atan2(sun_pos(2), sun_pos(1));
        angle_from_start = abs(wrapToPi(r_ang - start_ref_angle));
        if angle_from_start > 0.5, has_left_start = true; end
        if abs(wrapToPi(r_ang - s_ang_rel)) < 0.1
            plot(rover_pos(1), rover_pos(2), 'g*', 'MarkerSize', 5);
            if ~theta_leaf_computed
                theta_rover = atan2(rover_pos(2), rover_pos(1));
                theta_leaf = wrapToPi(theta_rover - sun_start_angle);
                theta_leaf_computed = true;
            end
            remaining_angle = abs(wrapToPi(sun_start_angle - r_ang));

            if remaining_angle >= abs(theta_leaf)
                % Still have place to draw leaf
                disp(remaining_angle);
                disp(theta_leaf);
                mode = 1;
            else
                disp("no");
                % No place, turn to beginning
                if has_left_start && angle_from_start < 0.2
                    disp("inner");
                    if current_idx < num_targets - 1
                        mode = 4;
                    else
                        sim_on = false;
                    end
                else
                    mode = 5; 
                end
            end
        end
    elseif mode == 4 % Climb next floor
        dir_sun = (sun_pos - rover_pos) / norm(sun_pos - rover_pos);
        rover_pos = rover_pos + v * dir_sun * dt;
        if current_dist >= outer_r
            current_idx = current_idx + 1;
            inner_r = R_TARGETS(current_idx);
            outer_r = R_TARGETS(current_idx + 1);
            start_ref_angle = atan2(rover_pos(2), rover_pos(1));
            has_left_start = false; theta_leaf_computed = false; mode = 1;
        end
    elseif mode == 5 % Circle follow
        r_ang = atan2(rover_pos(2), rover_pos(1));
        r_ang = r_ang + (v/inner_r)*dt; % move angular
        rover_pos = inner_r * [cos(r_ang) sin(r_ang)];
        
        remaining_angle = abs(wrapToPi(start_ref_angle - r_ang));
        if remaining_angle < 0.05 
            if current_idx < num_targets - 1
                mode = 4; 
            else
                sim_on = false;
            end
        end
 end
   
    % Save data
    trajectory_data(iter_count, :) = rover_pos;
    
    % ---  DRAW AND SAVE ---
    if mod(iter_count, skip_frames) == 0
        set(hRover, 'XData', rover_pos(1), 'YData', rover_pos(2))
        set(hSun, 'XData', sun_pos(1), 'YData', sun_pos(2))
        set(hPath, 'XData', trajectory_data(1:iter_count,1), 'YData', trajectory_data(1:iter_count,2))
        set(hSunLine, 'XData', [0 sun_pos(1)], 'YData', [0 sun_pos(2)])
        
        drawnow limitrate 
        % pause(0.0005)
        
        %  Add a frame to the video
        %  frame = getframe(fig);
        %  writeVideo(v_writer, frame);
    end
    
    if iter_count >= max_iter, break; end
end

% close(v_writer);
% disp(['Video ready: ', video_filename]);