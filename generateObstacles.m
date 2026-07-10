function obstacles = generateObstacles(ax, params)

    %   Generates random circular obstacles
    %   Obstacles are randomly distributed inside the exploration area.
    
    hold(ax,'on')
    
    theta = linspace(0,2*pi,80);
    
    N = params.num_obstacles;
    
    obstacles = struct( ...
        'center', cell(N,1), ...
        'radius', cell(N,1));
    
    for i = 1:N
    
        % Random position (uniform over area)
        ang = 2*pi*rand;
        r = sqrt(rand) * params.obstacle_field_radius;
    
        [x,y] = pol2cart(ang,r);
    
        % Random obstacle size
        radius = params.min_obstacle_radius + ...
            rand*(params.max_obstacle_radius - params.min_obstacle_radius);
    
        % Store obstacle
        obstacles(i).center = [x y];
        obstacles(i).radius = radius;
    
        % Draw obstacle
        fill(ax,...
            x + radius*cos(theta),...
            y + radius*sin(theta),...
            'r',...
            'EdgeColor','k',...
            'FaceAlpha',0.55);
    
    end

end