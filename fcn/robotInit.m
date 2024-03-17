function [Robot, Enviroment] = robotInit()
    %% Define the limits of workspace
    limits = [];
    while (length(limits) ~= 4)
        limits = input('Insert environment boundaries (e.g. [xmin xmax ymin ymax]): ');
    end
   
    %% Define the number of obstacles
    n_obs = 0;
    while(n_obs == 0)
        n_obs = ceil(abs(input('Insert number of obstacles: ')));
    end
   
    %% Create obstacles on a figure
    figure();
    axis('equal'); axis(limits);
    grid on; box on; hold on;
   
    Enviroment.obstacle = cell(1, n_obs);
    for i = 1 : n_obs
        title(sprintf('Draw obstacle %g',i));
        counter = 1; % Number of vertices in current obstacle
        button = 1;
        Enviroment.obstacle{i}.vertices = [];  
        Enviroment.obstacle{i}.color = rand([1, 3]); % Random obstacle color
        while (button == 1)
            [x,y,button] = ginput(1); % Read one point
            if (isempty(button) || button ~= 1) % Try to finish current obstacle (button empty if Enter was pressed)
                try % Finish obstacle if can construct convex hull
                    cv = convhull(Enviroment.obstacle{i}.vertices(1, :), Enviroment.obstacle{i}.vertices(2, :));
                catch % Otherwise give a warning and continue to read vertices
                    warning('A convex hull cannot be obtained from the set of points');
                    button = 1;
                end
            else % Read a vertex
                plot(x, y, '.k');
                Enviroment.obstacle{i}.vertices = [Enviroment.obstacle{i}.vertices [x; y]];
                counter = counter + 1;
            end
        end
   
        cv(end) = []; % Remove duplicated vertex
        Enviroment.obstacle{i}.vertices = Enviroment.obstacle{i}.vertices(:, cv);
        Enviroment.obstacle{i}.handle = fill(Enviroment.obstacle{i}.vertices(1, :), Enviroment.obstacle{i}.vertices(2, :), Enviroment.obstacle{i}.color);

        Enviroment.obstacle{i}.center = [mean(Enviroment.obstacle{i}.vertices(1, :)); mean(Enviroment.obstacle{i}.vertices(2, :))];
        text(Enviroment.obstacle{i}.center(1), Enviroment.obstacle{i}.center(2), ['O_' num2str(i)], ...
            'Color', 'w', ...
            'FontWeight', 'bold', ...
            'HorizontalAlignment', 'center');
    end
   
%     Robot = robotConfig();
%     Robot = forwardKinematics(Robot);
%     plot([Robot.A(1) Robot.B(1) Robot.C(1)], [Robot.A(2) Robot.B(2) Robot.C(2)], '-or');
%    

    %% Create the robot in the environment
    Robot.vertices = []; % Coordinates of each point
    Robot.points = ['A', 'B', 'C']; % Lables of the points
    
    for i = 1 : 3 % Draw the points on the figure and store them
        title(sprintf('Insert robot point %s', Robot.points(i)));
        [x, y] = ginput(1); % Read one point
        plot(x, y, '-og');
        Robot.vertices = [Robot.vertices [x; y]];
    end
   
    plot(Robot.vertices(1, :), Robot.vertices(2, :), '-g'); % Plot the created two-link robot

    Robot.L1 = norm(Robot.vertices(:, 1) - Robot.vertices(:, 2)); % Length of the first link
    Robot.L2 = norm(Robot.vertices(:, 2) - Robot.vertices(:, 3)); % Length of the second link

    distance = Robot.L1 + Robot.L2 + 1;
    while (distance >= (Robot.L1 + Robot.L2)) % This check is done wrong
        title(sprintf('Insert robot goal position'));
        [Robot.goal(1), Robot.goal(2)] = ginput(1); % Read one point
        plot(Robot.goal(1), Robot.goal(2), '*r');
        distance = norm(Robot.vertices(:, 1) - Robot.goal);
    end

    %TODO: check if the goal position can be reached

    %calculate L1,L2, theta1, theta2 from inverse kinematics
    %Robot = inverseKinematics(Robot);
   
%     %plot goal position (red)
%     Robot.goal = [x; y];
%     title(sprintf('Insert robot goal position C'));
%     [x,y] = ginput(1);   %read one point
%     plot(x,y,'or')
    %check if goal position gets good configuration by L2
%     while sqrt((Robot.vertices(1,2)-Robot.goal(1))^2+...
%         (Robot.vertices(2,2)-Robot.goal(2))^2) ~= Robot.L2
%         warning('invalid goal position');
%         title(sprintf('Insert robot goal position C'));
%         [x,y] = ginput(1);   %read one point
%         plot(x,y,'or')
%     end

end
