function my_alg = ProbabilisticMapping(my_alg, robot)

%% Initialisation
    if my_alg('is_first_time')
        % signal form used for controlling the motor change if necessary to 'voltage_pwm'
        my_alg('dc_motor_signal_mode') = 'omega_setpoint';
        
        map.ll_corner = [-10 -5];   % left lower corner coordinate
        map.ur_corner = [10 5];     % upper right corner coordinate
        map.res = 0.1;              % grid map resolution
        % number of grids in for row and column
        map_size = floor(map.ur_corner - map.ll_corner)./map.res+1;
        % set each grid in the map to 50% probability
        map.OGrid = 0.5* ones(map_size(2),map_size(1));
        disp(['map_size', map_size(1)])
        disp(['map_size', map_size(2)])
        
        
        % the type of mapping algorithm 'binary' or 'log_odds'
        map.type = 'binary';
        % Save the object map into the main algorithm
        my_alg('map') = map;
        
        % From the selected robot model access the angle Span of LiDAR
        Angle_span              = robot.components_tree.get('lidar').angle_span/360*2*pi;
        % From the selected robot model access the angle resolution of LiDAR
        Angle_resolution        = robot.components_tree.get('lidar').angle_resolution/360*2*pi;
        % create a vector of the set of all possible bearings of LiDAR in order
        my_alg('Th')            = -(Angle_span/2)+Angle_resolution:Angle_resolution:(Angle_span/2);
        
        my_alg('turn')          = false;  % 0 表示 没有开启转向模式
    my_alg('velocity')      = 0 ;
    my_alg('start_turn')      = 1 ;   % 1 表示没有开启转向初始化
    my_alg('acc_time')      = 0 ;
    end
    
%% Use driveToGoal to move the robot (change to driveToMultipleGoals if necessary)
    % my_alg = driveToGoal(my_alg, robot);

%% Required variables
    % Robot pose estimation [(m) (m) (rad)]'
    S                   = my_alg('localizer').pose;
    % Vector of the set of all possible bearings of LiDAR in order
    Theta               = my_alg('Th');
    % Vectoe of range measurements with respect to 'Theta'
    Range               = my_alg('lidar');
    % From the selected robot model access the maximum range measurement
    max_range           = robot.components_tree.get('lidar').max_range;
    % map object
    map                 = my_alg('map');
    
    turn            = my_alg('turn');
    velocity            = my_alg('velocity');  
    start_turn            = my_alg('start_turn');
    acc_time            = my_alg('acc_time');
% =========================================================================
%% ==== Coursework 2 - Task 5 - Occupancy Grid Binary Mapping =============
% ===== Start Here ========================================================


%Write your code here
% Convert LiDAR measurements to Cartesian coordinates
% [x_lidar, y_lidar] = pol2cart(Theta, Range);

% Loop over each LiDAR measurement
for i = 1:length(Range)
    if Range(i) == inf
        Range(i) = 5 ;
    end
    % Convert LiDAR measurements to global coordinates
    x_global = S(1) + Range(i) * cos(S(3) + Theta(i));
    y_global = S(2) + Range(i) * sin(S(3) + Theta(i));
    
    % Check if the converted coordinates are within the map bounds
    if x_global >= map.ll_corner(1) && x_global <= map.ur_corner(1) && ...
            y_global >= map.ll_corner(2) && y_global <= map.ur_corner(2)
        % Convert global coordinates to grid indices
        grid_x = floor((x_global - map.ll_corner(1)) / map.res) + 1;
        grid_y = floor((y_global - map.ll_corner(2)) / map.res) + 1;

        if Range(i) ~= 5
            Prob_m = map.OGrid(grid_y, grid_x);
            map.OGrid(grid_y, grid_x) = 0.85 * Prob_m/(0.85 * Prob_m + 0.2 * (1-Prob_m)); % Set probability of occupancy???
        end
        % Update the corresponding cell in the occupancy grid map
        % map.OGrid(grid_y, grid_x) = 1; % Set cell to occupied
        
        
        for r_nodet = 0:0.01:Range(i)-0.3
            
            x_global = S(1) + r_nodet * cos(S(3) + Theta(i));
            y_global = S(2) + r_nodet * sin(S(3) + Theta(i));
                
                % Convert global coordinates to grid indices
            grid_x = floor((x_global - map.ll_corner(1)) / map.res) + 1;
            grid_y = floor((y_global - map.ll_corner(2)) / map.res) + 1;
    
            
            % Update the corresponding cell in the occupancy grid map
            % map.OGrid(grid_y, grid_x) = 1; % Set cell to occupied
            Prob_m = map.OGrid(grid_y, grid_x);
            map.OGrid(grid_y, grid_x) = 0.3 * Prob_m/(0.3 * Prob_m + 0.9 * (1-Prob_m)); % Set probability of occupancy???

        end


    end
end


% MAP = map.OGrid(grid_y, grid_x)


% % Update the map object in my_alg
% my_alg('map') = map;
% 
% % Plot the estimated map
% figure(2), clf
% worldPlot2(map, 1);
% title('Probabilistic Mapping');




% ===== Finish Here =======================================================
% ===== Coursework 2 - Task 5 - Occupancy Grid Binary Mapping =============
% =========================================================================
    
    % save map object
    my_alg('map') = map;

    my_alg('turn')          = turn;
    my_alg('velocity')      = velocity;
    my_alg('start_turn')    = start_turn;
    my_alg('acc_time')      = acc_time;

    % plot the estimated map
    figure(2), clf
    worldPlot2( map,1);
    title('Probabilistic Mapping');
return