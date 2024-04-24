function [my_alg] = ObstacleAvoidanceLiDAR(my_alg, robot,delta_t)
%% Initialise all variables and settings
if my_alg('is_first_time') 
% =========================================================================
% ========= Coursework 2 - Task 4 - Obstacle Avoidance using LiDAR =========
% ========= Initialise variables Here =====================================
        
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


% ========= Finish initialisation Here ====================================
% ========= Coursework 2 - Task 4 - Obstacle Avoidance using LiDAR ======
% =========================================================================
%% Call required state variables and system constants
% Varaibles that can be used for Coursework 1 - Task 5
% Robot pose estimation [(m) (m) (rad)]'
S                   = my_alg('localizer').pose;
% From the selected robot model call the distance between the two wheels(m)
L                   = 2*robot.components_tree.get('left motor').transformation(2,end);
% From the selected robot model call the radius of the wheel (m)
r                   = robot.components_tree.get('left wheel').shape.diameter/2;
% maximum angular velocity of the motor (rad/s)
w_sat               = 13.8;
% right wheel angular velocity (rad/s)
w_r                 = my_alg('right motor');
% Left wheel angular velocity (rad/s)
w_l                 = my_alg('left motor');
% The set of all bearings
Theta               = my_alg('Th');
%
Range               = my_alg('lidar');

    turn            = my_alg('turn');
    velocity            = my_alg('velocity');  
    start_turn            = my_alg('start_turn');
    acc_time            = my_alg('acc_time');

% =========================================================================
%% ======== Coursework 2 - Task 4 - Obstacle Avoidance - LiDAR ============
% ========= Start Here ====================================================
%Write your code here


velocity = 10;
    %Write your code here
    % turn = 0;
    out_turn = 1;

     goal = [8  0]';
     e_x = goal(1) - S(1);
     e_y = goal(2) - S(2);
    % e_I = e_I + sqrt(e_x^2 + e_y^2);


    e_theata = atan2(e_y,e_x) - S(3);
    turn ;
    start_turn;
    % delta_t = 0.02;

     if Range(180) < 1.5  && start_turn == 1


        % servo_motor = -1 ;
        % dt   = toc(robot.timestamp);               % Compute Change in time
        % set_property(obj, 'timestamp' , tic);               % reset stopwatch
        acc_time = acc_time + delta_t;
        if acc_time<0.5
            w_l = -5 ;
            w_r = 5 ;

        else
            % servo_motor = -1 ;
            turn = 1;
            start_turn = 0 ;

        end
     end
    % range_dist;
    dist_min = min(Range(180:360))
    if dist_min == Inf
        dist_mint = 1.75;
    end


    if turn == 1

        if 2 <= dist_min %&& range_dist <1.5 % 右转

            % servo_motor = -1 ; % 舵机右转60°
            w_l = +5 + 1;
            w_r = -5 + 1;

        elseif 1.5 <= dist_min &&  dist_min <2   % 直行
            % servo_motor = -1 ;
            w_l = +velocity ;
            w_r = +velocity ;


        elseif 0 <=dist_min && dist_min < 1.5 % 左转
            % servo_motor = -1 ;
            w_l = -velocity + 1;
            w_r = +velocity + 1;

        % elseif 1.0 < range_dist && range_dist < 2 % 例如，如果检测到距离小于0.5米的障碍物
        % 
        %     w_l = -velocity + 1;
        %     w_r = velocity + 1;
        %     servo_motor = -0.5 ;
        % end
        end

     % e_theata

        % if abs(e_theata) < 0.2   && turn==1  %1.8 < range_dist
        %     turn = 0;    %  出循环
        %     % servo_motor = 0;
        %     % start_turn 
        % end

    end


% ========= Finish Here ===================================================
% ========= Coursework 2 - Task 4 - Obstacle Avoidance - LiDAR ============
% =========================================================================
    
%% Acquire robot input
    % Acquire wheels velocity
    my_alg('right motor')   = w_r;
    my_alg('left motor')    = w_l;

    my_alg('turn')          = turn;
    my_alg('velocity')      = velocity;
    my_alg('start_turn')    = start_turn;
    my_alg('acc_time')      = acc_time;

end