function [my_alg] = ObstacleAvoidanceRange(my_alg, robot,delta_t)
%% Initialise all variables and settings
    if my_alg('is_first_time') 

    % =========================================================================
    %% ======== Coursework 2 - Task 3 - Obstacle Avoidance ====================
    % ========= Initialise variables Here =====================================

        % Set the initial rate of change of the servo motor angle
        my_alg('dTh')           = 0.01;
        % Set the initial angle of the servo motor to 0% (PWM signal -1 - 1
        % representing -60 - 60 degree)
        my_alg('servo motor')   = 0;
        % Define Maximum positive and negative bearings of servo
        my_alg('max_b')         = 0.9;
        % minimum recorded range at bearing
        my_alg('min_r')         =   0   ; %[inf;0];
        % direction set point
        my_alg('Th_s')          = 0;
        % falg to indicate the application of obstacle avoidance
        my_alg('F')             = false;

        my_alg('turn')          = false;  % 0 表示 没有开启转向模式
        my_alg('velocity')      = 0 ;
        my_alg('start_turn')      = 1 ;   % 1 表示没有开启转向初始化
        my_alg('acc_time')      = 0 ;

        my_alg('D_all')      = [0] ;

        my_alg('timer')    =  0 ;
    % ========= Finish initialisation Here ====================================
    % ========= Coursework 2 - Task 3 - Obstacle Avoidance ====================
    % =========================================================================
    end

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
    % falg to indicate the application of obstacle avoidance
    F                   = my_alg('F');
    % Range of servo motor
    Sev_r               = [-pi/3;pi/3];
    if my_alg('Sensor') == 1
    % Reading form sonar (m)
        range_dist      = my_alg('sonar');
    elseif my_alg('Sensor') == 3
    % Reading form laser (m)
        range_dist      = my_alg('laser');
    end
    % bearing of the sonar (the signal is PWM -1 - 1, representing -60
    % - 60 degrees)
    servo_motor         = my_alg('servo motor');
    % Rate of change of the servo motor angle
    dTh                 = my_alg('dTh');
    % Maximum positive and negative bearings of servo
    max_b               = my_alg('max_b');
    % minimum recorded range at bearing
    min_r               = my_alg('min_r');
    % direction set point
    Th_s                = my_alg('Th_s');

    turn            = my_alg('turn');
    velocity            = my_alg('velocity');  
    start_turn            = my_alg('start_turn');
    acc_time            = my_alg('acc_time');  
    D_all            = my_alg('D_all');   
    timer            = my_alg('timer');

    % =========================================================================
    %% ======== Coursework 2 - Task 3 - Obstacle Avoidance - laser/sonar ======
    % ========= Start Here ====================================================

    

    velocity = 10;
    %Write your code here
    % turn = 0;
    out_turn = 1;

     goal = [8  0]';
     e_x = goal(1) - S(1);
     e_y = goal(2) - S(2);
    % e_I = e_I + sqrt(e_x^2 + e_y^2);

    D_all = [D_all sqrt(e_x^2 + e_y^2)];
    

    e_theata = atan2(e_y,e_x) - S(3);
    turn ;
    start_turn;
    % delta_t = 0.02;

     if range_dist < 0.8  && start_turn == 1
         
        
        % servo_motor = -1 ;
        % dt   = toc(robot.timestamp);               % Compute Change in time
        % set_property(obj, 'timestamp' , tic);               % reset stopwatch
        acc_time = acc_time + delta_t;
        if acc_time<0.5
            w_l = -5 ;
            w_r = 5 ;

        else
            servo_motor = -1 ;
            turn = 1;
            start_turn = 0 ;
            
        end
     end
    range_dist;
    
    % turn

    if turn == 1
        
        if 1.0 <=range_dist %&& range_dist <1.5 % 右转
    
            servo_motor = -1 ; % 舵机右转60°
            w_l = +5 + 3;
            w_r = -5 + 3;
            
        elseif 0.4 <=range_dist && range_dist <1.0   % 直行
            servo_motor = -1 ;
            w_l = +velocity ;
            w_r = +velocity ;
            
    
        elseif 0 <=range_dist && range_dist < 0.4 % 左转
            servo_motor = -1 ;
            w_l = -5 + 3;
            w_r = +5 + 3;
            
        % elseif 1.0 < range_dist && range_dist < 2 % 例如，如果检测到距离小于0.5米的障碍物
        % 
        %     w_l = -velocity + 1;
        %     w_r = velocity + 1;
        %     servo_motor = -0.5 ;
        % end
        end


        % bug 0;
       % if abs(e_theata) < 0.2   && turn==1  %1.8 < range_dist
       %          turn = 0;    %  出循环
       %          servo_motor = 0;
       %          start_turn = 1;
       % end

     % e_theata

        % bug 1;

        
     if sqrt(e_x^2 + e_y^2) >  mean(D_all(end-100:end-1)) && turn==1 && timer>17 %1.8 < range_dist D_all(end-15:end-1)
                turn = 0;    %  出循环
                servo_motor = 0;
                start_turn = 1;
                disp(['distance', num2str(sqrt(e_x^2 + e_y^2))]);
                disp(['distance_prev', num2str(D_all(end-15:end-1))]);
     end

     % bug = 1;
     % switch bug
     %     case 0
     % 
     %        if abs(e_theata) < 0.2   && turn==1  %1.8 < range_dist
     %            turn = 0;    %  出循环
     %            servo_motor = 0;
     %            start_turn = 1;
     %        end
     %     case 1
     %        if sqrt(e_x^2 + e_y^2) < mean(D_all(end-5:end-1))   && turn==1  %1.8 < range_dist
     %            turn = 0;    %  出循环
     %            servo_motor = 0;
     %            start_turn = 1;
     %            disp(" sqrt(e_x^2 + e_y^2) "sqrt(e_x^2 + e_y^2) )
     %        end
     % end

    end

    
    % % Update servo motor position to scan for obstacles
    % if servo_motor >= max_b
    %     dTh = -dTh; % Reverse the direction of the servo motor
    % elseif servo_motor <= -max_b
    %     dTh = -dTh;
    % end
    % servo_motor = servo_motor + 10*dTh; % Update servo motor angle
    % ========= Finish Here ===================================================
    % ========= Coursework 2 - Task 3 - Obstacle Avoidance - laser/sonar ======
    % =========================================================================

%% update robot input
    % Acquire unpdated variables used For range sensors
    my_alg('dTh')           = dTh;
    my_alg('servo motor')   = servo_motor;
    % Acquire wheels velocity
    my_alg('right motor')   = w_r;
    my_alg('left motor')    = w_l;
    % Save other data
    my_alg('min_r')         = min_r;
    my_alg('F')             = F;
    my_alg('Th_s')          = Th_s;

    my_alg('turn')          = turn;
    my_alg('velocity')      = velocity;
    my_alg('start_turn')    = start_turn;
    my_alg('acc_time')      = acc_time;  
    my_alg('D_all')      = D_all;   
    my_alg('timer')      = timer + delta_t; 


end

