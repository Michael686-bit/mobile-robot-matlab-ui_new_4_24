function my_alg = pathMPC(my_alg, robot)
% -------------------------------------------------------------------------
if my_alg('is_first_time')
    %% =========================Initialization=====================
    % =====================Initial the variables here======================
    my_alg('path')=load('trajectory_s.txt');% reference path (s shape)
    path=my_alg('path');              
    my_alg('dc_motor_signal_mode') = 'omega_setpoint';     % change if necessary to 'voltage_pwm'
    my_alg('right motor')   = 0;    % angular velocity set point for the right motor speed
    my_alg('left motor')    = 0;    % angular velocity set point for the left motor speed
    my_alg('t_loop')        = tic;  % operationg time for one loop
    my_alg('xr')=path(:,1); % reference path in x
    my_alg('yr')=path(:,2); % reference path in y
    my_alg('thetar')=path(:,3); % reference path in theta
    my_alg('path_x')=[]; % simulated robot path in x
    my_alg('path_y')=[]; % simulated robot in y
    my_alg('theta')=[];  % simulated robot in theta

    % =======================Personal parameter============================
    my_alg('NowU') = [0;0];
    my_alg('U_Desired') = [0;0];
    my_alg('k') = 1;
    my_alg('U_Desired') = [0;0];
    my_alg('V_All')     = [];
    my_alg('W_All')     = [];
    my_alg('wr_All')    = [];
    my_alg('wl_All')    = [];
    my_alg('Error_All') = [];
    my_alg('Error')     = 0;
    %% =======================Localizer============================
    my_alg('localizer') = LocalizationClass(...
        'method', 'wv',...
        'robot', robot,...
        'pose', [0 0 0],... % should always be the same as the initial 2D pose in mobile robot simulator GUI
        ...
        ...
        'ext_sensor_label', 'range');
    % =====================================================================
end

% Localization update
omegas_map = containers.Map({'right wheel', 'left wheel'},...
    [my_alg('right encoder'), my_alg('left encoder')]);
    my_alg('localizer') = Deadreckoning(my_alg('localizer'),omegas_map);
% Plotting section
my_alg = add_plot(my_alg, 'plot(my_alg(''localizer''))');     % plot pose estimation in the main figure
my_alg = add_plot(my_alg, 'plot(my_alg(''path_x''),my_alg(''path_y''),''k--'')');% plot robot passed path
my_alg('ref') = add_plot(my_alg, 'plot(my_alg(''xr''),my_alg(''yr''),''r'')');%plot premade reference path

%% Variables (add your own variables if needed) 
t_sampling          = 0.15;            % time period between two reference points (select a suitable time)
S                   = my_alg('localizer').pose; % current robot pose
w_r                 = my_alg('right motor');    % omega set point for right motor
w_l                 = my_alg('left motor');     % omega set point for left motor
L                   = 2*robot.components_tree.get('left motor').transformation(2,end);% length between wheels
r                   = robot.components_tree.get('left wheel').shape.diameter/2;% wheels radiu
my_alg('is_done')   = false;

% =======================Personal parameter============================
xr                  = my_alg('xr');
yr                  = my_alg('yr');
thetar              = my_alg('thetar');
N                   = 10;
my_alg('U_Delta')   = zeros(2*N,1);
Delta_u             = my_alg('U_Delta');
k                   = my_alg('k');
U                   = my_alg('U_Desired');
%======================================================================
%% build your system model and MPC controller in a control loop here
% =====================================================================
if(my_alg('k')<size(my_alg('path'),1))% If the tracking has not been finished
    if ((toc(my_alg('t_loop'))>=t_sampling))
        my_alg('t_loop')=tic;
     %% =======================Linearization model============================
        % Target point;
        Target_X = xr(k);
        Target_Y = yr(k);
        Target_Theta = thetar(k);
        Target_V = U(1,1);
        Target_W = U(2,1);
        
        % State space
        A = eye(3)+t_sampling*[0 0 -Target_V*sin(S(3));0 0 Target_V*cos(S(3)); 0 0 0];
        B = t_sampling*[cos(S(3)) 0;sin(S(3)) 0;0 1];
        C = eye(3);
        D = [0 0;0 0;0 0];
    
    %% =======================MPC controller============================
        StateNum = 3; % The number of the state
        InputNum = 2; % The number of the input
        OutputNum = 3; % The number of the output
        % U_weight = 0.1*diag([1 1]); % The weight of the U       
        % Y_weight = 1*diag([1.2 1.8 1.5]); % The weight of the Y     good parameter   for  circle
        % 

        U_weight = 0.1*diag([1 1]); % The weight of the Sbar
        Y_weight = 1*diag([1.2 1.8 1.6]); % The weight of the Qbar  1.4bad
        
       
        ConstrainMatrix = [1/r L/2*r;-1/r -L/2*r;1/r -L/2*r;-1/r L/2*r];
        L_Matrix = kron(eye(N), ConstrainMatrix);
        C_Matrix = repmat(13.75, 4*N, 1);
        
        R = zeros(3*N,1); % Reference martrix [3N*1]
    
        % =============Calculate the reference matrix during the process============
        if k+N < size(my_alg('path'),1)
            for i = 0:N-1
                 NowReference = [xr(k+i);yr(k+i);thetar(k+i)];
                 R((3*i)+1:(3*i+3),1) = NowReference;
            end
        % ===============Calculate the reference matrix near the end=============
        else 
            for i = 0:N-1
                if k+i<size(my_alg('path'),1)
                    NowReference = [xr(k+i);yr(k+i);thetar(k+i)];
                else
                    NowReference = [xr(size(my_alg('path'),1));yr(size(my_alg('path'),1));thetar(size(my_alg('path'),1))];
                end
                 R((3*i)+1:(3*i+3),1) = NowReference;
            end
        end
        
        % =======================Initial parameters for H and f matrix============================
        c = (eye(3)-A)*S + B*U;
        Lambda = zeros(N*StateNum,StateNum);Lambda(1:StateNum,:)=A;
        Phi = zeros(N*StateNum, N*InputNum);
        C=zeros(N*StateNum,1);C(1:StateNum,:)=c+ B*U;
        tmp =eye(StateNum);%tmp =A^i

        % =======================Calculate Phi, Lambda, C=======================
        for i=2 :N
            rows =StateNum*i-StateNum+1 :StateNum*i;
            Phi(rows,:)=[tmp*B,Phi(rows-StateNum,1:end-InputNum)];
            tmp = tmp * A;
            C(rows,:)=C(rows-StateNum,:)+tmp *(B*U + c);
            Lambda(rows,:)=tmp *A;
        end
    
        % =======================Calculate Q and S matrix=======================
        q = Y_weight*eye(OutputNum); % Weights on output deviation from setpoint
        Q = sparse(kron(eye(N),q));
        s = U_weight*eye(InputNum); % Weights on input deviation from setpoint
        S_Matrix = kron(diag([2*ones(1,N-1),1]),s);
        S_Matrix = S_Matrix + kron(diag([-ones(1,N-1)],-1),s);
        S_Matrix = S_Matrix + kron(diag([-ones(1,N-1)],1),s);
    
        E=[U_weight*U; zeros((N-1)*InputNum, 1)];
        
        % =======================Calculate H and f matrix=======================
        H = Phi'*Q*Phi + S_Matrix;
        H=(H+H')/2;
        f = Phi'*Q*(Lambda*S + C - R) - E;
    
        % =======================Calculate desired U=======================
        my_alg('U_Delta') = quadprog(H, f, L_Matrix, C_Matrix, [], [], [], [], [],optimset('display','off'));
        Delta_u = my_alg('U_Delta');
        my_alg('U_Desired') = my_alg('U_Desired') + Delta_u(1:2,1);

        % =======================Record tracking error=======================
        my_alg('Error') = sqrt((S(1)-Target_X)^2 + (S(2)-Target_Y)^2 + (S(3)-Target_Theta)^2);

        % ============Calculate desired angular speed of the wheel============
        trans_matrix = [r/2 r/2;r/L -r/L];
        v_matrix = my_alg('U_Desired');
        w = inv(trans_matrix) * v_matrix;
        w_r = w(1,1);
        w_l = w(2,1);

        my_alg('k') = my_alg('k') +1;

    end
    pause(0.001);
else
    my_alg('is_done') = true;% Tracking finished
end
    
% =====================================================================
if my_alg('is_done')
%% make your plots here

    figure(2);
    hold on;
    plot(my_alg('wr_All'), '-r', 'LineWidth', 1.5);
    plot(my_alg('wl_All'), '-b', 'LineWidth', 1.5);
    legend('right wheel velocity wr', 'left wheel velocity wl');
    xlabel("Sampling times")
    ylabel("velocity")


    hold off;

    figure(3);
    hold on;
    plot(my_alg('V_All'), '-r', 'LineWidth', 1.5);
    plot(my_alg('W_All'), '-b', 'LineWidth', 1.5);
    legend('Value of velocity V', 'Angular velocity w');
    xlabel("Sampling times")
    ylabel("velocity")
    hold off;

    figure(4);
    plot(my_alg('Error_All'), '-r', 'LineWidth', 1.5);
    legend('Tracking error');
    xlabel("Sampling times")
    ylabel("Tracking error")

    figure(5)
    % plot(my_alg('path_x'),my_alg('path_y') , '-b', 'LineWidth', 2);
    scatter(my_alg('path_x'),my_alg('path_y'), 20, 'blue', 'filled');
    legend('Path of robot');
    % xlim([-1, 1]); % 设置 x 范围为 0 到 5      circle
    % ylim([-0.5, 1.5]); % 设置 y 范围为 -1 到 1

    xlim([-1, 1]); % 设置 x 范围为 0 到 5          S
    ylim([-0.5, 3]); % 设置 y 范围为 -1 到 1

end


%% ====================update datas=========================================
my_alg('right motor')   = w_r;
my_alg('left motor')    = w_l;
my_alg('path_x')        = [my_alg('path_x') my_alg('localizer').pose(1)];
my_alg('path_y')        = [my_alg('path_y') my_alg('localizer').pose(2)];
my_alg('theta')         = [my_alg('theta')  my_alg('localizer').pose(3)];

% ========================Personal data====================================
my_alg('V_All')         = [my_alg('V_All') U(1,1)];
my_alg('W_All')         = [my_alg('W_All') U(2,1)];
my_alg('wr_All')        = [my_alg('wr_All') w_r];
my_alg('wl_All')        = [my_alg('wl_All') w_l];
my_alg('Error_All')     = [my_alg('Error_All') my_alg('Error')];
return