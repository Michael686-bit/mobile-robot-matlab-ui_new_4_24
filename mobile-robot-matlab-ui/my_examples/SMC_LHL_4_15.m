function my_alg = driveSMC_template(my_alg,robot)
% Initialization
if my_alg('is_first_time')
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%% Change the trajectory here %%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    my_alg('traj') = load('trajectory_line.txt');
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    my_alg('t_loop')        = tic;
    my_alg('k')=1;
    my_alg('right motor')   = 0;
    my_alg('left motor')    = 0;
    my_alg('vc')    = 0;
    my_alg('wc')    = 0;
    my_alg('xr_all') = [];
    my_alg('yr_all') = [];
    my_alg('thetar_all') = [];
    my_alg('vr_all') = [];
    my_alg('wr_all') = [];
    my_alg('vc_all') = [];
    my_alg('wc_all') = [];
    my_alg('xe_all') = [];
    my_alg('ye_all') = [];
    my_alg('thetae_all') = [];
    my_alg('s1_all') = [];
    my_alg('s2_all') = [];
    my_alg('wR_all') = [];
    my_alg('wL_all') = [];
    
    
    my_alg('localizer') = LocalizationClass(...
        'method', 'wv',...  % 'wv' is dead-reckoning localisation alg
        'robot', robot,...  % insert the robot object
        'pose', [0 0 0],... % Initial robot pose (must be the same as in GUI)
        ... % Not specified for dead-reckoning
        ... % Not specified for dead-reckoning
        'ext_sensor_label', 'range');   % specify sensor 'range' -> sonar/Laser
    
end

omegas_map = containers.Map({'right wheel', 'left wheel'},...
    [my_alg('right encoder'), my_alg('left encoder')]);

my_alg('localizer') = Deadreckoning(my_alg('localizer'),omegas_map);

% Compute change in time (dt)
dt = 0.15;

traj=my_alg('traj');
vc= my_alg('vc');
wc= my_alg('wc');

if(my_alg('k')<size(my_alg('traj'),1))
    k = my_alg('k')
	
    % Desired trajectory
    x_d =traj(k,1);
    y_d = traj(k,2);
    theta_d = traj(k,3);
    v_d = traj(k,4);
    w_d = traj(k,5);
    v_d_dot = traj(k,6);
    w_d_dot = traj(k,7);
	
    if ((toc(my_alg('t_loop'))>=dt))
        my_alg('t_loop')=tic;
        % Robot pose estimation [(m) (m) (rad)]'
        S                   = my_alg('localizer').pose;
        % right wheel angular velocity (rad/s)
        w_r                 = omegas_map('right wheel');
        % Left wheel angular velocity (rad/s)
        w_l                 = omegas_map('left wheel');
        % From the selected robot model call the distance between the two wheels(m)
        l                   = 2*robot.components_tree.get('left motor').transformation(2,end);
        % From the selected robot model call the radius of the wheel (m)
        r                   = robot.components_tree.get('left wheel').shape.diameter/2;
        
        my_alg('is_done')   = false;
        
        % =====================================================================
        %% ==== Coursework 1 - Task 5 - Sliding Mode Control ===============
        % ===== Start Here ====================================================
        % Real robot velocities
        vr = (w_r+w_l)*r/2;
        wr = (w_r-w_l)*r/l;
        

        Q1=5;   % 直线 
        Q2=1;
        P1=0.01;
        P2=1;
        k0=1;
        k1=3.5;
        k2=1;


        % Q1=2;
        % Q2=2;
        % P1=0.2;
        % P2=0.2;
        % k0=2;
        % k1=2;
        % k2=2;

        % Q1=2;
        % Q2=2;
        % P1=0.2;
        % P2=0.2;
        % k0=4;
        % k1=3;
        % k2=0.2;

        % Q1=1;   % 
        % Q2=1;
        % P1=0.1;
        % P2=1;
        % k0=1;
        % k1=1;
        % k2=1;


        % Q1=0.9 ;  % circle
        % Q2=1   ;
        % P1=0.1;
        % P2=0.1;
        % k0=3.2;
        % k1=0.5;
        % k2=2;

%         10  k0
% 0.3  k1  0.25line  0.35 circle
% 3.5 k2
% 0.2 P1
% 0.2 P2
% 1   Q1
% 1   Q2
        
        % Q1=1 ;  % circle
        % Q2=1   ;
        % P1=0.2;
        % P2=0.2;
        % k0=10;
        % k1=0.3;
        % k2=3.5;

        Q1=5 ;  % circle best 5
        Q2=1   ;
        P1=0.01;
        P2=0.1;   % best 0.1
        k0=3.2;   % best 3.2
        k1=0.5;    %best 0.5
        k2=2;




        % Compute the errors in position and heading
        x_e = cos(theta_d)*(S(1)-x_d)+sin(theta_d)*(S(2)-y_d);
        y_e = -sin(theta_d)*(S(1)-x_d)+cos(theta_d)*(S(2)-y_d);
        theta_e = S(3) - theta_d;

        % Normalize theta_e to the range [-pi, pi]
        %theta_e = wrapToPi(theta_e);
        
        xe_dot = -v_d+vr*cos(theta_e)+y_e*w_d;
        % ye_dot = vr*sin(S(3))-x_e*w_d;
        ye_dot = vr*sin(theta_e)-x_e*w_d;
        thetae_dot = wr-w_d;

        % Compute the sliding surfaces s1 and s2
        s1 = xe_dot + k1 * x_e;
        s2 = thetae_dot + k2 * theta_e + k0 * y_e;

        
        % Compute the control signals vc' and wc'
        vc_dot = (-Q1*s1 - P1*sign(s1) - k1*xe_dot + v_d_dot + vr*sin(theta_e)*thetae_dot - ye_dot*w_d - w_d_dot*y_e)/cos(theta_e);
        wc_dot = -Q2*s2 - P2*sign(s2) - k0*ye_dot + w_d_dot - k2*thetae_dot;

        % Integrate to get vc and wc
        vc = vc + vc_dot * dt;
        wc = wc + wc_dot * dt;

        % Compute wheel angular velocities
        wc_r = (vc + l * wc) / r;
        wc_l = (vc - l * wc) / r;

        % wc_r,wc_l - right and left wheel angular velocitiy setpoints
		
        % ===== Finish Here ===================================================
        % ===== Coursework 1 - Task 5 - Sliding Mode Control ===============
        % =====================================================================
        

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%% Do not change the code below %%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        
        % Increment time step
        
        my_alg('k') = my_alg('k') + 1;
        
        my_alg('vc')    = vc;
        my_alg('wc')    = wc;
        
        % Apply the control to the real robot
        my_alg('right motor')   = wc_r;
        my_alg('left motor')    = wc_l;
        
        % Save data for plotting
        my_alg('xr_all') = [my_alg('xr_all') S(1)];
        my_alg('yr_all') = [my_alg('yr_all') S(2)];
        my_alg('thetar_all') = [my_alg('thetar_all') S(3)];
        
        my_alg('vr_all') = [my_alg('vr_all') vr];
        my_alg('wr_all') = [my_alg('wr_all') wr];
        my_alg('vc_all') = [my_alg('vc_all') vc];
        my_alg('wc_all') = [my_alg('wc_all') wc];
        
        my_alg('xe_all') = [my_alg('xe_all') x_e];
        my_alg('ye_all') = [my_alg('ye_all') y_e];
        my_alg('thetae_all') = [my_alg('thetae_all') theta_e];
        
        my_alg('s1_all') = [my_alg('s1_all') s1];
        my_alg('s2_all') = [my_alg('s2_all') s2];
        
        my_alg('wR_all') = [my_alg('wR_all') w_r];
        my_alg('wL_all') = [my_alg('wL_all') w_l];
    end
    pause(0.001)
else
    figure(1)
    plot(traj(:,1),traj(:,2),':r','LineWidth',2);
    hold on;
    axis equal;
    plot(my_alg('xr_all'), my_alg('yr_all'), '-b', 'LineWidth', 2);
    legend('XY desired', 'XY robot')
    
    figure(2);
    plot(traj(:,4),':r','LineWidth',2);
    hold on;
    plot(my_alg('vr_all'), '-b', 'LineWidth', 2);
    plot(my_alg('vc_all'), '-g', 'LineWidth', 2);
    legend('V desired', 'V robot', 'V control')
    
    figure(3);
    plot(traj(:,5),':r','LineWidth',2);
    hold on;
    plot(my_alg('wr_all'), '-b', 'LineWidth', 2);
    plot(my_alg('wc_all'), '-g', 'LineWidth', 2);
    legend('W desired', 'W robot', 'W control')
    
    figure(4);
    plot(my_alg('xe_all'), '-r', 'LineWidth', 2);
    hold on;
    plot(my_alg('ye_all'), '-b', 'LineWidth', 2);
    plot(my_alg('thetae_all'), '-g', 'LineWidth', 2);
    legend('x error', 'y error', 'theta error')
    
    figure(5);
    plot(my_alg('s1_all'), '-b', 'LineWidth', 2);
    legend('Surface s1')
    
    figure(6);
    plot(my_alg('s2_all'), '-b', 'LineWidth', 2);
    legend('Surface s2');
    
    figure(7);
    plot(my_alg('wR_all'), '-b', 'LineWidth', 2);
    hold on;
    plot(my_alg('wL_all'), '-r', 'LineWidth', 2);
    legend('Wheel Angular Velocities');
    
    my_alg('is_done') = 1;
end

return