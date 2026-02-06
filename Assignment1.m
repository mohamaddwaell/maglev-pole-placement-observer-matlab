clc;
clear all;

A = [0 1 0 ; 980 0 -2.8 ; 0 0 -100 ];
B = [0 ; 0 ; 100];
C = [1 0 0];
D = 0;

sysc = ss(A,B,C,D);

%% b)

%{
Ts = [0.001 0.005 0.01];

 for k = 1:length(Ts)
    sysd = c2d(sysc, Ts(k), 'zoh');
    disp(['Ts = ', num2str(Ts(k))])
    disp(sysd.A)
    disp(sysd.B)
 end
%}

%% c) 
x0 = [0;0;0];          % initial condition
Tf = 0.5;              % final time
u_step = 0.001;

% Continuous simulation
t = linspace(0,Tf,5000);
u_cont = u_step*ones(size(t));
[yc,tc,xc] = lsim(sysc,u_cont,t,x0);

Ts_vec = [0.001 0.005 0.01];

for i = 1:length(Ts_vec)
    Ts = Ts_vec(i);
    
    % Discretize
    sysd = c2d(sysc, Ts, 'zoh');
    G = sysd.A;
    H = sysd.B;
    
    % Discrete simulation
    kmax = floor(Tf/Ts)+1;
    xk = zeros(3,kmax);
    uk = u_step*ones(1,kmax);
    
    for k = 1:kmax-1
        xk(:,k+1) = G*xk(:,k) + H*uk(k);
    end
    
    % time vector for discrete points
    tk = (0:kmax-1)*Ts;
    
    % Plot continuous + discrete together
    figure;
    plot(tc, xc(:,1), 'k', 'LineWidth',2); hold on;
    stem(tk, xk(1,:), 'r','filled');
    grid on;
    xlabel('Time (s)');
    ylabel('\Delta h (m)');
    title(['Continuous vs Discrete response, Ts = ', num2str(Ts),' s']);
    legend('Continuous','Discrete');
end



%% d)

Ts_vec = [0.001 0.005 0.01];

% Continuous pole choice
pc = [-20 -25 -30];

for i = 1:length(Ts_vec)

    Ts = Ts_vec(i);

    % Discretize
    sysd = c2d(sysc,Ts,'zoh');
    Ad = sysd.A;
    Bd = sysd.B;

    % Map poles to discrete
    pd = exp(pc*Ts);

    % State feedback gain
    Kd = place(Ad,Bd,pd);

    disp('----------------------------------')
    disp(['Ts = ', num2str(Ts)])
    disp('Desired discrete poles:')
    disp(pd)
    disp('State feedback gain Kd:')
    disp(Kd)
end

%}

%{





%% e)

K = {[-324.9310  -10.3901   -0.2265], ...
     [-359.1578  -11.4832   -0.1351], ...
     [-397.8860  -12.7199   -0.0296]};

Ts = [0.001 0.005 0.01];
Tf = 1;

for i = 1:3

    % Discretize plant
    sysd = c2d(sysc, Ts(i), 'zoh');
    Ad = sysd.A;
    Bd = sysd.B;
    Cd = sysd.C;
    Dd = sysd.D;

    % Closed-loop discrete system
    Acl = Ad - Bd*K{i};
    sys_cl = ss(Acl, Bd, Cd, Dd, Ts(i));

    % Step response
    figure;
    step(sys_cl, Tf);
    grid on;
    title(['Discrete closed-loop response, T_s = ', num2str(Ts(i))]);
    xlabel('Time (s)');
    ylabel('\Delta h (m)');
end

%}

%% f)


%{
rng(1);

K = {[-324.9310  -10.3901   -0.2265], ...
     [-359.1578  -11.4832   -0.1351], ...
     [-397.8860  -12.7199   -0.0296]};

Ts = [0.001 0.005 0.01];

% Noise levels
sigma_w = 1e-4;   % state noise
sigma_v = 1e-3;   % output noise

Tf = 0.2;
dt = 1e-4;
t = 0:dt:Tf;
for i = 1:3

    x = zeros(3,length(t));
    u = zeros(1,length(t));

    Ts_i = Ts(i);
    Kd = K{i};
    k_sample = 1;

    for k = 1:length(t)-1

        % Sample controller
        if mod(k, round(Ts_i/dt)) == 0
            y_meas = C*x(:,k) + sigma_v*randn;   % noisy output
            u(k) = -Kd * x(:,k);                 % state feedback
            k_sample = k;
        else
            u(k) = u(k_sample);
        end

        % State noise
        w = sigma_w*randn(3,1);

        % Continuous integration (Euler)
        x(:,k+1) = x(:,k) + dt*(A*x(:,k) + B*u(k) + w);
    end
figure;
plot(t,x(1,:),'LineWidth',1.5); hold on;
plot(t, zeros(size(t)), '--k'); % optional reference line
grid on;
xlabel('Time (s)');
ylabel('\Delta h (m)');
title(['Noisy response, T_s = ', num2str(Ts_i),' s']);

end

%}


%{
%% g)
Ts_vec = [0.001 0.005 0.01];

K = {[-324.9310  -10.3901   -0.2265], ...
     [-359.1578  -11.4832   -0.1351], ...
     [-397.8860  -12.7199   -0.0296]};

%% Observer poles (faster than controller poles)
po = [-60 -70 -80];   % continuous observer poles

%% Continuous simulation parameters
Tf = 0.2;
dt = 1e-4;
t = 0:dt:Tf;
N = length(t);

%% Loop over sampling times
for i = 1:length(Ts_vec)

    Ts = Ts_vec(i);
    Kd = K{i};

    % Discretize plant
    sysd = c2d(ss(A,B,C,0),Ts,'zoh');
    Ad = sysd.A;
    Bd = sysd.B;

    % Discrete observer poles
    zd_obs = exp(po*Ts);

    % Observer gain
    Ld = place(Ad',C',zd_obs)';

    % Initialize states
    x  = zeros(3,N);    % true state
    xh = zeros(3,N);    % estimated state
    u  = zeros(1,N);    % control input

    % Initial conditions (important!)
    x(:,1)  = [1e-3; 0; 0];   % small position offset
    xh(:,1) = [0; 0; 0];      % wrong initial estimate

    sample_step = round(Ts/dt);
    last_sample = 1;

    %% Simulation loop
    for k = 1:N-1

        if mod(k,sample_step) == 0

            % Measurement (NO NOISE)
            y = C*x(:,k);

            % Control using estimated state
            u(k) = -Kd*xh(:,k);

            % Observer update
            xh(:,k+1) = Ad*xh(:,k) + Bd*u(k) ...
                        + Ld*(y - C*xh(:,k));

            last_sample = k;

        else
            % Hold values between samples
            u(k) = u(last_sample);
            xh(:,k+1) = xh(:,k);
        end

        % Continuous plant integration
        x(:,k+1) = x(:,k) + dt*(A*x(:,k) + B*u(k));
    end

    %% Plot results
    figure;
    plot(t,x(1,:), 'k','LineWidth',1.7); hold on;
    plot(t,xh(1,:), '.','LineWidth',1.5);
    grid on;
    xlabel('Time (s)');
    ylabel('\Delta h (m)');
    legend('True state','Estimated state');
    title(['Observer-based control (No Noise), T_s = ',num2str(Ts),' s']);

end

%}

%{
%% h)


%% Sampling times
Ts_vec = [0.001 0.005 0.01];

%% Feedback gains (discrete-design applied to continuous plant)
K = {[-324.9310  -10.3901   -0.2265], ...
     [-359.1578  -11.4832   -0.1351], ...
     [-397.8860  -12.7199   -0.0296]};

%% Observer poles (continuous-time equivalent)
po = [-60 -70 -80];

%% Simulation settings
Tf = 0.4;        % total time
dt = 1e-4;       % integration step for continuous plant
t = 0:dt:Tf;
N = length(t);

%% Loop over each sampling period
for i = 1:length(Ts_vec)
    Ts = Ts_vec(i);
    Kd = K{i};

    % Discretize plant for observer design
    sysd = c2d(ss(A,B,C,0), Ts, 'zoh');
    Ad = sysd.A;
    Bd = sysd.B;

    % Discrete observer poles
    zd_obs = exp(po*Ts);
    Ld = place(Ad',C',zd_obs)';

    % Initialize
    x  = zeros(3,N);  % true state
    xh = zeros(3,N);  % estimated state
    u  = zeros(1,N);  % control input

    % Initial conditions: step in position
    x(:,1)  = [1e-3; 0; 0];  % small initial step
    xh(:,1) = [0;0;0];       % observer starts at 0

    sample_step = round(Ts/dt);  % number of dt steps per sample
    last_sample = 1;

    %% Simulation loop
    for k = 1:N-1
        if mod(k,sample_step) == 0
            % Measurement (no noise)
            y = C*x(:,k);

            % Control using observer estimate
            u(k) = -Kd*xh(:,k);

            % Discrete observer update
            xh(:,k+1) = Ad*xh(:,k) + Bd*u(k) + Ld*(y - C*xh(:,k));

            last_sample = k;
        else
            % Hold control and estimate
            u(k) = u(last_sample);
            xh(:,k+1) = xh(:,k);
        end

        % Continuous plant integration (Euler)
        x(:,k+1) = x(:,k) + dt*(A*x(:,k) + B*u(k));
    end

    %% Plot Δh (position) with observer points
    figure;
    plot(t, x(1,:), 'k', 'LineWidth',1.8); hold on;
    sample_idx = 1:sample_step:N;
    stem(t(sample_idx), xh(1,sample_idx), 'r','filled');
    grid on;
    xlabel('Time (s)');
    ylabel('\Delta h (m)');
    title(['Continuous step response with observer, T_s = ',num2str(Ts),' s']);
    legend('True state \Delta h', 'Estimated \Delta ĥ (observer)');

    %% Optional: plot control input
    figure;
    plot(t,u,'b','LineWidth',1.5);
    grid on;
    xlabel('Time (s)');
    ylabel('Control input u(t)');
    title(['Control input, T_s = ',num2str(Ts),' s']);
end

%}

