clc;
clear all;

%% System definition
A = [0 1 0; 980 0 -2.8; 0 0 -100];
B = [0; 0; 100];
C = [1 0 0];
D = 0;

sysc = ss(A,B,C,D);

%% Discrete feedback gains for different Ts
K = {[-324.9310  -10.3901   -0.2265], ...
     [-359.1578  -11.4832   -0.1351], ...
     [-397.8860  -12.7199   -0.0296]};

Ts = [0.001 0.005 0.01];   % sampling periods

%% Noise parameters
sigma_w = 1e-4;   % state noise
sigma_v = 1e-3;   % output noise

%% Simulation parameters
Tf = 0.2;          % final time
dt = 1e-4;         % integration step
t = 0:dt:Tf;
n = length(t);

%% Nominal response (no noise, for comparison)
x_nom = zeros(3,n);
u_nom = zeros(1,n);
u_step = 0;  % no external input, just feedback

for k = 1:n-1
    u_nom(k) = 0; % open-loop response (could also include feedback if desired)
    x_nom(:,k+1) = x_nom(:,k) + dt*(A*x_nom(:,k) + B*u_nom(k));
end

%% Simulation loop for each Ts
for i = 1:3
    Ts_i = Ts(i);
    Kd = K{i};
    
    x = zeros(3,n);      % states
    u = zeros(1,n);      % control input
    k_sample = 1;        % last sampled index
    
    for k = 1:n-1
        % Sample controller at discrete times
       if mod(k, round(Ts_i/dt)) == 0
    % Use full noisy state for state-feedback
    x_noisy = x(:,k) + sigma_w*randn(3,1);  % add process noise here
    u(k) = -Kd * x_noisy;                   % correct dimensions
    k_sample = k;
     else
    u(k) = u(k_sample);                       % hold last control
        end
        
        % State noise
        w = sigma_w*randn(3,1);
        
        % Euler integration
        x(:,k+1) = x(:,k) + dt*(A*x(:,k) + B*u(k) + w);
    end
    
    %% Plotting
    figure('Name',['Noisy response, Ts = ', num2str(Ts_i),' s']);
    
    subplot(3,1,1)
    plot(t, x_nom(1,:), 'k--', 'LineWidth',1.5); hold on;
    plot(t, x(1,:), 'r', 'LineWidth',1.2);
    grid on;
    ylabel('\Delta h (m)');
    legend('Nominal','Noisy');
    
    subplot(3,1,2)
    plot(t, x_nom(2,:), 'k--', 'LineWidth',1.5); hold on;
    plot(t, x(2,:), 'g', 'LineWidth',1.2);
    grid on;
    ylabel('\Delta \dot h (m/s)');
    
    subplot(3,1,3)
    plot(t, x_nom(3,:), 'k--', 'LineWidth',1.5); hold on;
    plot(t, x(3,:), 'b', 'LineWidth',1.2);
    grid on;
    xlabel('Time (s)');
    ylabel('\Delta i (A)');
    
    sgtitle(['Noisy continuous response with discrete gain, Ts = ', num2str(Ts_i),' s']);
end
