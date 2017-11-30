% Ballbeam Parameter File

% Physical parameters of the ballbeam known to the controller
P.m = 5;    % kg
P.k = 3; % N/m
P.b = .5; % N-s/m
P.length = 5;

% parameters for animation
% set here
P.force_max = 5;

% Initial Conditions
P.z0 = 0;         % initial box position, m
P.zdot0 = 0.0;             % initial ball velocity, m/s

% Simulation parameters
P.t_start = 0.0;  % Start time of simulation
P.t_end = 99.9;   % End time of simulation
P.Ts = 0.01;      % sample time for controller
P.t_plot = 0.1;   % the plotting and animation is updated at this rate

% dirty derivative parameters
% set here
P.sigma = .05;
P.beta = (2*P.sigma - P.Ts)/(2*P.sigma + P.Ts);

% uncertainty
P.uncertainty = 0;
P.alpha = .2;
P.randomUncertainty = (1 + 2*P.alpha*rand - P.alpha);

