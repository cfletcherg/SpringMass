clear all
springmassParamHW10;  % load parameters

% instantiate springmass, controller, and reference input classes 
% Instantiate Dynamics class
springmass = springmassDynamics(P);  
ctrl = springmassController(P);  

amplitude = .5; % amplitude of reference input
frequency = .02; % frequency of reference input
reference = signalGenerator(amplitude, frequency);  

% set disturbance input
disturbance = 0.0; % 0.01;

% instantiate the data plots and animation
dataPlot = plotData(P);
animation = springmassAnimation(P);
% observerPlot = plotObserverData(P);

% main simulation loop
t = P.t_start;  % time starts at t_start
while t < P.t_end  
    % Get referenced inputs from signal generators
    ref_input = reference.square(t);
    % Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot;
    while t < t_next_plot % updates control and dynamics at faster simulation rate
        u = ctrl.u(ref_input, springmass.outputs());  % Calculate the control value
        springmass.propagateDynamics(u);  % Propagate the dynamics
        t = t + P.Ts; % advance time by Ts
    end
    % update animation and data plots
    animation.drawSpringmass(springmass.states);
    springmass.states
    dataPlot.updatePlots(t, ref_input, springmass.states, u);
%   observerPlot.updatePlots(t, springmass.states, ctrl.x_hat);
end


