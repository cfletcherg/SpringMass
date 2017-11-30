% parameters are separated into two files -- one that are specific for the
% homework set (here) and the other that contains values that won't change.
% These can be combined into one, but be sure to track homework specific
% parameters carefully.

% inverted ballbeam - parameter file for hw8
springmassParam % general ballbeam parameters

% tuning parameters
% this could include your damping ration for inner/outer loop, desired rise
% times, separation value between inner loop and outer, et.

% saturation limits for beam angle and total force

% PD design for inner loop
% calculate the kp and kd gains for theta here...
Tr_z = 2;
wn_z = 2.2/Tr_z;
zeta = .707;
integrator_pole = -5;

% PD design
% calculate the kp and kd gains for z here...
P.kp_z = 10*zeta*wn_z; % kp
P.kd_z = 5*wn_z^2 - 3; % kd
P.ki_z = .5;

% Saturation limits for mass
P.F_max = 2;

%---------------------
% state space design
A = [0 1;... 
     -P.k/P.m -P.b/P.m];
B = [0;...
     1/P.m];
C = [1, 0];
A1 = [A, zeros(2,1);...
      -C, 0];
B1 = [B;...
      0];

% desired closed loop polynomial
wn_z = 2.2/Tr_z;
% gains for pole locations
des_char_poly = conv([1,2*zeta*wn_z,wn_z^2],poly(integrator_pole));
des_poles = roots(des_char_poly);

% is the system controllable?
K1 = place(A1,B1,des_poles);
P.K = K1(1:2);
P.ki = K1(3);
% print out your values to ensure calculated correctly
fprintf('\t kp_z: %f\n', P.kp_z)
fprintf('\t kd_z: %f\n', P.kd_z)
