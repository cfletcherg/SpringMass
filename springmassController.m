classdef springmassController < handle
    %
    %    This class inherits other controllers in order to organize multiple controllers.
    %
    %----------------------------
    properties
        zCtrl
        m
        k
        b
        Ts
        z_d1
        z_dot
        beta
        K
        kr
        ki
        limit
        integrator
        error_d1
    end
    %----------------------------
    methods
        %----------------------------
        function self = springmassController(P)
            self.m = P.m;
            self.k = P.k;
            self.b = P.b;
            self.Ts = P.Ts;
            self.z_d1 = 0.0;
            self.z_dot = P.zdot0;
            self.beta = P.beta;
            self.K = P.K;
            self.ki = P.ki;
            self.limit = P.F_max;
            self.integrator = 0.0;
            self.error_d1 = 0.0;
        end
        %----------------------------
        function F = u(self, y_r, y)
            % y_r is the referenced input
            % y is the current state
            z_r = y_r;
            z = y(1);
            self.differentiateZ(z);
            
            error = z_r - z;
            self.integrateError(error);
            % Implement your controller here...
            
            % You may choose to implement the PD control directly or call the
            % PDControl class.  The PDControl class will return a force output
            % for the given reference input and current state.
            % i.e. for the z-controller (already set up in the constructor)
            % call: z_force = self.zCtrl.PD(z_r, z, false);
            % For the theta controller call:
            %       theta_force = self.thetaCtrl.PD(theta_r, theta, false);
            % You will need to determine what the output is for these
            % controllers in relation to the block diagrams derived for the
            % inner and outer loop control.
            x = [z; self.z_dot];
            F_unsat = -self.K*x - self.ki*self.integrator;
            
            % compute the total force
            F = self.saturate(F_unsat);
            self.integratorAntiWindup(F, F_unsat);
        end
        %----------------------------
        function self = integratorAntiWindup(self, u_sat, u_unsat)
            self.integrator = self.integrator + self.Ts/self.ki*(u_sat - u_unsat);
        end
        %----------------------------
        function self = differentiateZ(self, z)
            self.z_dot = ...
                self.beta*self.z_dot...
                + (1-self.beta)*((z-self.z_d1) / self.Ts);
            self.z_d1 = z;
        end
        %----------------------------
        function self = integrateError(self, error)
            self.integrator = self.integrator + (self.Ts/2.0)*(error+self.error_d1);
            self.error_d1 = error;
        end
        %----------------------------
        function out = saturate(self,u)
            if abs(u) > self.limit
                u = self.limit*sign(u);
            end
            out = u;
        end
        
    end
end