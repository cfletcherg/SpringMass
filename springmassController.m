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
        %z_dot
        beta
        F_d1
        K
        kr
        ki
        A
        B
        C
        L
        limit
        integrator
        error_d1
        x_hat
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
            %self.z_dot = P.zdot0;
            self.beta = P.beta;
            self.K = P.K;
            self.ki = P.ki;
            self.limit = P.F_max;
            self.integrator = 0.0;
            self.error_d1 = 0.0;
            self.A = P.A;
            self.B = P.B;
            self.C = P.C;
            self.L = P.L;
            self.x_hat = [0.0;0.0];
            self.F_d1 = 0.0;
        end
        %----------------------------
        function F = u(self, y_r, y)
            % y_r is the referenced input
            % y is the current state
            z_r = y_r;
            
            % update the observer and extract z_hat
            self.updateObserver(y);
            z_hat = self.x_hat(1);
            
            % integrate error
            error = z_r - z_hat;
            self.integrateError(error);
            
            % compute equilibrium force F_e at old spot
            F_e = 0;
            
            % Compute the sate feedback controller
            F_unsat = -self.K*self.x_hat - self.ki*self.integrator;
            
            % compute the total force
            F = self.saturate(F_unsat + F_e);
            self.updateForce(F);
            
            %self.integratorAntiWindup(F, F_unsat);
        end
        %----------------------------
        function self = updateObserver(self, y_m)
            % compute equilibrium torque tau_e at old angle
            z_hat = self.x_hat(1);
            F_e = 0;

            N = 10;
            for i=1:N
                self.x_hat = self.x_hat + self.Ts/N*(...
                    self.A*self.x_hat...
                    + self.B*(self.F_d1-F_e)...
                    + self.L*(y_m-self.C*self.x_hat));
            end
        end
        %----------------------------
        function self = updateForce(self, F)
            self.F_d1 = F;
        end
        %----------------------------
%         function self = integratorAntiWindup(self, u_sat, u_unsat)
%             self.integrator = self.integrator + self.Ts/self.ki*(u_sat - u_unsat);
%         end
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