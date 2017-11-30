classdef springmassAnimation
    %
    %    Ballbeam animation
    %
    %--------------------------------
    properties
        mass_handle
        length
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = springmassAnimation(P)
            self.length = P.length;
            
            figure(1), clf
            plot([0,0],[0,0],'k');
            hold on
            % initialize the ball and beam to initial conditions
            self=self.drawMass(P.z0);
            axis([-P.length/2, P.length/2, 0, P.length]);
        end
        %---------------------------
        function self=drawSpringmass(self, x)
            % Draw ballbeam is the main function that will call the functions:
            % drawBall and drawBeam to create the animation.
            % x is the system state
            z = x(1);       % Horizontal position of ball, m
            
            self=self.drawMass(z);
            drawnow
        end
        %---------------------------
        function self=drawMass(self, z)
            % Put code here to draw your beam.
            % Save your data points into the X and Y vectors to draw below
            X = [z-.5 z+.5 z+.5 z-.5];
            Y = [1 1 0 0];

            % this will only 'draw' the data points if needed, otherwise it
            % will just change the values in the handle.  It will still
            % update the animation, but is faster than a redraw).
            if isempty(self.mass_handle)
                self.mass_handle = fill(X, Y, 'b');
            else
                set(self.mass_handle,'XData', X, 'YData', Y);
            end
        end
    end
end