function [P, Q]=tramoq(P1, P2, P3, tau, T, t)
    % Function that calculates the transformation (P - position, and Q - orientation) from P1 to P3 smoothing in P2 with Taylor method (quaternions)

    if (t<-T || t>T)
        % Out of allowed range
        disp('Parameter t out of range');
    else
        
        if (t<=-tau) % First segment (lineal)
            %% (1)!
        elseif (t>=tau) % Third segment (lineal)
            %% (2)!
        else % Second segment (smoothing)
            % Position interpolation
            %% (3)!

            % Orientation interpolation
            %% (4)!
        end

    end
end
            
           
            
            