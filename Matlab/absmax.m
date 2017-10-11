function res = absmax(wr, u)
%% Trivia
% Author            Laurens Znidarsic
% version           1
% Last edit         22-05-2017
% Last edit by      Laurens Znidarsic

%% Description 
% The function for slip (kappa) contains a denominator that is wheelspin
% times radius (wr) when the car is accelerating, and longitudinal speed
% (u) when the car is decelerating. The functions absmax evaluates the
% motion for acceleration or deceleration, and ouputs consequently.

%% Function
res = wr;
    if wr == u
        res = u;                     % neither
    elseif u > 0
        if wr > u                    % accelerating
            res = wr;
        elseif wr < u                 % braking 
            res = u;
        end
    elseif u < 0
        if wr < u                     % accelerating
            res = wr;
        elseif wr > u                 % braking
            res = u;
        end
    elseif u == 0
        res = wr;
    end
end
