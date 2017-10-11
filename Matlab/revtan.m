function res = revtan(v, u)
    %% Trivia
    % Author            Laurens Znidarsic
    % Version           2
    % Last edit         10-07-2017
    % Last edit by      Laurens Znidarsic

    %% Description
    % The regular atan in matlab does not support the entire circle [-pi 0 pi],
    % revtan does.

    %% Script

    res = atan(v/u);

    if v == 0
        res = pi/2-sign(u)*pi/2;
    elseif u < 0     % linksonder
        res = sign(v)*pi + res;
    elseif u == 0
        res = sign(v)*pi/2;
    end
end
