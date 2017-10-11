% specific for theta position in a straight test
% filters theta for data values outside 0 to pi (large deviations)
% DO NOT USE WHEN MAKING TURNS LARGER THAN 90 DEGREES!!!
function data = bwftheta(variabledata)
    for k = 1:length(variabledata)
        if variabledata(k) > pi || variabledata(k) < 0
            variabledata(k) = variabledata(k-1);
        else
            variabledata(k) = variabledata(k);
        end
    end
    data = variabledata;
end