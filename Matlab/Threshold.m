function res = Threshold(val, ref)

    if val < ref && val >= 0
        res = ref;
    elseif val > -ref && val < 0
        res = -ref;
    else 
        res = val;
    end
    
end