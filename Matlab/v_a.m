% Position to speed calculation
function a_data = v_a(speed_data,time)   % v_a is depended on velocity data and time step
    a_dif = diff(speed_data);
    a_dif = [0; a_dif];
    a_data = zeros(length(speed_data),1);
    for i = 1:length(speed_data)
        a_data(i) = a_dif(i)/time(i);
    end
end