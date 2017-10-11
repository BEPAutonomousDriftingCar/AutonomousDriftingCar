function POS = Speed_Calculations(filename, cnt)
    %% Initialize data
    addpath(genpath('D:\Google Drive\Laurens\TU\BEP\Databin\170531'))
    load(filename);

    %% Initialize variables
    markers = 1;                        % plot marker size
 
    [par1, par2, par3, par4, par5, par6, par7, par8, par9] = textread('smoothpam.txt', '%f %f %f %f %f %f %f %f %f');
    ssgx = par1(cnt);                        % s_x smooth gain
    ssgy = par2(cnt);                        % s_y smooth gain
    ssgt = par3(cnt);                        % s_theta smooth gain

    vsgx = par4(cnt);                        % v_x smooth gain
    vsgy = par5(cnt);                        % v_y smooth gain
    vsgt = par6(cnt);                        % v_theta smooth gain

    asgx = par7(cnt);                        % a_x smooth gain
    asgy = par8(cnt);                        % a_y smooth gain
    asgt = par9(cnt);                       % a_theta smooth gain

    %% Correct the time
    simulationtime = actualtime - actualtime(1);
    timestep = zeros(length(simulationtime),1);
    for j = 2:length(simulationtime)
        timestep(j) = simulationtime(j) - simulationtime(j-1);
    end

    %% Filtering data
    % DO NOT USE WHEN MAKING TURNS LARGER THAN 90 DEGREES!!!
    %s_theta_f = bwftheta(s_theta);
    
    %% Fixing the jump in theta
    s_theta_f = s_theta; 
    for i = 1:(length(s_theta_f)-1)
        if s_theta_f(i+1)-s_theta_f(i) < -6
            s_theta_f((i+1):length(s_theta_f)) =  s_theta_f((i+1):length(s_theta_f)) + 2*pi;
        elseif s_theta_f(i+1)-s_theta_f(i) >6
            s_theta_f((i+1):length(s_theta_f)) =  s_theta_f((i+1):length(s_theta_f)) + 2*pi;
        end
    end
        
    %% Smooth out the raw data from Simulink
    % 'loess' because it can follow the data more precisely than 'rloess'
    fprintf('smoothing s_x...');
    s_x_smooth = smooth(simulationtime,s_x,ssgx,'rloess');      
    fprintf('done. smoothing s_y...');
    s_y_smooth = smooth(simulationtime,s_y,ssgy,'rloess');
    fprintf('done. smoothing theta...');
    s_theta_smooth = smooth(simulationtime,s_theta_f,ssgt,'rloess');
    fprintf('done\n');
    

    %% Calculate the speed
    v_x = s_v(s_x_smooth,timestep);
    v_y = s_v(s_y_smooth,timestep);
    v_theta = s_v(s_theta_smooth,timestep);

    %% Smooth out the speed data
    fprintf('smoothing v_x...');
    v_x_smooth = smooth(simulationtime,v_x,vsgx,'loess');
    fprintf('done. smoothing v_y...');
    v_y_smooth = smooth(simulationtime,v_y,vsgy,'loess');
    fprintf('done. smoothing v_theta...');
    v_theta_smooth = smooth(simulationtime,v_theta,vsgt,'loess');
    fprintf('done\n');

    %% Calculate the acceleration
    a_x = v_a(v_x_smooth,timestep);
    a_y = v_a(v_y_smooth,timestep);
    a_theta = v_a(v_theta_smooth,timestep);

    %% Smooth out the acceleration data
    fprintf('smoocvthing a_x...');
    a_x_smooth = smooth(simulationtime,a_x,asgx,'loess');
    fprintf('done. smoothing a_y...');
    a_y_smooth = smooth(simulationtime,a_y,asgy,'loess');
    fprintf('done. smoothing a_theta...');
    a_theta_smooth = smooth(simulationtime,a_theta,asgt,'loess');
    fprintf('done\n');

%     % Remove startup time from MoCap data. 
%     disp('Removing startup time... ')
%     i = 115;
    
%     ref = sqrt(((v_x_smooth(1))^2)+((v_y_smooth(1))^2));
%     while ref < 0.2
%         i = i + 1;
%         ref = sqrt((abs(v_x_smooth(i))^2)+(abs(v_y_smooth(i))^2));
%     end

%     disp('Check smoothingoperators')
%     figure(1);
%     subplot(3,3,1);
%     plot(simulationtime, s_x, simulationtime, s_x_smooth);
%     ylabel('x');
%     subplot(3,3,2);
%     plot(simulationtime, s_y, simulationtime, s_y_smooth);
%     ylabel('y');
%     subplot(3,3,3);
%     plot(simulationtime, s_theta_f, simulationtime, s_theta_smooth);
%     ylabel('th');
%     subplot(3,3,4);
%     plot(simulationtime, v_x, simulationtime, v_x_smooth);
%     ylabel('vx');
%     subplot(3,3,5);
%     plot(simulationtime, v_y, simulationtime, v_y_smooth);
%     ylabel('vy');
%     subplot(3,3,6);
%     plot(simulationtime, v_theta, simulationtime, v_theta_smooth);
%     ylabel('vth');
%     subplot(3,3,7);
%     plot(simulationtime, a_x, simulationtime, a_x_smooth);
%     ylabel('ax');
%     subplot(3,3,8);
%     plot(simulationtime, a_y, simulationtime, a_y_smooth);
%     ylabel('ay');
%     subplot(3,3,9);
%     plot(simulationtime, a_theta, simulationtime, a_theta_smooth);
%     ylabel('ath')
%     keyboard; close all;
    POS = [simulationtime, s_x_smooth, s_y_smooth, s_theta_smooth, v_x_smooth, v_y_smooth, v_theta_smooth, a_x_smooth, a_y_smooth, a_theta_smooth];
end