%% Trivia 
% Author            Laurens Znidarsic, Robert van Wijk
% Version           2
% Last edit         10-07-2017
% Last edit by      Laurens Znidarsic

%% Description
% DataAnalyse reads experimental test data from the MotionCapture system
% and the HallEffect sensors from the test rig. It uses several functions
% to trim and edit these data matrices into and evenly timescaled Matrix,
% with the complete set of motion and force values over time, in the order:
%      1  2  3  4   5    6    7   8   9   10   11 12 13  14   15   16   17   18  19  20  21
% S = [t, v, u, r, v_d, u_d, r_d, wf, wr, del, x, y, th, Fxf, Fxr, Fyf, Fyr, kf, kr, af, ar]

%% Script

n = 24;                                                             % number of tests
addpath(genpath('D:\Google Drive\Laurens\TU\BEP\Databin\170531'))   % location of testdata

for i = 1:n
    
    clc; close all;
    [t1, t2, t3, t4, w1, w2, w3, w4] = readhalldata(sprintf('steer%.0f.txt', i)); % load and even out HallSensor data                       % Read wheelspin
    mc = Speed_Calculations(sprintf('steer%.0f_fixed.mat', i), i);                % load MoCap data                                               % Read positions (x y th)
    deltload = load(sprintf('delta_Steer%0.f.mat', i));                           % Load steering angle data
    deltin = deltload.delta(2,:).'; clear deltload;                               % Snip out steering data
    S1 = to_car_frame(mc, t1, t2, t3, t4, w1, w2, w3, w4, deltin); clear deltin;  % Calculate car position (v u r)
    S2 = to_tyre(S1);                                                             % Calculate Forces and alpha and kappa
    S = [S1, S2]; clear S1; clear S2;                                             % Put everything in 1 Matrix
    CarAnimation(S);                                                              % Simulate Car motion
    save(sprintf('S%0.f.mat', i),'S');                                            % Save State matrix
    
end
    
    function [t1, t2, t3, t4, w1, w2, w3, w4] = readhalldata(filename1)
        %% Description
        % Reads the Hall effect sensor data from a .txt file, and sorts it
        % into a [t1, t2, t3, t4, w1, w2, w3, w4] matrix
        addpath(genpath('D:\Google Drive\Laurens\TU\BEP\Databin')); % Location of the data file 
        t1 = [0]; w1 = [0];                                         % 1st wheel                                    
        t2 = [0]; w2 = [0];                                         % 2st wheel    
        t3 = [0]; w3 = [0];                                         % 3rd wheel    
        t4 = [0]; w4 = [0];                                         % 4th wheel    
        [n, w, t] = textread(filename1, '%f %f %f');                % Creates a [sensor #, w, t] matrix 
        t = (t./1000);                                              % Time in seconds (hall gives ms)
        for i = 1:length(n)                                         % Sort by sensor #                            
            if n(i) == 1
                t1 = [t1;t(i)];
                w1 = [w1;w(i)];
            elseif n(i) == 2
                t2 = [t2;t(i)];
                w2 = [w2;w(i)];
            elseif n(i) == 3
                t3 = [t3;t(i)];
                w3 = [w3;w(i)];
             elseif n(i) == 4
                t4 = [t4;t(i)];
                w4 = [w4;w(i)];   
            end
        end
    end
    
    function S_out = to_car_frame(S_mc, hst1, hst2, hst3, hst4, hsw1, hsw2, hsw3, hsw4, delta)
        %% Description
        % Uses real world coordinates and car orientation to derive car
        % speed [v, u, r] and acceleration [v_dot, u_dot, r_dot].
        [t, sc, sw, deltfixed] = timefix();                         
        w = [(sw(:,3) + sw(:,4)).*pi, (sw(:,1) + sw(:,2)).*pi];
        sc(:,3) = -sc(:,3)-pi/2;
        u = cos(sc(:,3)).*sc(:,4) - sin(sc(:,3)).*sc(:,5);
        v = sin(sc(:,3)).*sc(:,4) + cos(sc(:,3)).*sc(:,5);
        u_dot = cos(sc(:,3)).*sc(:,7) - sin(sc(:,3)).*sc(:,8);
        v_dot = sin(sc(:,3)).*sc(:,7) + cos(sc(:,3)).*sc(:,8);
        r = sc(:,6);
        r_dot = sc(:,9);
        S_out = [t.',v,u,r,v_dot,u_dot, r_dot,w, deltfixed, sc(:,1), sc(:,2),sc(:,3)];


        function [t, sc, sw, deltfix] = timefix()
            % Equals the endtime for all vectors, and creates even time
            % spacing.
            reftime = min(min(max(S_mc(:,1)), max(hst1)), min(min(max(hst2),max(hst3)),max(hst4))); %Find lowest end time
            [t, xt] = to_linspace(0.02, reftime, S_mc(:,1), S_mc(:,2));
            [t, yt] = to_linspace(0.02, reftime, S_mc(:,1), S_mc(:,3));
            [t, tht] = to_linspace(0.02, reftime, S_mc(:,1), S_mc(:,4));
            [t, vx] = to_linspace(0.02, reftime, S_mc(:,1), S_mc(:,5));
            [t, vy] = to_linspace(0.02, reftime, S_mc(:,1), S_mc(:,6));
            [t, vth] = to_linspace(0.02, reftime, S_mc(:,1), S_mc(:,7)); 
            [t, ax] = to_linspace(0.02, reftime, S_mc(:,1), S_mc(:,8));
            [t, ay] = to_linspace(0.02, reftime, S_mc(:,1), S_mc(:,9));
            [t, ath] = to_linspace(0.02, reftime, S_mc(:,1), S_mc(:,10));
            [t, w1] = to_linspace(0.02, reftime, hst1, hsw1);
            [t, w2] = to_linspace(0.02, reftime, hst2, hsw2);
            [t, w3] = to_linspace(0.02, reftime, hst3, hsw3);
            [t, w4] = to_linspace(0.02, reftime, hst4, hsw4);         
            sc = [xt,yt,(tht-3*pi/180),vx,vy,vth,ax,ay,ath];
            sw = [w1,w2,w3,w4];
            deltfix = delta(1:length(t));
            
            function [tout, dataout] = to_linspace(dt, tend, tin, datain)
                % Description
                % Turn a (ineaqually) time-spaced data vector into a linear spaced time vector with step size dt. 
                % Can also be used to change the time step in linearly spaced vectors. 

                % Vector initialization
                    tout = linspace(0,tend, round(tend/dt+1));
                    dataout = zeros(length(tout),1);

                % Filling the linear spaced vectors

                for i = 1:(length(tout))
                        t_c = (i-1)*dt;
                        dataout(i) = interp1(tin,datain, t_c);          
                end
            end
        end
    end
    
    function T = to_tyre(C) 
        v = C(:,2); u = C(:,3); r = C(:,4); v_dot = C(:,5); u_dot = C(:,6); 
        r_dot = C(:,7); w = C(:,8:9); del = C(:,10);
        F = to_F;               % Calculate Forces
        kappa = to_kappa;       % Calculate kappa
        alpha = to_alpha;       % Calculate alpha
        T = [F, kappa, alpha];  % Store in Tyre matrix
        
        function F = to_F()
            %% Description
            % Uses the acceleration and the equations of motion for the car to reverse-engineer
            % the x and y forces on the wheels
            clc; disp('Calculating Forces...');
            Fy = to_Fy;
            Fxw = (3.2.*(u_dot-v.*r) - tan(del).*Fy(:,1))./(cos(del) + 1 + tan(del).*sin(del)).*[1,1]; 
            Fyw = [(Fy(:,1) - Fxw(:,1).*sin(del))./cos(del), Fy(:,2)];
            F = [Fxw, Fyw];

            function Fy = to_Fy()
                Fyr = 0.5.*(3.2.*(v_dot+u.*r)-(r_dot.*0.048875./0.167));    %M = I*theta_dot
                Fyf = (r_dot.*0.048875./0.167) + Fyr;                        %F = m*a
                Fy = [Fyf,Fyr];
            end

        end

        function alpha = to_alpha()
            %% Description
            % Uses longitudinal and lateral speed and the slip angle
            % equations to reverse-engineer slip angles.
            clc; disp('Calculating alpha... ');
            alpha = zeros(length(v),2);
            for i = 1:length(v)
                alphaf = del(i) - revtan(v(i)+0.167*r(i), u(i));       % slipangle frontwheel
                alphar = - revtan(v(i)-0.167*r(i), u(i));                 % slipangle rearwheel
                alpha(i,:) = [alphaf, alphar];
            end
            clc;
            function res = revtan(vt, ut)
                %% Description
                % Extends the range of the functionality of atan() to [-pi
                % pi] instead of [-pi/1, pi/2]
                res = atan(vt/ut);
                if vt == 0
                    res = pi/2-sign(ut)*pi/2;
                elseif ut < 0    
                    res = sign(vt)*pi + res;
                elseif ut == 0
                    res = sign(vt)*pi/2;
                end
            end  
        end

        function kappa = to_kappa()
            %% Description
            % Uses wheelspin and wheelcenter velocity to calculate slip
            % ratio's
            clc; disp('calculating kappa...');
            % To bicycle model
            wf = (w(:,1));
            wr = (w(:,2));
      
            kappa = zeros(length(v),2);
            vxw = zeros(length(v),1);
            for g = 1:length(v)
                vxw(g) = cos(del(g))*u(g) + sin(del(g))*(v(g) + r(g)*0.167); % Front wheelcenter velocity
                if u_dot(g) == 0
                    kappa(g,:) = acceleration(g);
                elseif u_dot(g) > 0 && u(g) >= 0     %accelerating
                    kappa(g,:) = acceleration(g);
                elseif u_dot(g) < 0 && u(g) <= 0     %accelerating
                    kappa(g,:) = acceleration(g);
                elseif u_dot(g) < 0 && u(g) > 0      %braking
                    kappa(g,:) = braking(g);
                elseif u_dot(g) > 0 && u(g) < 0      %braking
                    kappa(g,:) = braking(g);
                end
            end
            
            function kappa = acceleration(g)            %When accelerating, divide by wheelspin
                kappaf = (wf(g)*0.048 - vxw(g))/(wf(g)*0.048);
                kappar = (wr(g)*0.048 - u(g))/(wr(g)*0.048);
                kappa = [kappaf, kappar];
            end

            function kappa = braking(g)                 % When braking, divide by wheelcentervelocity
                kappaf = (wf(g)*0.05 - vxw(g))/(vxw(g));
                kappar = (wr(g)*0.05 - u(g))/(u(g));
                kappa = [kappaf, kappar];
            end   
        end
    end
    

