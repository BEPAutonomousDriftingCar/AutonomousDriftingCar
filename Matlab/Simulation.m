%% Trivia
% Author            Laurens Znidarsic
% Version           5
% Last edit         22-05-2017
% Last edit by      Laurens Znidarsic

%% Description
% Runs the simulation of the simulink model 'CarSimulation' and presents it in a graphical environment 
% created by the function 'CarAnimation'

%% Script

clc; hold off; close all; clear all; 

options = simset('SrcWorkspace','current');
sim('CarModel.slx', [], options);

CarAnimation(S);

function CarAnimation(S)
%% Trivia
% Author            Laurens Znidarsic
% Version           4
% Last edit         10-07-2017
% Last edit by      Laurens Znidarsic

%% Description
% Starts with using the function to_linspace to convert all inut vectors to vectors with time step 0.05 
% (ideal framerate). It then uses input position vectors x_pos and y_pos, rotation vectors yaw and delta 
% to create a car-time matrix, where all the carmatices over time span [tout] are stored into matrix C 
% chronologically. It then uses function CarPlot to create a visual representation of every time step.
% After the animation, the function finally plots all the data associated with the motion in a combined figure.

%% Loading supportfunctions

%% Calculating the position and rotation vectors of the Car
tout = S(:,1); x_pos = S(:,11); y_pos = S(:,12); yaw = S(:,13); w = S(:,8:9); delta = S(:,10); kappa = S(:,18:19); 
alpha = S(:,20:21); Fxw = S(:,14:15); Fyw = S(:,16:17); v = S(:,2); u = S(:,3); r = S(:,2);

[t, x] = to_linspace(0.02, tout, x_pos);
[t, y] = to_linspace(0.02, tout, y_pos);
[t, th]  = to_linspace(0.02, tout, yaw);
[t, thw] = to_linspace(0.02, tout, delta);

C = zeros(9*(length(t)-1), 8);   % Initializing the C matrix (1 car has 9 rows)
n = 0.01;                        % See printing section
dt = t(2)-t(1);
l = 0;

for p = 1:length(t)
    
    % Selecting sections of 9 rows in C, and filling them with a the car
    % subjected to translations x(i) and y(i), and rotations (th(i) and
    % thw(i)
    imin = (p-1)*9+1;
    imax = p*9;
    C(imin:imax,:) = Car(x(p), y(p), 180*th(p)/pi-90, 180*thw(p)/pi);
        
    % Printing the progress in percent at the moment the for-loop crosses 
    % a 0.01 section  
    m = p/length(t);   
    if m >= l
       clc;
       l = l + 0.01;
       txt2 = sprintf('Converting to linear time spacing: 100 %% \nCalculating position matrix:       %.0f %%', l*100);
       disp(txt2)
    end
end

%%  Simulating the movement of the Car
figure('units','normalized','outerposition',[0 0 1 1]);                  % Create a new figure

ax = [min(x)-1,  max(x)+1, min(y)-1, max(y)+1, (max(y)-min(y)+2)/(max(x)-min(x)+2)]; 

for i = 1:length(t)-1
    imin = (i-1)*9+1;
    imax = i*9;
    Carplot(C(imin:imax, :),ax,i*dt);
    clc;
    pause(0.00001);
end

pause(1); close all;

figure('units','normalized','outerposition',[0 0 1 1]);
subplot(3,2,1);
plot(tout, r);
pbaspect([3 1 1]);
ylabel('yaw rate [rad/s]')
legend('Car')

subplot(3,2,2);
plot(tout, kappa(:,1), tout, kappa(:,2));
pbaspect([3 1 1]);
axis([0 max(tout) -2 2]);
ylabel('kappa [-]')
legend('Frontwheel', 'Rearwheel')

subplot(3,2,3);
plot(tout, alpha(:,1), tout, alpha(:,2), tout, delta);
pbaspect([3 1 1]);
ylabel('Angle [rad]')
legend('Front (slip)', 'Rear (slip)', 'steering')

subplot(3,2,4);
plot(tout, u, tout, v);
pbaspect([3 1 1]);
ylabel('Speed [m/s]');
legend('Longitudinal', 'Lateral');

subplot(3,2,5);
plot(tout, w(:,1), tout, w(:,2));
pbaspect([3 1 1]);
ylabel('omega [rad/s]')
legend('Frontwheel', 'Rearwheel')

subplot(3,2,6);
plot(tout, Fxw(:,1), tout, Fxw(:,2), tout, Fyw(:,1), tout, Fyw(:,2));
pbaspect([3 1 1]);
ylabel('F [N]')
legend('Front (x)', 'Rear (x)','Front (y)', 'Rear (y)')

    function C = Car(x, y, th1, th2)
    %% Trivia
    % Author            Laurens Znidarsic
    % Version           3
    % Last edit         04-04-2017
    % Last edit by      Laurens Znidarsic

    %% Description
    % Uses inputs x-position (x), y-position (y),          __________________
    % car angle (th1) and relative wheel angle (th2)      |_________6________|
    % to create the vectors of 9 blocks, which          %  __   |      |   __
    % are positioned in such a way that they form       % | 1|  |      |  | 2|
    % the car as depicted to the right.                 % |__|  |      |  |__|
    % This function outputs a 9x8 matrix, which         %  _____|      |_____
    % contains all the corner points of these blocks.   % |     |      |     |
    % The function Carplot can be used to give a        % |  8  |   5  |  9  |
    % graphical representation of this matrix.          % |     |      |     |
                                                        % |_____|      |_____|
                                                        %  __   |      |   __
                                                        % | 3|  |      |  | 4|
                                                        % |__|  |      |  |__|
                                                        %  _____|______|_____
                                                        % |________7_________|

    %% Forming posotion vectors for the individual blocks
    % Position vector = rotationmatrix * [position of block within car] +
    % [position of car]                                                 

    r1 = 0.05*rotz(th1)*[-2, 3, 0].' + [x; y; 0];    % Position vector of block 1           
    r2 = 0.05*rotz(th1)*[2, 3, 0].' + [x; y; 0];     % Position vector of block 2                
    r3 = 0.05*rotz(th1)*[-2, -3, 0].' + [x; y; 0];   % Position vector of block 3              
    r4 = 0.05*rotz(th1)*[2, -3, 0].' + [x; y; 0];    % Position vector of block 4              
    r5 = 0.05*rotz(th1)*[0, 0, 0].' + [x; y; 0];     % Position vector of block 5              
    r6 = 0.05*rotz(th1)*[0, 4.75, 0].' + [x; y; 0];  % Position vector of block 6            
    r7 = 0.05*rotz(th1)*[0, -4.75, 0].' + [x; y; 0]; % Position vector of block 7          
    r8 = 0.05*rotz(th1)*[-1.5, 0, 0].' + [x; y; 0];  % Position vector of block 8           
    r9 = 0.05*rotz(th1)*[1.5, 0, 0].' + [x; y; 0];   % Position vector of block 9                                                             

    %% Creating the line vectors for the individual blocks
    y1 = Block(0.1,0.05, r1, th1+th2);   % Left front wheel   
    y2 = Block(0.1,0.05, r2, th1+th2);   % right front wheel
    y3 = Block(0.1,0.05, r3, th1);       % Left rear wheel
    y4 = Block(0.1,0.05, r4, th1);       % Right rear wheel
    y5 = Block(0.45, 0.075, r5, th1);    % Base
    y6 = Block(0.025, 0.25, r6, th1);    % Front bumper
    y7 = Block(0.025, 0.25, r7, th1);    % Rear bumper
    y8 = Block(0.15, 0.075, r8, th1);    % Left side
    y9 = Block(0.15, 0.075, r9, th1);    % Right side

    %% Saving the vectors in a single matrix
    C = [[y1]; [y2]; [y3]; [y4]; [y5]; [y6]; [y7]; [y8]; [y9]];


        function s = Block(l,b,p,th)
        %% Description
        % This function requires 4 inputs: length (l), width (b), a position vector (p), and an
        % angle (th). It uses these to initialize a rotated block, consisting of 2 sides
        % (specifying 4 points) around the specified position vector.

        %% Initializing
        % Initializes a block with the specified length and width around the
        % origin
            whtl = [-b/2, l/2, 0];      
            whtr = [b/2, l/2, 0];
            whbl = [-b/2, -l/2, 0];
            whbr = [b/2, -l/2, 0];

        %% Rotating
        % Rotates the block with the specified angle
            R = rotz(th);
            whtl2 = R*whtl.';
            whtr2 = R*whtr.';
            whbl2 = R*whbl.';
            whbr2 = R*whbr.';

        %% Vectors
        % Creates a line vector (s) [[x1,x2,y1,y2],[x3,x4,y3,y4]] and translates it to point p
            right =  [[whtr2(1), whbr2(1)] + p(1), [whtr2(2), whbr2(2)] + p(2)];
            left =   [[whbl2(1), whtl2(1)] + p(1), [whbl2(2), whtl2(2)] + p(2)];

            s = [right, left];




        end
    end


    function Carplot(C, ax,t)
    %% Trivia
    % Author            Laurens Znidarsic
    % Version           3
    % Last edit         22-05-2017
    % Last edit by      Laurens Znidarsic

    %% Description
    % Creates a visual car representation of input matrix C, with axis 
    % dimensions specified in input ax.

    %% Plot initialisation
    clf;                                            % Clear previous figure                    
    hold on;                                        % Plot all vectors in  current figure
    pbaspect([1 ax(5) 1]);                          % Set axis ratio
    axis([ax(1), ax(2), ax(3), ax(4), 0 , 1]);      % Set axis limits

    %% Body

    % Put all x-coordinates of the body in the right order in an x-vector
    xc = [C(6,6), C(6,1), C(6,2), C(5,1), C(9,6), C(9,1), C(9,2), C(9,5), C(5,2), C(7,1), C(7,2), C(7,5), C(7,6), C(5,5), C(8,2), C(8,5), C(8,6), C(8,1), C(5,6), C(6,5), C(6,6)];
    % Put all y-coordinates of the body in the right order in an y-vector
    yc = [C(6,8), C(6,3), C(6,4), C(5,3), C(9,8), C(9,3), C(9,4), C(9,7), C(5,4), C(7,3), C(7,4), C(7,7), C(7,8), C(5,7), C(8,4), C(8,7), C(8,8), C(8,3), C(5,8), C(6,7), C(6,8)];
    % Plot x over y
    plot(xc,yc, 'r');

    %% Wheels 
    % Plot 4 wheels
    for i = 1:4
        % Put all x-coordinates of the wheel in the right order in an x-vector
        xw = [C(i,6), C(i,1), C(i,2), C(i,5), C(i,6)];
        % Put all y-coordinates of the wheel in the right order in an y-vector
        yw = [C(i,8), C(i,3), C(i,4), C(i,7), C(i,8)];
        % Plot x over y
        plot(xw,yw, 'b');
    end

    %% Time
    % Presents the simulation time on the screen. 
    txt = sprintf('Time elapsed:   %.2f s', t);
    xlabel(txt)
    end


    function [tout, dataout] = to_linspace(dt, tin, datain)
    % Description
    % Turn a (ineaqually) time-spaced data vector into a linear spaced time vector with step size dt. 
    % Can also be used to change the time step in linearly spaced vectors. 

    % Vector initialization
        tend = max(tin);
        tout = linspace(0,tend, tend/dt+1);
        dataout = zeros(length(tout),1);

    % Filling the linear spaced vectors
        for k = 1:(length(tout)-2)
            t_c = (k)*dt;
            dataout(k) = interp1(tin, datain, t_c);
        end
    end
end

