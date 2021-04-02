% SCRIPT_BicycleModel_Integrator_SS_and_TF.m
% Plots the results of 3 outputs of a simulink model: the State-space
% representation, the all-integrator form, and the transfer function form.
% Note that all three are the same (which is the point). It also tests this
% across 3 vehicles, and can create a movie animation of the results at the
% end.
%
% This script was originally written on 2007_06_25 by S. Brennan
% Questions or comments? sbrennan@psu.edu 

% Revision history:
% 2020_09_17 - rewrite of the movie section for speed
% 2021_03_31 - cleaned up code for class consumption, moving plotting
% routine out of calculations so students can see process more clearly

%% Prep the workspace
close all;  % close all the plots

flag_make_movies = 1; % Set this flag to 1, and it it makes a movie of results

%% Inputs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   _____                   _       
%  |_   _|                 | |      
%    | |  _ __  _ __  _   _| |_ ___ 
%    | | | '_ \| '_ \| | | | __/ __|
%   _| |_| | | | |_) | |_| | |_\__ \
%  |_____|_| |_| .__/ \__,_|\__|___/
%              | |                  
%              |_| 
% See: http://patorjk.com/software/taag/#p=display&f=Big&t=Inputs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Set core simulation inputs: the speed, and steering amplitude
U = 20;  % U is forward velocity of vehicle in longitudinal direction, [m/s] (rule of thumb: mph ~= 2* m/s)

% Items used to define steering inputs
steering_amplitude_degrees = 2; % 2 degrees of steering amplitude for input sinewave
Period = 3; % Units are seconds. A typical lane change is about 3 to 4 seconds based on experimental highway measurements

% Define items used to determine how long to run sim, number of time
% points, etc.
TotalTime = 1.5*Period;  % This is how long the simulation will run. Usually 1.5 times the period is enough.
deltaT = 0.01; % This is the time step of the simulation. See the "Model Settings" submenu in Simulink to see where this variable is used.
N_timeSteps = ceil(TotalTime/deltaT) + 1; % This is the number of time steps we should have



%% Fill in the vehicle parameters. 
% Use a structure array so we can have several vehicles

% Approximately a Ford Taurus
vehicle(1).m          = 1031.9; % kg
vehicle(1).Iz         = 1850; % kg-m^2
vehicle(1).a          = 0.9271; % Distance from front axle to CG, in meters
vehicle(1).b          = 1.5621;  % Distance from rear axle to CG, in meters
vehicle(1).Caf        = -77500; % N/rad;
vehicle(1).Car	      = -116250; % N/rad;

% A random vehicle (race car tires)
vehicle(2).m          = 1670; % kg
vehicle(2).Iz         = 2100; % kg-m^2
vehicle(2).a          = 0.99; % meters
vehicle(2).b          = 1.7;  % meters
vehicle(2).Caf        = -123200; % N/rad;
vehicle(2).Car	      = -104200; % N/rad;

% A variation on the random vehicle (go-cart like)
vehicle(3).m          = 167; % kg
vehicle(3).Iz         = 210; % kg-m^2
vehicle(3).a          = 0.99; % meters
vehicle(3).b          = 1.7;  % meters
vehicle(3).Caf        = -70200; % N/rad;
vehicle(3).Car	      = -80200; % N/rad;

% The number of vehicles is the length of the structure array
N_vehicles = length(vehicle);

%% Main code
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   __  __       _       
%  |  \/  |     (_)      
%  | \  / | __ _ _ _ __  
%  | |\/| |/ _` | | '_ \ 
%  | |  | | (_| | | | | |
%  |_|  |_|\__,_|_|_| |_|
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This runs the codes to predict core behavior within SIMULINK, looping
% through each vehicle.
% For this simulation set, the core behavior is
% V: lateral speed
% r: rotational rate of vehicle around z-axis

% Initialize all the arrays with Not-a-Number (nan). When plotting, any
% values that are nan will be left empty.
all_X_ss   = nan(N_timeSteps,N_vehicles);
all_Y_ss   = nan(N_timeSteps,N_vehicles);
all_phi_ss = nan(N_timeSteps,N_vehicles);
all_r_ss   = nan(N_timeSteps,N_vehicles);
all_V_ss   = nan(N_timeSteps,N_vehicles);

all_X_tf   = nan(N_timeSteps,N_vehicles);
all_Y_tf   = nan(N_timeSteps,N_vehicles);
all_phi_tf = nan(N_timeSteps,N_vehicles);
all_r_tf   = nan(N_timeSteps,N_vehicles);
all_V_tf   = nan(N_timeSteps,N_vehicles);

all_X_int   = nan(N_timeSteps,N_vehicles);
all_Y_int   = nan(N_timeSteps,N_vehicles);
all_phi_int = nan(N_timeSteps,N_vehicles);
all_r_int   = nan(N_timeSteps,N_vehicles);
all_V_int   = nan(N_timeSteps,N_vehicles);


% Loop through all the vehicles, simulating the trajectory of each within
% the for loop
for vehicle_i=1:N_vehicles
    % Print to the console which vehicle we are working on
    fprintf(1,'Working on vehicle: %d\n',vehicle_i);
    
    % Fill in the parameters needed by the equations below and by the
    % simulation for it to run
    m = vehicle(vehicle_i).m;
    Iz = vehicle(vehicle_i).Iz;
    a = vehicle(vehicle_i).a;
    b = vehicle(vehicle_i).b;
    L = a+b;
    Caf = vehicle(vehicle_i).Caf;
    Car = vehicle(vehicle_i).Car;
    
    % Fill in the 2-state state-space matrices, A and B used in the
    % Simulink simulation
    A = [(Caf+Car)/(m*U) (a*Caf-b*Car)/(m*U)-U;
        (a*Caf-b*Car)/(Iz*U) (a^2*Caf+b^2*Car)/(Iz*U)];
    B = [-Caf/m -Car/m; -a*Caf/Iz b*Car/Iz];
    
    % Fill in the transfer functions used in the simulation
    num_yawrate = [-a*Caf/Iz Caf*Car*L/(m*U*Iz)];
    num_latvel = [-Caf/m b*Caf*Car*L/(m*U*Iz)+a*Caf*U/Iz];
    den = [1 -((Caf+Car)/(m*U)+(Caf*a^2+Car*b^2)/(Iz*U)) (Caf*Car*L^2/(m*Iz*U^2)+(a*Caf-b*Car)/Iz)];
    
    % Run the simulation in SIMULINK
    sim('MODEL_BicycleModel_Integrator_SS_and_TF.slx', TotalTime);
      

    % Save the results in a big array (for plotting in next part) 
    % Before saving, we need to check if the full vector is shorter than
    % expected length of N_timeSteps
    if length(t) ~= N_timeSteps
        warning('More time was spent than expected in the simulation. Keeping only the expected time portion.')        
    end    

    % Keep the shorter of either the actual length, or expected length:
    shorter_index = min(N_timeSteps,length(t));
    
    % Fill in the data arrays
    all_X_ss(1:shorter_index,vehicle_i)   = X_ss(1:shorter_index);
    all_Y_ss(1:shorter_index,vehicle_i)   = Y_ss(1:shorter_index);
    all_phi_ss(1:shorter_index,vehicle_i) = phi_ss(1:shorter_index);
    all_r_ss(1:shorter_index,vehicle_i)   = r_ss(1:shorter_index);
    all_V_ss(1:shorter_index,vehicle_i)   = V_ss(1:shorter_index);

    all_X_tf(1:shorter_index,vehicle_i)   = X_tf(1:shorter_index);
    all_Y_tf(1:shorter_index,vehicle_i)   = Y_tf(1:shorter_index);
    all_phi_tf(1:shorter_index,vehicle_i) = phi_tf(1:shorter_index);
    all_r_tf(1:shorter_index,vehicle_i)   = r_tf(1:shorter_index);
    all_V_tf(1:shorter_index,vehicle_i)   = V_tf(1:shorter_index);
    
    all_X_int(1:shorter_index,vehicle_i)   = X_int(1:shorter_index);
    all_Y_int(1:shorter_index,vehicle_i)   = Y_int(1:shorter_index);
    all_phi_int(1:shorter_index,vehicle_i) = phi_int(1:shorter_index);
    all_r_int(1:shorter_index,vehicle_i)   = r_int(1:shorter_index);
    all_V_int(1:shorter_index,vehicle_i)   = V_int(1:shorter_index);
end

%% Plot the results
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   _____  _       _   _   _             
%  |  __ \| |     | | | | (_)            
%  | |__) | | ___ | |_| |_ _ _ __   __ _ 
%  |  ___/| |/ _ \| __| __| | '_ \ / _` |
%  | |    | | (_) | |_| |_| | | | | (_| |
%  |_|    |_|\___/ \__|\__|_|_| |_|\__, |
%                                   __/ |
%                                  |___/ 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Start the plotting. All three of the plots for each situation should
% agree with each other (otherwise the equations are not consistent)

% Loop through each of the vehicles
for vehicle_i=1:N_vehicles
    % Plot the yaw rate
    h1 = figure(99);
    hold on;
    set(h1,'Name','Yawrate')
    plot(...
        t,all_r_ss(:,vehicle_i),'ro',...
        t,all_r_int(:,vehicle_i),'b',...
        t,all_r_tf(:,vehicle_i),'gx'); 
    legend('SS','int','tf');
    xlabel('Time (sec)'); 
    ylabel('Yawrate (rad/sec)');
    
    % Plot the lateral velocity
    h2 = figure(88);
    hold on;
    set(h2,'Name','LatVel')
    plot(...
        t,all_V_ss(:,vehicle_i),'ro',...
        t,all_V_int(:,vehicle_i),'b',...
        t,all_V_tf(:,vehicle_i),'gx'); 
    legend('SS','int','tf');
    xlabel('Time (sec)'); 
    ylabel('Lateral Velocity (m/sec)');
    
    % The XY Plots
    h3 = figure(77);
    hold on;
    set(h2,'Name','XYposition')
    plot(...
        all_X_ss(:,vehicle_i),all_Y_ss(:,vehicle_i),'ro',...
        all_X_int(:,vehicle_i),all_Y_int(:,vehicle_i),'b',...
        all_X_tf(:,vehicle_i),all_Y_tf(:,vehicle_i),'gx'); 
    legend('SS','int','tf');
    xlabel('X position from all-integrator [m]'); 
    ylabel('Y position from all-integrator model [m]');
end

%% Make the movies
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
%   __  __       _     _______ _          __  __            _      
%  |  \/  |     | |   |__   __| |        |  \/  |          (_)     
%  | \  / | __ _| | _____| |  | |__   ___| \  / | _____   ___  ___ 
%  | |\/| |/ _` | |/ / _ \ |  | '_ \ / _ \ |\/| |/ _ \ \ / / |/ _ \
%  | |  | | (_| |   <  __/ |  | | | |  __/ |  | | (_) \ V /| |  __/
%  |_|  |_|\__,_|_|\_\___|_|  |_| |_|\___|_|  |_|\___/ \_/ |_|\___|
%                                                                  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
% The following section makes an animation of the vehicles using stick
% figures

if flag_make_movies == 1
    
    % Sometimes the movie creation process is interrupted, leaving the file
    % open from the previous run so we might have to close the file first. 
    try
        close(newVid)
    catch
    end
    
    % Set up a figure
    hFigAnimation = figure(46466); % Opens up the figure with an arbitrary number
    set(hFigAnimation,'Name','XY_Animation'); % Puts a name of the figure
    clf;  % Clear the figure
    set(gcf,'DoubleBuffer','on');  % Makes animations more flicker-free
    set(gcf, 'Position', get(0, 'Screensize')); % Makes the figure fill the screen
    axis equal; % Makes the axes have the same scale in X and Y
    grid minor; % Adds a grid to the figure
    hold on;  % Let's us plot multiple times on same figure
    
    % Label the axes
    xlabel('Global X (m)');
    ylabel('Global Y (m)');
    
    % Set up the axis to be within the limits of the data. We use a buffer
    % size to give a bit of white space around the plots
    max_X = max(max(all_X_int));
    max_Y = max(max(all_Y_int));
    min_X = min(min(all_X_int));
    min_Y = min(min(all_Y_int));
    buffer_spacing = 0.05*(max_X-min_X);
    axis([min_X-buffer_spacing max_X+buffer_spacing min_Y-buffer_spacing max_Y+buffer_spacing]);
    
    % Set video file paramters
    outputfilename = 'vehicleLanechange.mp4';
    fps = 30;  % This is the frames per second. 20 to 30 per second is typical
    % 2020_09_17 - SNB commented out because avifile is deprecated function
    % now. Replaced with VideoWriter
    % mov = avifile('3vehicles_lanechange.avi','Quality',100)
    newVid = VideoWriter(outputfilename,'MPEG-4');  % Use MPEG-4 encoding
    newVid.FrameRate = fps;  % Set frame rate to be the fps variable
    newVid.Quality = 100;  % Set quality to be high
    open(newVid); % Opens the movie for writing
    
    % Prep the plots fill all data with NaN so that nothing plots at first
    % and leaves an empty matrix that, when we fill later causes the plot
    % to be shown.
    plotted_all_X = all_X_int*NaN;
    plotted_all_Y = all_Y_int*NaN;
    h_plot = plot(plotted_all_X(:,1),plotted_all_Y(:,1),'b',...
        plotted_all_X(:,2),plotted_all_Y(:,2),'b',...
        plotted_all_X(:,3),plotted_all_Y(:,3),'b',...
        NaN*[0;0],NaN*[0;0],'r',...
        NaN*[0;0],NaN*[0;0],'r',...
        NaN*[0;0],NaN*[0;0],'r');
    
    % Put labels on all the vehicles, and save the handles for each
    h_vehicle_text{1} = text(all_X_int(1,1),all_Y_int(1,1),'Vehicle 1');
    h_vehicle_text{2} = text(all_X_int(1,2),all_Y_int(1,2),'Vehicle 2');
    h_vehicle_text{3} = text(all_X_int(1,3),all_Y_int(1,3),'Vehicle 3');
    
    % Calculate how many frames we need for the for loop
    speed_up_ratio = 1;  % How much to speed up the movie relative to real-time. For example, a number 5 here means 5 times faster than real-time
    duration_in_seconds = t(end)-t(1);  % How long the simulation lasted (duration)
    total_frames = fps*duration_in_seconds; % How many frames we expect
    plot_every = round(N_timeSteps/total_frames)*speed_up_ratio;
    
    multiplier_slip = 10;  % How much to exaggerate the slip angle (so we can better see side-slip)
    multiplier_size = 1; % How much to exaggerate the size of the vehicle (so we can see it on big trajectories)

    % Loop through the time steps, skipping intervals to get the desired fps and
    % speed-up ratio
    for j=1:plot_every:N_timeSteps
        
        % Update the arrays that are plotted
        plotted_all_X(1:j,:) = all_X_int(1:j,:);
        plotted_all_Y(1:j,:) = all_Y_int(1:j,:);
        
        % Draw the vehicles
        for vehicle_i=1:N_vehicles
            
            % Calculate the new vehicle position
            alpha_r = all_phi_int(j,vehicle_i)-multiplier_slip*(all_V_int(j,vehicle_i)-vehicle(vehicle_i).b*all_r_int(j,vehicle_i))/U;
            bodyx_start = all_X_int(j,vehicle_i)+multiplier_size*vehicle(vehicle_i).a*cos(alpha_r);
            bodyy_start = all_Y_int(j,vehicle_i)+multiplier_size*vehicle(vehicle_i).a*sin(alpha_r);
            bodyx_end = all_X_int(j,vehicle_i)-multiplier_size*vehicle(vehicle_i).b*cos(alpha_r);
            bodyy_end = all_Y_int(j,vehicle_i)-multiplier_size*vehicle(vehicle_i).b*sin(alpha_r);
            
            % Update the plots using "set" command, which is MUCH faster
            % than re-plotting
            set(h_plot(vehicle_i),'XData',plotted_all_X(:,vehicle_i));
            set(h_plot(vehicle_i),'YData',plotted_all_Y(:,vehicle_i));
            set(h_plot(vehicle_i+N_vehicles),'Xdata',[bodyx_start; bodyx_end]);
            set(h_plot(vehicle_i+N_vehicles),'Ydata',[bodyy_start; bodyy_end]);
            
            % Update the text position as well
            set(h_vehicle_text{vehicle_i},'Position',[all_X_int(j,vehicle_i),all_Y_int(j,vehicle_i) 0]);
            
        end
        
        % Force our drawings to be shown (not buffered)
        drawnow;
        
        % Save the current axes to a movie
        frame = getframe(gca); % Grab the plot to create a frame of the movie
        writeVideo(newVid,frame); % Write the frame to the movie
        
    end
    close(newVid);
end

