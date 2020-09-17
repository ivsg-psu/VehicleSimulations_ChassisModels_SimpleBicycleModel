close all;
U = 20;  % U is forward velocity of vehicle in longitudinal direction, [m/s] (rule of thumb: mph ~= 2* m/s)
make_movies = 1;

% Approximately a Ford Taurus
vehicle(1).m          = 1031.9; % kg
vehicle(1).Iz         = 1850; % kg-m^2
vehicle(1).a          = 0.9271; % Distance from front axle to CG, in meters
vehicle(1).b          = 1.5621;  % Distance from rear axle to CG, in meters
vehicle(1).Caf        = -77500; % N/rad;
vehicle(1).Car	      = -116250; % N/rad;


% vehicle(1).m          = 1670; % kg
% vehicle(1).Iz         = 2100; % kg-m^2
% vehicle(1).a          = 0.99; % meters
% vehicle(1).b          = 1.7;  % meters
% vehicle(1).Caf        = -123200; % N/rad;
% vehicle(1).Car	      = -104200; % N/rad;

% Core behavior is
% V: lateral speed
% r: rotational rate of vehicle around z-axis

%%%%%%% V,r Plot %%%%%%%%%%%%%%%%%

Tfinal = 150;
clear all_X all_Y all_r all_phi
for vehicle_i=1:1
    vehicle_i
    m = vehicle(vehicle_i).m;
    Iz = vehicle(vehicle_i).Iz;
    a = vehicle(vehicle_i).a;
    b = vehicle(vehicle_i).b;
    L = a+b;
    Caf = vehicle(vehicle_i).Caf;
    Car = vehicle(vehicle_i).Car;
    
    % FILL THIS IN WITH CORRECT TERMS!
    A = [(Caf+Car)/(m*U) (a*Caf-b*Car)/(m*U)-U;
        (a*Caf-b*Car)/(Iz*U) (a^2*Caf+b^2*Car)/(Iz*U)];
    B = [-Caf/m -Car/m; -a*Caf/Iz b*Car/Iz];
    
    % FILL THIS IN WITH CORRECT TERMS   
    num_yawrate = [-a*Caf/Iz Caf*Car*L/(m*U*Iz)];
    num_latvel = [-Caf/m b*Caf*Car*L/(m*U*Iz)+a*Caf*U/Iz];
    den = [1 -((Caf+Car)/(m*U)+(Caf*a^2+Car*b^2)/(Iz*U)) (Caf*Car*L^2/(m*Iz*U^2)+(a*Caf-b*Car)/Iz)];
    
    %disp('Run sim now');
    %pause;
    
    sim('MODEL_BicycleModel_Integrator_SS_and_TF.slx', Tfinal);
    %figure(1)
    %[AX,H1,H2] = plotyy(t,r,t,V);      
    %xlabel('time (sec)');
    %set(get(AX(1),'Ylabel'),'String','yaw rate (rad/sec)')
    %set(get(AX(2),'Ylabel'),'String','lateral velocity (m/s)')
    
    h1 = figure(99);
    set(h1,'Name','Yawrate')
    plot(t,r,'ro',t,r_int,'b',t,r_tf,'gx'); legend('SS','int','tf');
    xlabel('Time (sec)'); ylabel('Yawrate (rad/sec)');
    
    h2 = figure(88);
    set(h2,'Name','LatVel')
    plot(t,V,'ro',t,V_int,'b',t,V_tf,'gx'); legend('SS','int','tf');
    xlabel('Time (sec)'); ylabel('Lateral Velocity (m/sec)');
        
    h3 = figure(77);
    set(h2,'Name','XYposition')
    plot(X_int,Y_int,'b-');
    xlabel('X position from all-integrator [m]'); ylabel('Y position from all-integrator model [m]');
    
    
    
    all_X(:,vehicle_i) = X;
    all_Y(:,vehicle_i) = Y;
    all_phi(:,vehicle_i) = phi;
    all_r(:,vehicle_i) = r;
end
%set(gca,'XTick',[0:1:5]);
%set(gca,'YTick',[0:0.2:2]);



if make_movies == 1
    % To create vehicle skid plot
    figure;
    multiplier = 10;
    
    clear M;
    set(gcf,'DoubleBuffer','on');
    mov = avifile('3vehicles_lanechange.avi','Quality',100)      
    for j=1:round(length(phi)/100):length(phi)
        for vehicle_i=1:3
            alpha_r(vehicle_i) = all_phi(j,vehicle_i)-multiplier*(V(j)-vehicle(vehicle_i).b*all_r(j,vehicle_i))/U;
            bodyx_start(vehicle_i) = all_X(j,vehicle_i)+multiplier*vehicle(vehicle_i).a*cos(alpha_r(vehicle_i));
            bodyy_start(vehicle_i) = all_Y(j,vehicle_i)+multiplier*vehicle(vehicle_i).a*sin(alpha_r(vehicle_i));
            bodyx_end(vehicle_i) = all_X(j,vehicle_i)-multiplier*vehicle(vehicle_i).b*cos(alpha_r(vehicle_i));
            bodyy_end(vehicle_i) = all_Y(j,vehicle_i)-multiplier*vehicle(vehicle_i).b*sin(alpha_r(vehicle_i));
        end
        plot(all_X(1:j,1),all_Y(1:j,1),'b',...
            all_X(1:j,2),all_Y(1:j,2),'b',...
            all_X(1:j,3),all_Y(1:j,3),'b',...
            [bodyx_start(1); bodyx_end(1)],[bodyy_start(1),bodyy_end(1)],'r',...
            [bodyx_start(2); bodyx_end(2)],[bodyy_start(2),bodyy_end(2)],'r',... 
            [bodyx_start(3); bodyx_end(3)],[bodyy_start(3),bodyy_end(3)],'r'); 
        axis equal
        xlabel('Global X (m)');
        ylabel('Global Y (m)');  
        text(all_X(j,1),all_Y(j,1),'Vehicle 1');
        text(all_X(j,2),all_Y(j,2),'Vehicle 2');
        text(all_X(j,3),all_Y(j,3),'Vehicle 3');
        axis([0 200 -50 150]);
        grid on;
        pause(0.01);
        M(j) = getframe;
        F = getframe(gca);
        mov = addframe(mov,F);
    end   
    mov = close(mov);
    disp('Paused...');
    pause;
    
end