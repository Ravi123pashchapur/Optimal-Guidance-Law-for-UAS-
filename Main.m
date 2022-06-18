%.. Matlab Initialise 
    clear all; clc; warning off; 

%% Simulation of homing Guidance law at N = 3.
%.. Simulation Initialise
    Sim_Parameters;
    missile_states = [MX10 MY10 VMX10 VMY10 0 0 GAM_M10];
    target_states  = [TX0 TY0];
    time = 0;
    ZEM_1 = 0;
    Acc_1 = 0;
for t = 0:DT:10
    %.. Seeker
        [R1,SIG1,GAM1,SIGR1] = Seeker(target_states,missile_states(end,:));
    %.. Guidance Law
        [AM zem_1] = Guidance_1(R1, SIG1, GAM1, SIGR1);
        Acc_1 = [Acc_1;AM];
        ZEM_1 = [ZEM_1;zem_1];
    %.. Missile Dynamics
        missile_states_update = Dynamics(missile_states(end,:),AM);
        missile_states = [missile_states;missile_states_update];
        time = [time;t];
    %.. Simulation End
        if R1 <=0.01
            break;
        end
end
%.. Plot
% design your own codes to plot the results

X1 = missile_states(:,1);
Y1 = missile_states(:,2);
time_1 = time;


%% Simulation of homing Guidance law at N = 4.
%.. Simulation Initialise
    Sim_Parameters;
    missile_states = [MX10 MY10 VMX10 VMY10 0 0 GAM_M10];
    target_states  = [TX0 TY0];
    time = 0;
    ZEM_2 = 0;
    Acc_2 = 0;
for t = 0:DT:10
    %.. Seeker
        [R1,SIG1,GAM1,SIGR1] = Seeker(target_states,missile_states(end,:));
    %.. Guidance Law
        [AM zem_2] = Guidance_2(R1, SIG1, GAM1, SIGR1);
        Acc_2 = [Acc_2;AM];
        ZEM_2 = [ZEM_2;zem_2];
    %.. Missile Dynamics
        missile_states_update = Dynamics(missile_states(end,:),AM);
        missile_states = [missile_states;missile_states_update];
        time = [time;t];
    %.. Simulation End
        if R1 <=0.01
            break;
        end
end
%.. Plot
% design your own codes to plot the results

X2 = missile_states(:,1);
Y2 = missile_states(:,2);
time_2 = time;


%% Simulation of homing Guidance law at N = 5.
%.. Simulation Initialise
    Sim_Parameters;
    missile_states = [MX10 MY10 VMX10 VMY10 0 0 GAM_M10];
    target_states  = [TX0 TY0];
    time = 0;
    ZEM_3 = 0;
    Acc_3 = 0;
for t = 0:DT:10
    %.. Seeker
        [R1,SIG1,GAM1,SIGR1] = Seeker(target_states,missile_states(end,:));
    %.. Guidance Law
        [AM zem_3]  = Guidance_3(R1, SIG1, GAM1, SIGR1);
        Acc_3 = [Acc_3;AM];
        ZEM_3 = [ZEM_3;zem_3];
    %.. Missile Dynamics
        missile_states_update = Dynamics(missile_states(end,:),AM);
        missile_states = [missile_states;missile_states_update];
        time = [time;t];
       
    %.. Simulation End
        if R1 <=0.01
            break;
        end
end
%.. Plot
% design your own codes to plot the results

X3 = missile_states(:,1);
Y3 = missile_states(:,2);
time_3 = time;




%% Plotting the results 

% Simulator
for k = 1:length(X3)
    if k<=length(X1)
        plot(X1(k),Y1(k),'rx',LineWidth = 2,MarkerSize = 7)
        xlabel('X [m]');
        ylabel('Y [m]');
        title('Intercept Trajectories Simulator')
       
    end
    hold on
    if k<=length(X1)
        text(X1(k),Y1(k)+0.2,'Missile')

    end
    if k<=length(X1)
        plot(X1(1:k),Y1(1:k),'m-.',LineWidth = 2)
    end
    axis([0 22 -1 2])
    plot(20,0,'o',LineWidth = 5)
    text(19,-0.2,'Target')
    pause(0.000001)

    
    if k ~= length(X1)
        clf
    end
        
end    


% Plot the Missile trajectory at N = 3
figure
plot(X1,Y1,'m-.',LineWidth = 2)
xlabel('X [m]');
ylabel('Y [m]');
title('Intercept Trajectories')
axis([0 22 -1 2])
grid on
grid minor
hold on
plot(20,0,'ro',LineWidth = 5)
text(19,-0.09,'Target')
legend('N = 3 (Optimal value)')

% Optimal Accleration
figure
plot(time_1(2:2026),Acc_1(2:2026),'m-.',LineWidth = 2)
xlabel('Time [sec]');
ylabel('Acceleration [m/s^2]');
title('Acceleration Command')
grid on
grid minor
legend('N = 3 (Optimal value)')

% Plot the Missile trajectory at N = 3, N = 4,N = 5
figure
plot(X1,Y1,'m-.',LineWidth = 2)
xlabel('X [m]');
ylabel('Y [m]');
title('Intercept Trajectories')
axis([0 22 -1 2])
grid on
grid minor
hold on
plot(X2,Y2,'g-.',LineWidth = 2)
plot(X3,Y3,'b-.',LineWidth = 2)
plot(20,0,'ro',LineWidth = 5)
text(19,-0.09,'Target')
legend('N = 3','N = 4','N = 5')


% Plot Acceleration command at N=3, N=4, N=5
figure
plot(time_1(2:2026),Acc_1(2:2026),'m-.',LineWidth = 2)
xlabel('Time [sec]');
ylabel('Acceleration [m/s^2]');
title('Acceleration Command')
grid on
grid minor
hold on
plot(time_2(2:2019),Acc_2(2:2019),'g-.',LineWidth = 2)
plot(time_3(2:2015),Acc_3(2:2015),'b-.',LineWidth = 2)
legend('N = 3','N = 4','N = 5')

% Plot Zero miss error 
figure
plot(time_1(2:2026),ZEM_1(2:2026),'m-.',LineWidth = 2)
xlabel('Time [sec]');
ylabel('ZEM [m]');
title('Zero Miss Error')
grid on
grid minor
hold on
plot(time_2(2:2019),ZEM_2(2:2019),'g-.',LineWidth = 2)
plot(time_3(2:2015),ZEM_3(2:2015),'b-.',LineWidth = 2)
legend('N = 3','N = 4','N = 5')

save A3.mat
