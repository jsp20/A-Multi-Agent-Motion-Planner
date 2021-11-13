close all 
clc 
clear
%% ===========Load data to be plotted===========
% Import complete ROS log for 
ROS_Log = importdata('ROS_Pose_Log.txt');
% Run sim parameters for 4 vehicle sim
run('Simulation_Parameters.m');

% Open Computation time 
ROS_CompTime = importdata('ROS_CompTime.txt');


%% ===========Data formating===========

% Length of the full ROS log
n = length(ROS_Log);

% Initialize pose based on number of agents
q_1 = zeros(3,n/(3*m))';
q_2 = zeros(3,n/(3*m))';
q_3 = zeros(3,n/(3*m))';
q_4 = zeros(3,n/(3*m))';

% Parse out data from ROS Log
Agent_Num = 1;
row_num = 1;
col_num = 1;
index = 0;
for i = 1:n
    index = index +1;
    
    if index <= 3
        q_1(row_num,col_num)= ROS_Log(i);
        col_num = col_num +1;
        if col_num ==4 && index == 3
            col_num = 1;
        end
    end
    
    if index <= 6 && index >= 4
        q_2(row_num,col_num)= ROS_Log(i);
        col_num = col_num +1;
        if col_num ==4 && index == 6
            col_num = 1;
        end
    end
    
    if index <= 9 && index >=7
        q_3(row_num,col_num)= ROS_Log(i);
        col_num = col_num +1;
        if col_num ==4 && index == 9
            col_num = 1;
        end
    end
    
    if index <= 12 && index >= 10
        q_4(row_num,col_num)= ROS_Log(i);
        col_num = col_num +1;
        if col_num ==4 && index == 12
            col_num = 1;
        end
    end
    
    if index == 12
        row_num = row_num+1;
        index = 0;
    end 

end

t = linspace(0,n/(3)*0.01,(n/(3*m)))';

% Relative separation between agent 1 and all other agents
q2_rel_1 = vecnorm([q_2(:,1)-q_1(:,1),q_2(:,2)-q_1(:,2)], 2,2);
q3_rel_1 = vecnorm([q_3(:,1)-q_1(:,1),q_3(:,2)-q_1(:,2)], 2,2);
q4_rel_1 = vecnorm([q_4(:,1)-q_1(:,1),q_4(:,2)-q_1(:,2)], 2,2);

d_min = ones(1,n/(3*m))*d_prime_m;

% Setting up desired step size for plotting 
t_CompTime = length(ROS_CompTime); 
dt_plot = ones(1,t_CompTime)*dt;


%% ===========Data plotting===========

figure(1)
title('Paths Combined');
hold on
for i = 1:m 
scatter (q_0(1,i),q_0(2,i),'r','x')
scatter (q_goal(1,i),q_goal(2,i),'b','o')
end
legend( 'Start','Finish')
scatter(q_1(:,1),q_1(:,2),'DisplayName','Path-Agent1')
scatter(q_2(:,1),q_2(:,2),'DisplayName','Path-Agent2')
scatter(q_3(:,1),q_3(:,2),'DisplayName','Path-Agent3')
scatter(q_4(:,1),q_4(:,2),'DisplayName','Path-Agent4')
legend()

grid on 
grid minor 
hold off


figure(2)
hold on 
plot(t,q2_rel_1)
plot(t,q3_rel_1)
plot(t,q4_rel_1)
plot(t,d_min,'--')
hold off
grid on 
grid minor 
legend ('d2-1', 'd3-1','d4-1','d_{min}allowed')
xlabel('Time(s)')
ylabel('Pairwise distance (m)')

figure(3)
axis([0 n/(3)*0.01 0 0.02])
hold on 
plot(t,ROS_CompTime)
plot(t,dt_plot)
hold off 
grid on 
grid minor 
legend ('Ros - CompTime', 'Step Size')
xlabel('Time(s)')
ylabel('Comp Time (s)')
title('4 Agent Simulation Computation Time')
