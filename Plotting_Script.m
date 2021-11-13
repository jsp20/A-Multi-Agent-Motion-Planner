%% ------------------------ Description ------------------------
% This code was designed to plot the simulation parameters:
% qi_time     - Used to store the actual pose for m agents in simulation
% CompTime    - Computation time for calculation of m agents input commands
% qj_rel_i    - COmputation of the relative distance between agent 1 and all other agents in the simulation
% Field-Angle - Angular rate of reference field for m agents 

%% ------------------------ Section 1 ------------------------
% Pose Data parsing 
q1=reshape(qi_time(:,1,:),[3,round(T/dt)+1]);
q2=reshape(qi_time(:,2,:),[3,round(T/dt)+1]);
q3=reshape(qi_time(:,3,:),[3,round(T/dt)+1]);
q4=reshape(qi_time(:,4,:),[3,round(T/dt)+1]);
% q5=reshape(qi_time(:,5,:),[3,round(T/dt)+1]);
% q6=reshape(qi_time(:,6,:),[3,round(T/dt)+1]);
% q7=reshape(qi_time(:,7,:),[3,round(T/dt)+1]);
% q8=reshape(qi_time(:,8,:),[3,round(T/dt)+1]);
% q9=reshape(qi_time(:,9,:),[3,round(T/dt)+1]);
% q10=reshape(qi_time(:,10,:),[3,round(T/dt)+1]);

% Configure time 
time = max(find(q1(1,:)));
t = linspace(0,time*dt,time);
SampleTime = ones(1,time)*dt;

for i = 1:time
q1_act(:,i) = q1(:,i);
q2_act(:,i) = q2(:,i);
q3_act(:,i) = q3(:,i);
q4_act(:,i) = q4(:,i);
% q5_act(:,i) = q5(:,i);
% q6_act(:,i) = q6(:,i);
% q7_act(:,i) = q7(:,i);
end

path1 = animatedline('Color', [0, 0.4470, 0.7410],'LineWidth',4);
path2 = animatedline('Color', 'g','LineWidth',4);
path3 = animatedline('Color', [0.8500, 0.3250, 0.0980]	,'LineWidth',4);
path4 = animatedline('Color', 'k','LineWidth',4);
% path5 = animatedline('Color', [0.4660, 0.6740, 0.1880],'LineWidth',4);
% path6 = animatedline('Color', [0.4940, 0.1840, 0.5560],'LineWidth',4);
% path7 = animatedline('Color', [0.6350, 0.0780, 0.1840],'LineWidth',4);

figure(1)
% To generate gif uncomment this and line 64 
% filename='4_Agent_Sim.gif';
title(['Paths Combined for ',num2str(m),' Agents']);
hold on 
axis([-30 30 -30 30])
for i = 1:m 
scatter (q_0(1,i),q_0(2,i),'r','x')
scatter (q_goal(1,i),q_goal(2,i),'b','o')
end 

for i = 1:100:time
    addpoints(path1,q1_act(1,i),q1_act(2,i));
    addpoints(path2,q2_act(1,i),q2_act(2,i));
    addpoints(path3,q3_act(1,i),q3_act(2,i));
    addpoints(path4,q4_act(1,i),q4_act(2,i));
%     addpoints(path5,q5_act(1,i),q5_act(2,i));
%     addpoints(path6,q6_act(1,i),q6_act(2,i));
%     addpoints(path7,q7_act(1,i),q7_act(2,i));  
    drawnow
%     Animation_GIF( filename,i,gcf )
end

legend ('Path-Agent1', 'Path-Agent2','Path-Agent3','Path-Agent4','Start','Finish')
hold off
grid on
grid minor 


%% ------------------------ Section 2 ------------------------ 
% Computation Time data parsing 


figure(2)

axis([0 time*dt 0 0.03])

hold on
title(['Computation Time for ',num2str(m),' agents'])
plot(t,CompTime )
str = ['Comp Time ',num2str(m),' Agents'];
legend (str)
xlabel('Simulation Time(s)')
ylabel('Computation Time(s)')
grid on
grid minor
hold off 

%% ------------------------ Section 3 ------------------------
% Relative distance calculation for agent 1 w.r.t all other agents in sim

n = length(q1);
% Relative separation between agent 1 and all other agents
q2_rel_1 = vecnorm([q2_act(1,:)-q1_act(1,:);q2_act(2,:)-q1_act(2,:)],2,1);
q3_rel_1 = vecnorm([q3_act(1,:)-q1_act(1,:);q3_act(2,:)-q1_act(2,:)],2,1);
q4_rel_1 = vecnorm([q4_act(1,:)-q1_act(1,:);q4_act(2,:)-q1_act(2,:)],2,1);
% q5_rel_1 = vecnorm([q5_act(1,:)-q1_act(1,:);q5_act(2,:)-q1_act(2,:)],2,1);
% q6_rel_1 = vecnorm([q6_act(1,:)-q1_act(1,:);q6_act(2,:)-q1_act(2,:)],2,1);
% q7_rel_1 = vecnorm([q7_act(1,:)-q1_act(1,:);q7_act(2,:)-q1_act(2,:)],2,1);



for i = 1:time
    q2_1_rel(1,i) = q2_rel_1(1,i);
    q3_1_rel(1,i) = q3_rel_1(1,i);
    q4_1_rel(1,i) = q4_rel_1(1,i);
%     q5_1_rel(1,i) = q5_rel_1(1,i);
%     q6_1_rel(1,i) = q6_rel_1(1,i);
%     q7_1_rel(1,i) = q7_rel_1(1,i);
end 

d_min = ones(1,time)*d_prime_m;



figure(3)
hold on
title(['Relative distance between agent 1 and all ',num2str(m),' agents'])
plot(t, q2_1_rel)
plot(t, q3_1_rel)
plot(t, q4_1_rel)
% plot(t, q5_1_rel)
% plot(t, q6_1_rel)
% plot(t, q7_1_rel)
plot(t,d_min,'--')
hold off
grid on 
grid minor 
% legend ('d2-1', 'd3-1','d4-1','d5-1','d6-1','d7-1','d_{min}allowed')
legend ('d2-1', 'd3-1','d4-1','d_{min}allowed')
xlabel('Time(s)')
ylabel('Pairwise distance (m)')


%% ------------------------ Section 4 ------------------------
% Ref Field Angle rate for m agents

Field_Angle_A1 = Field_Angle(1,:);
Field_Angle_A2 = Field_Angle(2,:);
Field_Angle_A3 = Field_Angle(3,:);
Field_Angle_A4 = Field_Angle(4,:);
% Field_Angle_A5 = Field_Angle(5,:);
% Field_Angle_A6 = Field_Angle(6,:);
% Field_Angle_A7 = Field_Angle(7,:);


% Angular rate limits: 
Ang_Rate_Max = ones(1,length(Field_Angle_A1))*pi;
Ang_Rate_Min = ones(1,length(Field_Angle_A1))*-pi;

figure(4)
axis([0 time*dt -2*pi 2*pi])

hold on
title(['Reference Field Angular Rate for  ',num2str(m),' Agents'])
plot(t,Field_Angle_A1)
plot(t,Field_Angle_A2)
plot(t,Field_Angle_A3)
plot(t,Field_Angle_A4)
% plot(t,Field_Angle_A5)
% plot(t,Field_Angle_A6)
% plot(t,Field_Angle_A7)
plot(t,Ang_Rate_Max,'k')
plot(t,Ang_Rate_Min,'k')
hold off
% legend ('Field-Rate Agent1', 'Field-Rate Agent2','Field-Rate Agent3','Field-Rate Agent4','Field-Rate Agent5','Field-Rate Agent6','Field-Rate Agent7')
legend ('Field-Rate Agent1', 'Field-Rate Agent2','Field-Rate Agent3','Field-Rate Agent4')

xlabel('Simulation Time(s)')
ylabel('Angular Rate(rad/s)')
grid on 
grid minor