    
%% ------------------------Simulation Description--------------------------
% The following code is an implementation of the multiagent control protocol 
% presented in the paper:
% Robust Semi-Cooperative Multi-Agent Coordination In the Presence of Stochastic Disturbances 
% Authors: Kunal Garg  Dongkun Han Dimitra Panagou

%% --------------------------Simulation Variables-------------------------------
clc 
clear
run('Simulation_Parameters.m')
% Values related to the actual pose of the robot

% Pose of robot i
qi       = q_0;
qi_next  = zeros(3,m); 
theta_i   = qi(3,:);
r_i       = zeros(2,m);

% Pose rate of robot i
qi_dot   = zeros(3,m);
qi_dot_next = zeros(3,m);

% Linear velocity of robot i
u_i       = ones(1,m)*0.01;

% Angular velocity of robot i 
omega_i   = ones(1,m)*0.0; 

% Pose measurement of robot i 
z_i       = zeros(3,1);

% Pose measurement noise for agent i
v_i       = randn(3,m);

% variable used to store the pose of each agent as the simulation is running 
qi_time  = zeros(3,m,round(T/dt)+1);
count     = 1;
arrived   = zeros(1,m);

% List of agent ID which is within sensing radius:
L         = zeros(1,m);


%--------------Variables used and calculated by the EKF observer--------------
% Values related to the predicted pose of the robot

% Predicted pose of robot i
qi_pred          = zeros(3,1);
qi_pred_next     = zeros(3,1);
theta_i_pred      = zeros(1,m);

% Predicted pose rate of robot i
qi_dot_pred      = zeros(3,m);
qi_dot_pred_next = zeros(3,m);
% predicted pose measurements for agent i 
z_i_pred          = zeros(3,m);
% pose error between predicted and actual
qi_error         = zeros(3,m);
% Measurement matrix (Pose of agent is fully observable)
H_i          = eye(3,3);
gamma        = [1 0;0 1; 0 0];
% Rate of change for the error covariance Pi 
    for i=1:m
        P_i_dot(:,:,1) = eye(3,3)*0.001;
    end
% Position noise (Pw_i)
Pw_i      = eye(2,2)*0.01;

A_i       = zeros(3,3);
% Kalman Gain
    for i =1:m
        K_i(:,:,i) = eye(3,3)*0.01;
    end 
% Covariance matrix for pose error
    for i=1:m
        P_i(:,:,1) = eye(3,3)*0.1;
    end

%--------------Variables used and calculated using velocity control law--------------     
% orientation change caused by reference field 
psi_dot_p        = zeros(1,m);
psi_dot_p_next   = zeros(1,m);
psi_p            = zeros(1,m);    
% relative distance between agent i and agent k within the sensing radius
dik_pred     = zeros(1,m);
% predicted x-y of agent k within sensing radius. 
rk_pred     = zeros(2,m);
% predicted x-y of agent i and 
ri_pred     = zeros(2,m);    
% Linear gain for ui, has to be greater than 0
K_ui         = 0.7;
% Linear gain for wi, has to be greater than 0
K_wi         = 0.5;

ui_k         = zeros(1,m);
ui_c         = zeros(1,m);
ui_n         = zeros(1,m);
ni_pred      = ones(2,m)*0.01;
nk_pred      = ones(2,m)*0.01;
Jk_pred      = zeros(1,m);
d_ki_pred    = zeros(1,m);
theta_k_pred = zeros(1,m);
r_ki_pred    = zeros(2,m);


CompTime =[];
%--------------Pose perturbation at t=0 --------------     
% Initial position noise due to wind:
% Average value of wind in x direction
w_x_bar = (w_max-w_min)*rand(1,m)+w_min;
% Average value of wind in y direction
w_y_bar = (w_max-w_min)*rand(1,m)+w_min;

% vector holding the position disturbance from the wind
wi_bar = [w_x_bar ; w_y_bar]*dt;

% Change of vehicle state from a small perturbance:
    for i = 1:m 
        qi_dot(:,i) = qi_dot(:,i) + [u_i(1,i)*cos(theta_i(1,i)); u_i(1,i)*sin(theta_i(1,i)); omega_i(1,i)]+[wi_bar(:,i); 0 ];
        qi(:,i)          = qi(:,i)+qi_dot(:,i).*dt;
        r_i(1,i)          = qi(1,i);
        r_i(2,i)          = qi(2,i);
        z_i(:,i)          = qi(:,i)+v_i(:,i);
        theta_i(1,i)      = wrapToPi(qi(3,i));
    end
       
    
%  Change of predicted pose from a small perturbance:
     for i = 1:m
        qi_dot_pred(:,i) = qi_dot_pred(:,i)+ randn(3,1)*0.01;
        qi_pred(:,i)     = qi(:,i)+qi_dot_pred(:,i).*dt;
        ri_pred(1,i)     = qi_pred(1,i);
        ri_pred(2,i)     = qi_pred(2,i);
        z_i_pred(:,i)     = qi_pred(:,i);
        theta_i_pred(1,i) = wrapToPi(qi_pred(3,i));
     end
     
     
%% Main loop for time
for t = dt:dt:T
t
tic
% A = tic;

%% Position noise caused by wind, this noise is a function of time and therefore changes during each iteration 

% Initial position noise due to wind:
% Average value of wind in x direction
w_x_bar = (w_max-w_min)*rand(1,m)+w_min;
% Average value of wind in y direction
w_y_bar = (w_max-w_min)*rand(1,m)+w_min;

% vector holding the position disturbance from the wind
wi_bar = [w_x_bar ; w_y_bar]*dt;


% Cycle through each robot to incrementally change the position. 
    for i =1:m
        
% Condition to check if current agent has indeed arrived to the goal
% location. This must be done because the motion planning algorithm is only
% valid if qi != q_goal 

if abs(q_goal(1,i)-qi(1,i))<=0.25 && abs(q_goal(2,i)-qi(2,i))<=1.5
    fprintf('Agent %i has arrived! \n' ,i);
    arrived(1,i) = 1;
    qi_time(:,i,count) = qi(:,i);
    
end

% Reset variables at the start of each loop
Neigh_j     = zeros(1,m);
Jk_pred     = zeros(1,m);
rk          = zeros(2,m);
ri_pred     = zeros(2,m);
d_ki_pred   = zeros(1,m);
ui_k        = zeros(1); 
L           = zeros(1);

% Number of agents inside the sensing radius and are also within the critical distance de 
k            = 1;

%-------------- Checking agent i for worst case neighbour--------------              
        
for j = 1:m
          % finding the distance between agent i and j and checking if
          % agent j is within the robot's sensing radius.      
          if norm([qi(1,j)-qi(1,i); qi(1,j)-qi(1,i)]) < Rc && j~= i             
%             This is the list of agents who are within the sensing radius
%             of Rc
              rk(1,j) = qi(1,j);
              rk(2,j) = qi(2,j);              
              Neigh_j(1,j) = j; 

              
          end         
end
% --------------EKF based observer model for estimating the pose rate of the agent--------------
% 
% Eq 8a)
qi_dot_pred(:,i) = [u_i(1,i)*cos(theta_i_pred(1,i)); u_i(1,i)*sin(theta_i_pred(1,i)); omega_i(1,i)]+gamma*wi_bar(:,i)+K_i(:,:,i)*(z_i(:,i)-z_i_pred(:,i));

% Predicted pose and pose rate update
qi_pred_next(1,i)     = qi_pred(1,i)+qi_dot_pred(1,i)*dt;
qi_pred_next(2,i)     = qi_pred(2,i)+qi_dot_pred(2,i)*dt;
qi_pred_next(3,i)     = wrapToPi(qi_pred(3,i)+qi_dot_pred(3,i)*dt);
ri_pred(1,i)          = qi_pred_next(1,i);
ri_pred(2,i)          = qi_pred_next(2,i);
theta_i_pred(1,i)     = qi_pred_next(3,i);
qi_pred(:,i)          = qi_pred_next(:,i);


%  eq 8c)
qi_error = qi(:,i)-qi_pred(:,i);
Error_log(:,i,count) = qi_error;
P_i(:,:,i) = cov(qi_error*qi_error');

%  Eq 8b)
A_i = [ 0 0 -u_i(1,i)*sin(theta_i_pred(1,i));...
        0 0 u_i(1,i)*cos(theta_i_pred(1,i));...
        0 0 0;];


P_i_dot(:,:,i)= A_i(:,:).*P_i(:,:,i)+P_i(:,:,i).*A_i(:,:)'-K_i(:,:,i).*H_i.*P_i(:,:,i)+gamma*Pw_i*gamma';      

% eq 8c)
z_i_pred(:,i) = qi_pred(:,i)+v_i(:,i);

% Updating the Pi covariance matrix based on Pi_dot
P_i(:,:,i) = P_i(:,:,i)+P_i_dot(:,:,i).*dt;

%  eq 8d)
K_i(:,:,i) = -P_i(:,:,i)*H_i'/Pv_i;

%--------------Linear and Angular velocity calculation using control law equations--------------

% This is calculated before the reference field because it is the linear
% velocity used for calculating the normalized and perturbed force. 

% Linear veloicty needs to be divided into two sections, the first portion

% Eq11
ui_n(1,i) = K_ui*tanh(norm([ri_pred(1,i)-q_goal(1,i); ri_pred(2,i)-q_goal(2,i)]));     


%           Calculate the Attractive field for agent i

[Fi_x, Fi_y, ui_c(1,i), psi_p(1,i), dFi_pnx_dx ,dFi_pnx_dy,dFi_pny_dx,dFi_pny_dy,Fi_pnx,Fi_pny] = Reference_Field(r_i(:,i),qi_pred(:,i),rk,ui_n(1,i),wi_bar(:,i),i,m,Neigh_j);
F_i           = [Fi_x Fi_y];




% using the list of critical neighbours lets check which ones match the first condition for u_i given in equation 9  

% check if any agent within the sensing radius matches condition 1 for the
% piecewise function, if not then the second condition needs to be checked.
% calcuation of the linear velocity for agent i 

    for j =1:m
        % we check that we are not our own neighbour 
        if Neigh_j(1,j) ~= 0 
            r_ki_pred(:,Neigh_j(1,j))= [qi_pred(1,i)-qi_pred(1,j); qi_pred(2,i)-qi_pred(2,j)];
            d_ki_pred(1,j) = norm(r_ki_pred);
                if d_prime_m <= d_ki_pred(1,j) && d_ki_pred(1,j)<= d_e
%                     this is a list of all the agents that are within the
%                     safety zone of agent i. 
                    L(1,k) = j; 
                    k = k+1;
                end
        end           
    end
        
    
% Checking if any of the agents are within condition 1 for velocity 
Check = any(L);

    if Check == 1
        for j = 1: length(L) 
            if L(1,j) ~= 0
            ni_pred = [ cos(theta_i_pred(1,i)) ; sin(theta_i_pred(1,i))];
            nk_pred = [ cos(theta_i_pred(1,L(1,j))); sin(theta_i_pred(1,L(1,j)))];
            rki=  r_ki_pred(:,L(1,j));
            uis_k =  (Ei*u_i(1,j)+(d_prime_m+Ew)*w_delta)*(r_ki_pred(:,L(1,j))'*nk_pred)/(r_ki_pred(:,L(1,j))'*ni_pred);       
            ui_k(1,j) = ui_c(1,i)*((d_ki_pred(1,L(1,j))-d_prime_m)/(d_e-d_prime_m))+uis_k*((d_e-d_ki_pred(1,L(1,j)))/(d_e-d_prime_m)); %m/                                      
            end
        end
        
% This is where we estimate the smallest velocity of agent i if there are
% any agents which meet condition 1 of the linear velocity control law
       u_i(1,i) = -1/miu*log(sum(exp(-miu*ui_k(:)))); 
    else   
        
% If there are no agents which meet condition 1 then we default to setting
% the linear velocity equal to the nominal velocity u_ic.
    u_i(1,i) = ui_c(1,i);        
    end

% Saturation for linear velocity command: 

if u_i(1,i)> 0.7 
    u_i(1,i) = 0.7;
    elseif u_i(1,i)>=0 && u_i(1,i) < 0.1
        u_i(1,i) =0.1;
    elseif u_i(1,i) >-0.1 && u_i(1,i) <0
        u_i(1,i) = -0.1
    elseif u_i(1,i)<-0.7
        u_i(1,i) = -0.7;
end 
    
    
    
    
    
%compute si_dot
psi_dot_p_next(1,i) = ((dFi_pny_dx*cos(theta_i_pred(1,i))+dFi_pny_dy*sin(theta_i_pred(1,i)))*Fi_pnx-(dFi_pnx_dx*cos(theta_i_pred(1,i))+(dFi_pnx_dy*sin(theta_i_pred(1,i))))*Fi_pny)* u_i(1,i);
psi_dot_p(1,i) = psi_dot_p_next(1,i);
Field_Angle(i,count) = psi_dot_p(1,i);
% Angular velocity
% Eq13
% omega_i(1,i) = -K_wi*(wrapToPi(theta_i_pred(1,i)-psi_p(1,i)))+psi_dot_p(1,i)*0.1;
omega_i(1,i) = -K_wi*(wrapToPi(theta_i_pred(1,i)-psi_p(1,i)))+psi_dot_p(1,i)*0.1;
% Saturation for angular velocity command: 

if omega_i(1,i)> pi 
    omega_i(1,i) = pi;
    elseif omega_i(1,i)<-pi
        omega_i(1,i) = -pi;
end 
    
    end
    
for i =1:m    
%--------------Update pose of agent i based on motion model and output of velocity control law--------------

% Update of pose future rate based on control output
% qi_dot(t+1) = qi_dot(t) +[u_i*C(theta); u_i*S(theta); omega_i] + [w_bar 0 ]
qi_dot_next(:,i) = [u_i(1,i)*cos(theta_i(1,i)); u_i(1,i)*sin(theta_i(1,i)); omega_i(1,i)]+[wi_bar(:,i); 0 ];


% Integrate pose rate to determine future pose of vehicle 
% qi(t+1) = qi(t)+qi_dot(t+1)*dt 
qi_next(3,i) = wrapToPi(qi(3,i)+qi_dot_next(3,i).*dt);

qi_next(1,i) = qi(1,i)+qi_dot_next(1,i).*dt;
qi_next(2,i) = qi(2,i)+qi_dot_next(2,i).*dt;

% State measurement with aditive noise
z_i(:,i) = qi_next(:,i)+v_i(:,i);

% Update the current pose and pose rate based on the future calculations
qi_dot(:,i) = qi_dot_next(:,i);
qi(:,i) = qi_next(:,i);
r_i(1,i) = qi(1,i);
r_i(2,i) = qi(2,i);
theta_i(1,i) = qi(3,i);


% Variable used to store the computation time during each cycle. 
% CompTime(1,i,count) = toc;
 
qi_time(:,i,count) = qi(:,i); 
end
CompTime(1,count) = toc;
    
    count = count+1;

    if prod(arrived(1,:)~=0)
        fprintf('All agents have arrived!!');
        break   
    end

end






