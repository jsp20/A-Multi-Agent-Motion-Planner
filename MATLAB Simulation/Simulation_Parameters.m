 %% --------------------------Variable initialization-----------------------
% Setup of global variables: 


%% User specified 
% Duration of simulation
iter = 20000;
% iter = 1000;
dt = 0.01;%seconds
T = iter*dt; %seconds 



% Goal location for agents
% q_goal = [xi, yi]';

% home location of agents
% qi = [xi, yi, theta_i]';


% 7 Agent Case Good
q_goal= [ -10 -10;...
          -6 5;...
          10 -15;...
          9 10;...
%           20 -17;...
%           20 25;...
%           20 -22
                    ]';
     
     

q_0 = [  10 10 0.1; ...
           6.2 -6.2 0.1; ...
          -10 15 0.1; ...
          -9 -5 0.1
%          -20 17 0.1;...
%          -20 25 0.1;...
%          -20 22 0.1
                     ]';



%% Agent information: 
Rho = 0.4; %m
del_d = 1.42; %m
E_f = 1.52;   %m


% Number of agents in the workspace 
[n m] = size(q_0);


% Noise related to the pose measurment 
Pv_i      = eye(n,n).*0.01;


%d parameters
dm = 2*Rho; %m 
d_theta = sqrt(norm(Pv_i)+1);
d_d     = sqrt(2*(norm(Pv_i)+1));
d_prime_m = dm +2*d_d;
%   safety parameter for worst case neighbour
d_J=((2*d_d+dm)*sin(d_theta)+2*d_d)/cos(d_theta);
Rc =2*d_prime_m;  
dc = Rc; %m  
dr = (dc-dm)*rand(1)+dm; %m




% Effect of wind
w_min = 5.6; %m/sec 
w_max = 7.25; %m/sec
w_delta = w_max-w_min;

% making EW and E equal for every agent. 
Ew = rand();
Ei = rand(1);
miu = 10;

% Range of distance which is: dm<de<dr
 d_e = dr-Ei;
 