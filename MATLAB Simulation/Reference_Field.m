% Reference Field V2 
% The following code is an implementation of the multiagent control protocol 
% presented in the paper:
% Robust Semi-Cooperative Multi-Agent Coordination In the Presence of Stochastic Disturbances 
% Authors: Kunal Garg  Dongkun Han Dimitra Panagou

% Inputs to function: ri,rk,ui_n,k,i,wi_bar
% Output of function: ui_c, psi_p, dFi_pnx_dx ,dFi_pnx_dy,dFi_pny_dx,dFi_pny _dy   

% While x and y are variable in time at each iteration they define the
% current pose of agent i.

function [Fi_qx, Fi_qy, ui_c, psi_p, dFi_pnx_dx ,dFi_pnx_dy,dFi_pny_dx,dFi_pny_dy,Fi_pnx,Fi_pny] = Reference_field (ri,qi_pred,rk,ui_n,wi_bar,i,m,Neigh_j)
%% Sigma function: 
run('Simulation_Parameters.m')

% Nomenclature:
% sigma_ij: The bump function to relate the amount of force which should be
% attractive or repulsive w.r.t agent i. 
% NOTE: sigma_ii doesn't exist since this is the influence of your
% repulsion w.r.t yourself. 
% dik: Distance between agent i and agent k 


sigma_ij = zeros(1,m);

% As per the definition in the paper k is the list of critical agents 
%currently in the sensing radius of agent i. 
for S= 1:m

    if Neigh_j(1,S) ~= 0 && S~=i
        dik = norm([rk(1,S)-ri(1,1) rk(2,S)-ri(2,1)]);
        a = -(2/(dr-dc)^3);
        b = 3*(dr+dc)/(dr-dc)^3;
        c = -((6*dr*dc)/(dr-dc)^3);
        d = (dc^2)*(3*dr-dc)/((dr-dc)^3);
        
% Defining the bump function based on eq 21 for reference 27:
        if dik >= dm && dik < dr
            sigma_ij(1,S) = 1;
            
        elseif dik >= dr && dik <= dc
            sigma_ij(1,S)= a*dik^3+b*dik^2+c*dik+d;
            
        elseif dik> dc
            sigma_ij(1,S) = 0;
            
        end
    end 
        
end

% This field value will be used to calculate ui_c since this depends on the
% actual position of the agent. 
[Fi_qx, Fi_qy, dFi_Ax_dx,dFi_Ax_dy, dFi_Ay_dx,dFi_Ay_dy,dFi_R_ijx_dx,dFi_R_ijx_dy,dFi_R_ijy_dx, dFi_R_ijy_dy] = Field(sigma_ij, ri, rk,i);  

Fi_q = [Fi_qx ; Fi_qy];



% This field value is using the predicted pose of agent i from the observer
% model to calculate the linear and angular velocities. 

[Fi_x_pred, Fi_y_pred, dFi_Ax_dx_pred,dFi_Ax_dy_pred, dFi_Ay_dx_pred,dFi_Ay_dy_pred,dFi_R_ijx_dx_pred,dFi_R_ijx_dy_pred,dFi_R_ijy_dx_pred, dFi_R_ijy_dy_pred]  = Field(sigma_ij,qi_pred,rk,i); 

Fi_q_pred = [Fi_x_pred ; Fi_y_pred];

%% Supplementary functions related to the reference field: 

% Nomenclature:
% ui_n: Modulated linear velocity w.r.t how far the agent is from its goal
% position. 
% wi_bar: Wind noise disturbance. 

% perturbed reference field  
Fi_p = (ui_n*(Fi_q)./(norm(Fi_q))-wi_bar);

% Linear velocity from normalized reference force defined by eq10.
ui_c = norm(Fi_p);


%%  Functions related to the predicted pose of agent i 


% recalculate the perturbed field but with Fi now being the calculated
% using (x,y) pred

Fi_p = (ui_n*(Fi_q_pred)./(norm(Fi_q_pred))-wi_bar);

% Normalize the perturbed reference field. 
Fi_pn = Fi_p./norm([Fi_p(1) Fi_p(2)]);
Fi_pnx = Fi_pn(1);
Fi_pny = Fi_pn(2);


% Use the perturbed reference field to calculate psi_p.
psi_p = atan2(Fi_pn(2),Fi_pn(1));


% The first set of partial derivatives are calculated based on having the
% predicted value for (x,y) from the observer model 
% Partial derivatives: 


% x direction 
dFi_x_dx = (prod(ones(1,m)-sigma_ij))*dFi_Ax_dx_pred+ sum(sigma_ij.*dFi_R_ijx_dx_pred,2);
dFi_x_dy = (prod(ones(1,m)-sigma_ij))*dFi_Ax_dy_pred+ sum(sigma_ij.*dFi_R_ijx_dy_pred,2);

% y direction
dFi_y_dx = (prod(ones(1,m)-sigma_ij))*dFi_Ay_dx_pred+ sum(sigma_ij.*dFi_R_ijy_dx_pred,2);
dFi_y_dy = (prod(ones(1,m)-sigma_ij))*dFi_Ay_dy_pred+ sum(sigma_ij.*dFi_R_ijy_dy_pred,2);


% intermediate variable used to simplify calculation of partial derivatives
% for normalized/perturbed reference field Fi
A = norm([Fi_q_pred(1),Fi_q_pred(2)])^2;
norm_Fi = sqrt(A);
B = norm([Fi_p(1),Fi_p(2)])^(-1);
% Partial derivative of norm of reference field Fi: 
% partial derivative in the x direction 
dFi_norm_dx_inv = -(1/2)*(A^(-3/2))*(2*Fi_x_pred*dFi_x_dx+2*Fi_y_pred*dFi_y_dx);

% Partial derivative in the y direction 
dFi_norm_dy_inv = -(1/2)*(A^(-3/2))*(2*Fi_x_pred*dFi_x_dy+2*Fi_y_pred*dFi_y_dy);

% Partial derivatives of the normalized/perturbed reference field Fi: 


% x-direction
dFi_pnx_dx = ui_n*(dFi_x_dx*1/norm([Fi_qx;Fi_qy])+Fi_qx*dFi_norm_dx_inv)*B+Fi_pnx*(-1/2)*B^3*(2*Fi_pnx*(dFi_x_dx*1/norm([Fi_qx;Fi_qy])+Fi_qx*dFi_norm_dx_inv)+2*Fi_pny*(dFi_y_dx*1/norm([Fi_qx;Fi_qy])+Fi_qy*dFi_norm_dx_inv)); 
dFi_pnx_dy = ui_n*(dFi_x_dy*1/norm([Fi_qx;Fi_qy])+Fi_qy*dFi_norm_dy_inv)*B+Fi_pny*(-1/2)*B^3*(2*Fi_pnx*(dFi_x_dy*1/norm([Fi_qx;Fi_qy])+Fi_qx*dFi_norm_dy_inv)+2*Fi_pny*(dFi_y_dy*1/norm([Fi_qx;Fi_qy])+Fi_qy*dFi_norm_dy_inv));

% y-direction
dFi_pny_dx = ui_n*(dFi_y_dx*1/norm([Fi_qx;Fi_qy])+Fi_qx*dFi_norm_dx_inv)*B+Fi_pnx*(-1/2)*B^3*(2*Fi_pnx*(dFi_y_dx*1/norm([Fi_qx;Fi_qy])+Fi_qx*dFi_norm_dx_inv)+2*Fi_pny*(dFi_y_dx*1/norm([Fi_qx;Fi_qy])+Fi_qy*dFi_norm_dx_inv));
dFi_pny_dy = ui_n*(dFi_y_dy*1/norm([Fi_qx;Fi_qy])+Fi_qy*dFi_norm_dy_inv)*B+Fi_pny*(-1/2)*B^3*(2*Fi_pnx*(dFi_x_dy*1/norm([Fi_qx;Fi_qy])+Fi_qx*dFi_norm_dy_inv)+2*Fi_pny*(dFi_y_dy*1/norm([Fi_qx;Fi_qy])+Fi_qy*dFi_norm_dy_inv));

end 






function [Fi_x, Fi_y, dFi_Ax_dx,dFi_Ax_dy, dFi_Ay_dx,dFi_Ay_dy,dFi_R_ijx_dx,dFi_R_ijx_dy,dFi_R_ijy_dx, dFi_R_ijy_dy ] = Field (sigma_ij, ri, rk,i) 
% Loading the goal locations to be used by this function
run('Simulation_Parameters.m')

x = ri(1,1);
y = ri(2,1);


%defining the goal location of agent i.
xg = q_goal(1,i);
yg = q_goal(2,i);

%% Set of equations related to the attractive field
% Nomenclature:
% Fi_A: Attractive force as defined by Eqs 6a and 6b 

%Constant: 
%rg = [xg,yg] Goal location. 

%Variable: 
%r= [x,y] Current location for agent i. 

Fi_Ax = -(x-xg)/((norm([x-xg,y-yg]))^2);
Fi_Ay = -(y-yg)/((norm([x-xg,y-yg]))^2);
Fi_A = [Fi_Ax;Fi_Ay];


% Partial derivatives of the attractive field: 
% x-direction 
dFi_Ax_dx = ((x-xg)^2+(y-yg)^2)/((x-xg)^2+(y-yg)^2)^2;
dFi_Ax_dy = 2*(x-xg)*(y-yg)/((x-xg)^2+(y-yg)^2)^2;  


% y-direction 
dFi_Ay_dx = 2*(x-xg)*(y-yg)/((x-xg)^2+(y-yg)^2)^2; 
dFi_Ay_dy= ((x-xg)^2+(y-yg)^2)/((x-xg)^2+(y-yg)^2)^2;


%% Set of equations related to the repulsive field 
% Fi_R_ij: Repulsive force of agent j w.r.t agent i as defined by ref [27]
% eqn 20a and 20b.

%Constant: 
%rj = [xj; yj] Location of agent we want to move away from. This should be a
% matrix of agents up to k agents

%Variable: 
%r= [x,y] Current location for agent i. 

% Also need to know number of repulsive agents, the repulsive force should
% be a matrix with the # of columns representing the number of critical
% agents 

% Initializing variables 
% Reference functions 
Fi_R_ijx = zeros(1,m);
Fi_R_ijy = zeros(1,m);
Fi_R_ij  = zeros(2,m);


% Partial derivatives 
dFi_R_ijx_dx = zeros(1,m);
dFi_R_ijx_dy = zeros(1,m);
dFi_R_ijy_dx = zeros(1,m);
dFi_R_ijy_dy = zeros(1,m);

for a = 1:m
    if a ~=i && rk(1,a)~= 0 && rk(2,a) ~=0 
        Fi_R_ijx(1,a) = (x-rk(1,a))/norm([x-rk(1,a), y-rk(2,a)]);
        Fi_R_ijy(1,a) = (y-rk(2,a))/norm([x-rk(1,a), y-rk(2,a)]);

        % Partial derivatives of the repulsive field:
        % x-direction
        dFi_R_ijx_dx(1,a) =  (((x-rk(1,a))^2+(y-rk(2,a))^2)^(-1/2))*(1-((x-rk(1,a))^2)*((x-rk(1,a))^2+(y-rk(2,a))^2)^(-1));
        dFi_R_ijx_dy(1,a) = -1/(2*((x-rk(1,a))^2+(y-rk(2,a))^2)^(3/2));

        % y-direction
        dFi_R_ijy_dx(1,a) = -1/(2*((x-rk(1,a))^2+(y-rk(2,a))^2)^(3/2));
        dFi_R_ijy_dy(1,a) = (((x-rk(1,a))^2+(y-rk(2,a))^2)^(-1/2))*(1-((x-rk(1,a))^2)*((x-rk(1,a))^2+(y-rk(2,a))^2)^(-1));
    end
end 
Fi_R_ij = [Fi_R_ijx;Fi_R_ijy]; 

%% Reference field force

% Nomenclature:
% Fi_x: Resultant force in the x-direction as a combination of attractive,
% repulsive forces and bump function.
% Fi_y: Resultant force in the y-direction as a combination of attractive,
% repulsive forces and bump function.


% Fi should be a 2x1 with Fi(1,1) being x direction and Fi(2,1) beingy
% direction. 



Fi = prod(ones(1,m)-sigma_ij)*Fi_A + sum(sigma_ij.*Fi_R_ij,2);

Fi_x = Fi(1);
Fi_y = Fi(2);
end 


