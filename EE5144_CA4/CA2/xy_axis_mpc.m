function j = xy_axis_mpc(K,dt,p_0,v_0,a_0,pt,vt,at)
w1 = 100;
w2 = 1;
w3 = 1;
w4 = 1;
%% Construct the prediction matrix
[Tp, Tv, Ta, Bp, Bv, Ba] = getPredictionMatrix(K,dt,p_0,v_0,a_0);

%% Construct the optimization problem
H = [];
F = [];

A = [];
b = [];
%% Solve the optimization problem
J = quadprog(H,F,A,b,[],[],-3*ones(20,1),3*ones(20,1));

%% Apply the control
j = J(1);
end
