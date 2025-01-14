function j = xy_axis_mpc(K,dt,p_0,v_0,a_0,pt,vt,at)
w1 = 100;
w2 = 1;
w3 = 1;
w4 = 1;
w5 = 1e4;
%% Construct the prediction matrix
[Tp, Tv, Ta, Bp, Bv, Ba] = getPredictionMatrix(K,dt,p_0,v_0,a_0);

%% Construct the optimization problem
H = blkdiag(w4*eye(K)+w1*(Tp'*Tp)+w2*(Tv'*Tv)+w3*(Ta'*Ta),w5*eye(2*K));
F = [w1*(Bp-pt)'*Tp+w2*(Bv-vt)'*Tv+w3*(Ba-at)'*Ta,zeros(1,2*K)];

A = [
    Tv, -eye(K), zeros(K);
    -Tv, -eye(K), zeros(K);
    Ta, zeros(K), -eye(K);
    -Ta, zeros(K), -eye(K)
];
b = [
    6*ones(K,1)-Bv;
    6*ones(K,1)+Bv;
    3*ones(K,1)-Ba;
    3*ones(K,1)+Ba
];

lower_bound = [-3*ones(K,1); zeros(2*K,1)];
upper_bound = [3*ones(K,1); inf(2*K,1)];  
%% Solve the optimization problem
J = quadprog(H,F,A,b,[],[],lower_bound,upper_bound);

%% Apply the control
j = J(1);
end
