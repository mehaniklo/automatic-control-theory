A = [-1, 0, 0, 0; 
    0, 2,  0, 0; 
    0, 0, 3, 4; 
    0, 0, -4, 3];
B = [0; 5; 0; 6];
C = [1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 1, 0; 0, 0, 0, 1];
D = zeros(4, 1);
sys = ss(A, B, C, D);
x0 = [-7; -5; 2; 10];

% Desired decay rate
a = [0.1, 0.5, 1];
% mu = [100, 200, 300];

% Lyapunov inequality
cvx_begin sdp;
variable P(4,4);
variable Y(1,4);
P > 0.0001*eye(4);
P*A' + A*P + 2*a(1,1)*P + Y'*B' + B*Y <= 0;
cvx_end
K_1 = Y*inv(P);

cvx_begin sdp;
variable P(4,4);
variable Y(1,4);
P > 0.0001*eye(4);
P*A' + A*P + 2*a(1,2)*P + Y'*B' + B*Y <= 0;
cvx_end
K_2 = Y*inv(P);

cvx_begin sdp;
variable P(4,4);
variable Y(1,4);
P > 0.0001*eye(4);
P*A' + A*P + 2*a(1,3)*P + Y'*B' + B*Y <= 0;
cvx_end
K_3 = Y*inv(P);

eig1 = eig(A + B*K_1)
eig2 = eig(A + B*K_2)
eig3 = eig(A+B*K_3)