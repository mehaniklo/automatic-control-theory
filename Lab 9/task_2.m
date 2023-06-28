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
% g_1 = 20^2;
% g_2 = 22^2;
% g_3 = 28^2;

% Lyapunov inequality
cvx_begin sdp;
variable P(4,4);
variable Y(1,4);
variable g_1;
minimize g_1;
P > 0.0001*eye(4);
[P, x0; x0', 1] > 0;
[P, Y'; Y, g_1] > 0;
P*A' + A*P + 2*a(1,1)*P + Y'*B' + B*Y <= 0;
cvx_end
K_1 = Y*inv(P);

cvx_begin sdp;
variable P(4,4);
variable Y(1,4);
variable g_2;
minimize g_2;
P > 0.0001*eye(4);
[P, x0; x0', 1] > 0;
[P, Y'; Y, g_2] > 0;
P*A' + A*P + 2*a(1,2)*P + Y'*B' + B*Y <= 0;
cvx_end
K_2 = Y*inv(P);

cvx_begin sdp;
variable P(4,4);
variable Y(1,4);
variable g_3;
minimize g_3;
P > 0.0001*eye(4);
[P, x0; x0', 1] > 0;
[P, Y'; Y, g_3] > 0;
P*A' + A*P + 2*a(1,3)*P + Y'*B' + B*Y <= 0;
cvx_end
K_3 = Y*inv(P);

a1 = sqrt(g_1);
e1 = eig(A + B*K_1);
a2 = sqrt(g_2);
e2 = eig(A + B*K_2);
a3 = sqrt(g_3);
e3 = eig(A+B*K_3);