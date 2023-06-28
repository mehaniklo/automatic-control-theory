% given data
A = [3, -11, -7, 5; -11, 3, -5, 7; -7, -5, 3, 11; 5, 7, 11, 3];
B = [2; 4; 2; 4];
C = [2, -2, 2, 2; 2, 4, -2, 4];
D = 0;
sys = ss(A, B, C, D);
x0 = [10; -15; 1; -5];
x_0 = [-10; 0; 0; 5];

% Desired decay rate
a_k = 1.2;
a_l = 1.1;

% Lyapunov inequality
cvx_begin sdp;
variable P(4,4);
variable Y(1,4);
P > 0.0001*eye(4);
P*A' + A*P + 2*a_k*P + Y'*B' + B*Y <= 0;
variable Q(4,4);
variable Y1(4,2);
Q > 0.0001*eye(4);
A'*Q + Q*A + 2*a_l*Q + C'*Y1' + Y1*C <= 0;
cvx_end

K = Y*inv(P)
eig(A + B*K)
L = inv(Q)*Y1
eig(A + L*C)