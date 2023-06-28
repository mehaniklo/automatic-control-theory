% given data
A = [0, 3, 0, 0; -3, 0,  0, 0; 0, 0, 0, 1; 0, 0, -1, 0];
B = [0; 0; 0; 0];
C = [2, 0, 0, 3];
D = 0;
sys = ss(A, B, C, D);
x0 = [5; -10; 2; -6];
x_0 = [2; 5; -2; 10];

% Desired decay rate
a = [1, 0.2, 3];
% mu = [100, 200, 300];

% Lyapunov inequality
cvx_begin sdp;
variable Q(4,4);
variable Y(4,1);
Q > 0.0001*eye(4);
A'*Q + Q*A + 2*a(1,1)*Q + C'*Y' + Y*C <= 0;
cvx_end
L_1 = inv(Q)*Y;

cvx_begin sdp;
variable Q(4,4);
variable Y(4,1);
Q > 0.0001*eye(4);
A'*Q + Q*A + 2*a(1,2)*Q + C'*Y' + Y*C <= 0;
cvx_end
L_2 = inv(Q)*Y;

cvx_begin sdp;
variable Q(4,4);
variable Y(4,1);
Q > 0.0001*eye(4);
A'*Q + Q*A + 2*a(1,3)*Q + C'*Y' + Y*C <= 0;
cvx_end
L_3 = inv(Q)*Y;

a(1,1)
L_1
eig(A + L_1*C)

a(1,2)
L_2
eig(A + L_2*C)

a(1,3)
L_3
eig(A + L_3*C)