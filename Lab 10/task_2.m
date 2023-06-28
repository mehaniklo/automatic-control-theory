%given data
A = [6, 7, -4, 8; 0, 9, 0, 15; 10, 13, -6, 19; 0, -6, 0, -9];
B = [-8, 0; 4, 0; -6, 0; -2, 0];
x0 = [1; -1; 1; -1];
C = eye(4);
D = zeros(4, 2);

% LQR
Q_1 = 2*eye(4);
R_1 = 2*eye(2);
[P_1,K_1,L_1] = icare(A,B,Q_1,R_1,0,eye(4),0);
K_1 = -K_1;
J_1 = x0'*P_1*x0;

% Controller with desired decay rate
a = [0.3, 3];

cvx_begin sdp;
variable P(4,4);
variable Y(2,4);
P > 0.0001*eye(4);
P*A' + A*P + 2*a(1,1)*P + Y'*B' + B*Y <= 0;
cvx_end
K_2 = Y*inv(P);

cvx_begin sdp;
variable P(4,4);
variable Y(2,4);
P > 0.0001*eye(4);
P*A' + A*P + 2*a(1,2)*P + Y'*B' + B*Y <= 0;
cvx_end
K_3 = Y*inv(P);

eig1 = eig(A + B*K_1)
eig2 = eig(A + B*K_2)
eig3 = eig(A + B*K_3)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% hyeta
% 2 task
K3 = K_1;
R3 = R_1;
Q3 = Q_1;
% plant parameters
% A = [-6 19 10 -13; 0 -9 0 6; -4 8 6 -7; 0 -15 0 9];
% B = [2 0; 1 0; 3 0; 2 0];
A = [6, 7, -4, 8; 0, 9, 0, 15; 10, 13, -6, 19; 0, -6, 0, -9];
B = [-8, 0; 4, 0; -6, 0; -2, 0];

% initial conditions
% x0 = [1; -2; -1; 2];
x0 = [1; -1; 1; -1];

% modal regulator
%K_modal = [286/5 -(2238/5) -(598/5) 334; 286/5 -(2238/5) -(598/5) 334];
% K_modal = [6/5 -21/10 -1/10 -7/2; 6/5 -21/10 -1/10 -7/2];
K_modal = K_2;

% LMI regulator

alpha = 3;
cvx_begin sdp
variable P_lmi(4,4)
variable Y_lmi(2,4)
P_lmi > 0.00001*eye(4);
P_lmi*A' + A*P_lmi + 2*alpha*P_lmi + Y_lmi'*B' + B*Y_lmi <= 0;
cvx_end
% K_lmi = Y_lmi*inv(P_lmi);
K_lmi = K_3;

% J finding
lqr_j_sum = sum(out.lqr_sum.signals.values);
modal_j_sum = sum(out.modal_sum.signals.values);
lmi_j_sum = sum(out.lmi_sum.signals.values);

lqr_j_qqq = zeros(4001, 1);
modal_j_qqq = zeros(4001, 1);
lmi_j_qqq = zeros(4001, 1);
lqr_j_qqq(1) = out.lqr_sum.signals.values(1);
modal_j_qqq(1) = out.modal_sum.signals.values(1);
lmi_j_qqq(1) = out.lmi_sum.signals.values(1);

for i = 2:4001
    lqr_j_qqq(i) = lqr_j_qqq(i-1) + out.lqr_sum.signals.values(i);
    modal_j_qqq(i) = modal_j_qqq(i-1) + out.modal_sum.signals.values(i);
    lmi_j_qqq(i) = lmi_j_qqq(i-1) + out.lmi_sum.signals.values(i);
end