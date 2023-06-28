%given data
A = [1, -2, 3; 2, -3, 2; -2, 1, -4];
B = [3; 1; -1];
C = [0, 0, 0];
D = 0;
sys = ss(A, B, C, D);
x_1 = [4; 3; -3];
x_2 = [3; 3; -2];
t_1 = 3;

% Сontrollability matrix
U = ctrb(sys);
rank_U = rank(U);

% Checking the state vector for belonging to the controllability subspace
U_x_1 = [U, x_1];
U_x_2 = [U, x_2];
rank_U_x_1 = rank(U_x_1);
rank_U_x_2 = rank(U_x_2);

% System in Jordan basis (complex)
[P_complex, A_jordan_complex] = eig(A);
B_jordan_complex = inv(P_complex)*B;

% System in Jordan basis (real)
[P_real, A_jordan_real] = cdf2rdf(P_complex, A_jordan_complex);
B_jordan_real = inv(P_real)*B;

% Hautus matrices
e1_hautus = [(A - A_jordan_complex(1,1)*eye(3)) B];
rank_e1_hautus = rank(e1_hautus);

e2_hautus = [(A - A_jordan_complex(2,2)*eye(3)) B];
rank_e2_hautus = rank(e2_hautus);

e3_hautus = [(A - A_jordan_complex(3,3)*eye(3)) B];
rank_e3_hautus = rank(e3_hautus);

% controllability gramian for time t_1
opt = gramOptions('TimeIntervals',[0 t_1]);
Gr_t_1 = gram(sys, 'c', opt);
eig_Gr = eig(Gr_t_1);

% input to lead state vector from 0 to x_1 during time t_1
syms t real;
exp_a = expm(A'*(t_1-t));
exp_a_simplify = simplify(exp_a, "Steps", 100);

u_x_1 = B'*exp_a_simplify*pinv(Gr_t_1)*x_1;
u_x_1_simplify = simplify(u_x_1);