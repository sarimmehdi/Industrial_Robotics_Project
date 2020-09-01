%%Trajectory generation

%LINE
T1 = transl(0.6, -0.5, 0.0); % START
T2 = transl(0.4, 0.5, 0.2);	% DESTINATION
res=20;
TTl=ctraj(T1,T2,res);
qq=ikine6s(p560,TTl);

% plot(p560,qq)

%signal for simulink

tfin=10;
timegran=size(qq);
timestep=timegran(1);
t=linspace(0,tfin,timestep);
qqs=[t' qq];

% qqs(:,6) = pi/4;

% ROBUST CONTROL PARAMETERS
D = [eye(6)*0; eye(6)];
matrixSize = 12;
while true
  A = rand (matrixSize, matrixSize);
  if rank (A) == matrixSize; break ; end % will be true nearly all the time    
end
Q = A' * A;

%Q4: ROBUST MODE STARTS HERE
% choose one of four initial joint coordinates
% q_curr = qz;
% % q_curr = qr;
% % q_curr = qs;
% % q_curr = qn;
% qd_curr = ones(1,6); 
% all_q = [];
% all_qtilde = [];
% all_t = [];
% new_t = [0:0.5:1];
% for time=1:timestep
%     q = jtraj(q_curr,qq(time,:),new_t);
%     q_curr = qq(time,:);
%     all_q = [all_q;q];
%     new_t = [new_t(end):0.5:new_t(end)+1];
% end
% qq = all_q;
% all_q = [];
% new_t = [0:0.05:1];
% % [timestep, coords] = size(qq);
% timestep = 5;
% %change these to observe different effects
% Kp = 1000.0;
% Kd = 100.0;
% matrixSize = 12;
% while true
%   A = rand (matrixSize, matrixSize);
%   if rank (A) == matrixSize; break ; end % will be true nearly all the time    
% end
% Q = A' * A;
% for time=1:timestep
%     [tempq, tempqd, qdd] = jtraj(q_curr,qq(time,:),new_t);
%     [t_vals, q, qd] = pumanofriction.fdyn(1, ... 
%         @(pumanofriction, new_t, q, qd) mytorqfun_robust(pumanofriction, p560Unc, q, qd, qq(time,:), tempqd(end,:), qdd(end,:), Kp, Kd, Q), ... 
%         q_curr, qd_curr);
%     q_curr = q(end,:);
%     qd_curr = qd(end,:);
%     all_q = [all_q;q];
%     [q_rows, q_cols] = size(q);
%     all_qtilde = [all_qtilde;repmat(qq(time,:),q_rows,1)-q];
%     all_t = [all_t; t_vals+new_t(1)];
%     new_t = [new_t(end):0.05:new_t(end)+1];
%     disp(['COMPLETED ',num2str(time),' OUT OF ',num2str(timestep)])
% end
% 
% figure;
% subplot(2,1,1);
% plot(all_t, all_q)
% legend('q1','q2','q3','q4','q5','q6');
% xlabel('Time (s)'); 
% ylabel('Position (rad)');
% hold on
% subplot(2,1,2);
% plot(all_t, all_qtilde)
% legend('q1','q2','q3','q4','q5','q6');
% xlabel('Time (s)');
% ylabel('Position error (rad)');
% hold on
% 
% function u = mytorqfun_robust(robot, robot_unc, q, qd, qstar, qdstar, qddstar, Kp, Kd, Q)
%     M_hat = robot_unc.inertia(q);
%     C_hat = robot_unc.coriolis(q, qd);
%     g_hat = robot_unc.gravload(q);
%     n_hat = C_hat*transpose(qd) + transpose(g_hat);
%     M = robot.inertia(q);
%     C = robot.coriolis(q, qd);
%     g = robot.gravload(q);
%     n = C*transpose(qd) + transpose(g);
%     q = transpose(q);
%     qd = transpose(qd);
%     qstar = transpose(qstar);
%     qdstar = transpose(qdstar);
%     qddstar = transpose(qddstar);
%     x = norm(q-qstar)
%     x = transpose(q-qstar)
%     D = [eye(6)*0; eye(6)];
%     q_tilde = qstar - q;
%     qd_tilde = qdstar - qd;
%     epsilon = [q_tilde; qd_tilde];
%     z = transpose(D) * Q * epsilon;
%     z_unit_vector = z/norm(z);
%     Q_M = max(abs(qddstar)) + 0.1;
%     N_M = norm(n_hat-n) + 0.1;
%     M_M = norm(inv(M)) + 5;
%     K = [eye(6)*0 eye(6); -1*eye(6)*Kp -1*eye(6)*Kd];
%     alpha = 1e-10;
%     rho = (1/(1-alpha))*(alpha*Q_M + alpha*norm(K)*norm(epsilon) + 10*50);
%     w = rho * z_unit_vector;
%     y = qddstar + Kd*eye(6)*qd_tilde + Kp*eye(6)*q_tilde + w;
%     u = M_hat*y + n_hat;
%     u = transpose(u);
%     q = transpose(q);
%     qd = transpose(qd);
% end
%Q4: ROBUST MODE ENDS HERE