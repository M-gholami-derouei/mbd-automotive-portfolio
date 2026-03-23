% Deviation-variable model linearized around (v_op, F_op, v_lead = v_op)
% States: Δx = [v - v_op ; d]
% Input:  Δu = F_traction - F_op
% Disturbance: Δw = v_lead - v_op
% Outputs: Δy = [v - v_op ; d]
% Nominal values set via mpcobj.Model.Nominal so the MPC
% converts between absolute Simulink signals and deviations.

run('parameters.m')

A = [-(C_D * rho * A_f * v_op)/m, 0;
     -1, 0];
B = [1/m ; 0];
C = eye(2);
D = zeros(2, 1);
E = [0 ; 1]; % disturbance input matrix for v_lead

% linearized model used for MPC prediction
plant_c = ss(A, [B E], C, [D zeros(2,1)]);
plant   = c2d(plant_c, Ts);
plant   = setmpcsignals(plant, 'MV', 1, 'MD', 2);

% MPC object creation
mpcobj = mpc(plant, Ts, Np, Nc);

% Constraints
mpcobj.MV.Min = F_brake;
mpcobj.MV.Max = F_max;

% Weights
% mpcobj.Weights.OV     = [1 2];   
% mpcobj.Weights.MV     = 0.1;     
% mpcobj.Weights.MVRate = 0.1;

% Scalling
mpcobj.OV(1).ScaleFactor = 30;    % typical velocity range m/s
mpcobj.OV(2).ScaleFactor = 200;   % typical gap range m
mpcobj.MV(1).ScaleFactor = 4500;  % max traction force N

mpcobj.Model.Nominal.U = [F_op; v_op];   % [MV nominal; MD nominal]
mpcobj.Model.Nominal.Y = [v_op; 0];   % [v nominal; d nominal]

% MV — Manipulated Variable: the signal that the controller commands. In our case: F_traction. This is what the optimizer solves for.
% MD — Measured Disturbance: an external signal the controller knows about but cannot control. In our case: v_lead. The MPC uses it in its prediction.
% OV — Output Variable: what the controller measures and tracks. In our case: v and d.
% DV — Disturbance Variable: in the mpc() syntax this refers to unmeasured disturbances — things the controller cannot see. In our case we have none.

review(mpcobj)
% This checks for problems in your design — unstable prediction model, ill-conditioned weights, infeasible constraints.