function [deltaCmd, ctrlOut] = afs_controller(P, t, gyro, q, qbar, phiCmd)
%AFS_CONTROLLER Discrete fin-tab flight controller (runs at P.sens.Ts).
%   Roll      : PID on roll angle; roll angle is integrated from the gyro,
%               as the flight computer would do it.
%   Pitch/yaw : PID holding the body axis on P.ctrl.targetDir.
%   Desired angular accelerations become moments through the nominal
%   inertia, are divided by the (floored) dynamic pressure (gain
%   scheduling), and are mapped onto the four tabs with the pseudo-inverse
%   of the tab effectiveness matrix.
%
%   ctrlOut = [phiEst; ePitch; eYaw; Mcmd(3); enabled]
persistent phi iRoll iPy
if isempty(phi)
    phi = 0; iRoll = 0; iPy = zeros(2, 1);
end
Ts = P.sens.Ts;

% Roll-angle estimate from the roll-rate gyro
phi = phi + gyro(1)*Ts;

% Pointing error: small-angle rotation (body y, z) taking body x onto the target
C   = afs_quat2dcm(q/norm(q));
dB  = C.'*P.ctrl.targetDir;
eB  = cross([1; 0; 0], dB);
ePy = eB(2:3);

enabled = P.ctrl.enable > 0 && t >= P.ctrl.tEnable && qbar >= P.ctrl.qbarMin;
deltaCmd = zeros(P.geo.nFins, 1);
Mcmd = zeros(3, 1);
if enabled
    eRoll = phiCmd - phi;
    iRoll = min(max(iRoll + eRoll*Ts, -P.ctrl.roll.iMax), P.ctrl.roll.iMax);
    iPy   = min(max(iPy + ePy*Ts, -P.ctrl.py.iMax), P.ctrl.py.iMax);

    accCmd = [P.ctrl.roll.Kp*eRoll + P.ctrl.roll.Ki*iRoll - P.ctrl.roll.Kd*gyro(1);
              P.ctrl.py.Kp*ePy     + P.ctrl.py.Ki*iPy     - P.ctrl.py.Kd*gyro(2:3)];
    Mcmd = P.ctrl.Inom*accCmd;
    deltaCmd = P.ctrl.Binv*Mcmd/max(qbar, P.ctrl.qbarFloor);
    deltaCmd = min(max(deltaCmd, -P.ctrl.defMax), P.ctrl.defMax);
else
    iRoll = 0; iPy = zeros(2, 1);
end
ctrlOut = [phi; ePy; Mcmd; double(enabled)];
end
