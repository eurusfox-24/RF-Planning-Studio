clear; clc; close all;

%% -------------------------
% Baseline link parameters
%% -------------------------
c = 3e8;
f = 868e6; % Hz
lambda = c/f;
D = 4000; % m (link length)
Ptx = 14; % dBm
Gtx = 2; Grx = 2; % dBi
Lcable = 1; % dB (cable+connectors)
Srx = -120; % dBm 
fadeMargin = 10; % dB (safety margin)
htx = 20; hrx = 2; % m (gateway, sensor)
n = 2.7; % environment exponent
xObs = 0.5*D; % obstacle position
hObsAboveLOS = 3; % m above LOS line
clearRatio = 0.60; % 60% Fresnel clearance rule

%% -------------------------
% Helper formulas
%% -------------------------
FSPL = @(fHz, d) (20*log10(fHz) + 20*log10(d) - 147.55); 
PLlog = @(PL0, d, d0, n) (PL0 + 10*n*log10(d./d0)); 
EIRP = @(Ptx, Gtx, Lc) (Ptx + Gtx - Lc); 
PrxFun = @(EIRP, Grx, PL, m) (EIRP + Grx - PL - m); 
FresnelR = @(lambda, x, D) sqrt((lambda .* x .* (D-x)) ./ D);

%% -------------------------
% Distance sweep for plots
%% -------------------------
d = logspace(log10(200), log10(12000), 300); 
dkm = d/1000;
d0 = 100; 
PL0 = FSPL(f, d0); 

% Baseline Calculation
PL = PLlog(PL0, d, d0, n);
Prx = PrxFun(EIRP(Ptx,Gtx,Lcable), Grx, PL, fadeMargin);

%% -------------------------
% Plot setup
%% -------------------------
figure('Name','RF Planning Studio');
semilogx(dkm, Prx, 'b', 'LineWidth', 2); grid on; hold on;
yline(Srx, '--r', 'Sensitivity', 'LineWidth', 1.5);
xlabel('Distance (km)');
ylabel('Received power (dBm)');
title('RF Planning Studio Results');

%% =========================================================
% EXPERIMENTS (These now have access to all baseline variables)
%% =========================================================

%% (1) Increase gateway height by +5 m
htx_1 = htx + 5;
maxLOS_1 = 3.57*(sqrt(htx_1) + sqrt(hrx)); 
fprintf('\n(1) Height +5m => Max LOS: %.1f km\n', maxLOS_1);

%% (2) Increase antenna gains to 5 dBi
Gtx_2 = 5; Grx_2 = 5;
Prx_2 = PrxFun(EIRP(Ptx,Gtx_2,Lcable), Grx_2, PL, fadeMargin);
semilogx(dkm, Prx_2, 'g', 'LineWidth', 2); 
fprintf('(2) Gains to 5 dBi => Green curve added\n');

%% (3) Change environment exponent n
n_3 = 3.5; 
PL_3 = PLlog(PL0, d, d0, n_3);
Prx_3 = PrxFun(EIRP(Ptx,Gtx,Lcable), Grx, PL_3, fadeMargin);
semilogx(dkm, Prx_3, 'm', 'LineWidth', 2); 
fprintf('(3) Exponent n=%.1f => Magenta curve added\n', n_3);

%% (4) Move gateway location (Fresnel Study)
xObs_4 = 0.3*D; 
rF_4 = FresnelR(lambda, xObs_4, D);
req_4 = clearRatio * rF_4;
fresnelOK_4 = (hObsAboveLOS < req_4);
if fresnelOK_4; status = 'PASS'; else; status = 'FAIL'; end
fprintf('(4) Obstacle at 30%% => r=%.2f m, 60%%=%.2f m => %s\n', rF_4, req_4, status);

legend('Baseline','Sensitivity','Gain=5 dBi','n=3.5','Location','best');
