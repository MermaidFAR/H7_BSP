clear; clc; close all;

repo = "D:/KnowledgeBase/MyProject/BSP/H7_BSP";
outdir = fullfile(repo, "sysid", "position_loop", "results");
logfile = fullfile(repo, "sysid", "position_loop", "logs", "rtt_angle_sysid_after_single_calc.log");

if ~exist(outdir, "dir")
    mkdir(outdir);
end

txt = fileread(logfile);
expr = "Angle Tgt:(-?\d+) Now:(-?\d+) Err:(-?\d+) SpdOut:(-?\d+) SpdNow:(-?\d+) YawOut:(-?\d+)";
tok = regexp(txt, expr, "tokens");
n = numel(tok);
raw = zeros(n, 6);
for i = 1:n
    raw(i, :) = str2double(tok{i}) ./ 1000.0;
end

Ts_log = 0.1;
t = (0:n-1)' * Ts_log;
r = raw(:, 1);
y = raw(:, 2);
u = raw(:, 4);
v = raw(:, 5);

idx_change = find(abs(diff(r)) > 0.2) + 1;
if isempty(idx_change)
    first_excited = 1;
else
    first_excited = max(1, idx_change(1) - 5);
end
id_idx = first_excited:n;

data_u2y = iddata(y(id_idx), u(id_idx), Ts_log, ...
    "InputName", "yaw_speed_cmd", ...
    "OutputName", "yaw_angle");
data_u2y = detrend(data_u2y, 0);

split = floor(0.70 * length(id_idx));
id_data = data_u2y(1:split);
val_data = data_u2y(split+1:end);

fprintf("Parsed %d frames, %.1fs, excited window starts at %.1fs\n", n, t(end), t(id_idx(1)));
fprintf("Target range %.3f..%.3f rad, yaw range %.3f..%.3f rad, speed-cmd range %.3f..%.3f rad/s\n", ...
    min(r), max(r), min(y), max(y), min(u), max(u));

models = {};
model_names = {};
fits = [];

for np = 1:3
    for nz = 0:max(0, np-1)
        try
            opt = tfestOptions("Display", "off", "InitializeMethod", "all");
            sys = tfest(id_data, np, nz, 0, opt);
            [~, fit, ~] = compare(val_data, sys);
            models{end+1} = sys; %#ok<SAGROW>
            model_names{end+1} = sprintf("tfest_np%d_nz%d", np, nz); %#ok<SAGROW>
            fits(end+1) = fit; %#ok<SAGROW>
            fprintf("%s validation fit: %.2f%%\n", model_names{end}, fit);
        catch ME
            fprintf("tfest np=%d nz=%d failed: %s\n", np, nz, ME.message);
        end
    end
end

for nx = 1:4
    try
        opt = ssestOptions("Display", "off", "Focus", "simulation");
        sys = ssest(id_data, nx, opt);
        [~, fit, ~] = compare(val_data, sys);
        models{end+1} = sys; %#ok<SAGROW>
        model_names{end+1} = sprintf("ssest_nx%d", nx); %#ok<SAGROW>
        fits(end+1) = fit; %#ok<SAGROW>
        fprintf("%s validation fit: %.2f%%\n", model_names{end}, fit);
    catch ME
        fprintf("ssest nx=%d failed: %s\n", nx, ME.message);
    end
end

[best_fit, best_i] = max(fits);
best_sys = models{best_i};
best_name = model_names{best_i};
fprintf("\nBest model: %s, validation fit %.2f%%\n", best_name, best_fit);
disp(best_sys);

% A compact physics prior for the outer-loop plant:
% yaw angle ~= integral of measured gyro response.  Fit SpdOut -> SpdNow,
% then append 1/s.  This avoids over-trusting direct u->angle fits from
% closed-loop data and yaw wrap/drift.
data_u2v = iddata(v(id_idx), u(id_idx), Ts_log, ...
    "InputName", "yaw_speed_cmd", ...
    "OutputName", "yaw_rate");
data_u2v = detrend(data_u2v, 0);
id_v = data_u2v(1:split);
val_v = data_u2v(split+1:end);
speed_models = {};
speed_names = {};
speed_fits = [];
for np = 1:3
    for nz = 0:max(0, np-1)
        try
            opt = tfestOptions("Display", "off", "InitializeMethod", "all");
            sysv = tfest(id_v, np, nz, 0, opt);
            [~, fitv, ~] = compare(val_v, sysv);
            speed_models{end+1} = sysv; %#ok<SAGROW>
            speed_names{end+1} = sprintf("speed_tfest_np%d_nz%d", np, nz); %#ok<SAGROW>
            speed_fits(end+1) = fitv; %#ok<SAGROW>
            fprintf("%s validation fit: %.2f%%\n", speed_names{end}, fitv);
        catch ME
            fprintf("speed tfest np=%d nz=%d failed: %s\n", np, nz, ME.message);
        end
    end
end
[best_speed_fit, best_speed_i] = max(speed_fits);
best_speed_sys = speed_models{best_speed_i};
fprintf("\nBest speed model: %s, validation fit %.2f%%\n", speed_names{best_speed_i}, best_speed_fit);
disp(best_speed_sys);

s = tf("s");
plant = minreal(tf(best_speed_sys) / s);

dt = 0.001;
plant_d = c2d(ss(plant), dt, "tustin");
[Ad, Bd, Cd, Dd] = ssdata(plant_d);

sim_t = 0:dt:8;
step_profile = zeros(size(sim_t));
step_profile(sim_t >= 0.5 & sim_t < 2.0) = 0.5;
step_profile(sim_t >= 2.0 & sim_t < 3.5) = -0.5;
step_profile(sim_t >= 3.5 & sim_t < 5.5) = 1.0;
step_profile(sim_t >= 5.5) = 0.0;

current.Kp = 7.0;
current.Ki = 1.0;
out_max = 50.0;
iout_max = 15.0;

function [cost, met, yy, uu] = eval_pi(Kp, Ki, Ad, Bd, Cd, Dd, r, dt, out_max, iout_max)
    nx = size(Ad, 1);
    x = zeros(nx, 1);
    integ = 0;
    yy = zeros(size(r));
    uu = zeros(size(r));
    e_prev = 0;
    for k = 1:numel(r)
        yk = Cd * x;
        e = r(k) - yk;
        if Ki > 1e-9
            integ = max(min(integ, iout_max / Ki), -iout_max / Ki);
        else
            integ = 0;
        end
        integ = integ + dt * e;
        iout = Ki * integ;
        iout = max(min(iout, iout_max), -iout_max);
        uk = Kp * e + iout;
        uk = max(min(uk, out_max), -out_max);
        x = Ad * x + Bd * uk;
        yy(k) = yk + Dd * uk;
        uu(k) = uk;
        e_prev = e; %#ok<NASGU>
    end
    err = r(:) - yy(:);
    iae = trapz(abs(err)) * dt;
    u_rms = sqrt(mean(uu.^2));
    du_rms = sqrt(mean(diff(uu).^2));
    overs = 0;
    changes = find(abs(diff(r)) > 0.2) + 1;
    for ii = 1:numel(changes)
        st = changes(ii);
        en = min(numel(r), st + round(1.2 / dt));
        mag = abs(r(st) - r(st-1));
        if mag < 1e-6
            continue;
        end
        seg = yy(st:en);
        if r(st) > r(st-1)
            peak = max(seg);
        else
            peak = min(seg);
        end
        overs = overs + max(0, abs(peak - r(st)) / mag);
    end
    cost = iae + 0.020 * u_rms + 0.002 * du_rms + 3.0 * overs;
    met.iae = iae;
    met.u_rms = u_rms;
    met.du_rms = du_rms;
    met.overshoot_sum = overs;
    met.rmse = sqrt(mean(err.^2));
    met.max_abs_err = max(abs(err));
    met.max_abs_u = max(abs(uu));
end

[current_cost, current_met, current_y, current_u] = eval_pi( ...
    current.Kp, current.Ki, Ad, Bd, Cd, Dd, step_profile, dt, out_max, iout_max);

fprintf("\nCurrent PI Kp=%.3f Ki=%.3f cost=%.4f rmse=%.4f max|u|=%.2f\n", ...
    current.Kp, current.Ki, current_cost, current_met.rmse, current_met.max_abs_u);

Kp_grid = linspace(1.0, 12.0, 56);
Ki_grid = linspace(0.0, 5.0, 51);
best = struct("cost", inf, "Kp", NaN, "Ki", NaN, "met", []);

for a = 1:numel(Kp_grid)
    for b = 1:numel(Ki_grid)
        Kp = Kp_grid(a);
        Ki = Ki_grid(b);
        [cost, met] = eval_pi(Kp, Ki, Ad, Bd, Cd, Dd, step_profile, dt, out_max, iout_max);
        if cost < best.cost
            best.cost = cost;
            best.Kp = Kp;
            best.Ki = Ki;
            best.met = met;
        end
    end
end

obj = @(x) eval_pi(exp(x(1)), exp(x(2)), Ad, Bd, Cd, Dd, step_profile, dt, out_max, iout_max);
x0 = log([best.Kp, max(best.Ki, 0.05)]);
opt = optimset("Display", "off", "MaxIter", 300, "TolX", 1e-4, "TolFun", 1e-5);
xopt = fminsearch(obj, x0, opt);
Kp_opt = exp(xopt(1));
Ki_opt = exp(xopt(2));
[opt_cost, opt_met, opt_y, opt_u] = eval_pi(Kp_opt, Ki_opt, Ad, Bd, Cd, Dd, step_profile, dt, out_max, iout_max);

fprintf("Grid best PI Kp=%.4f Ki=%.4f cost=%.4f rmse=%.4f max|u|=%.2f\n", ...
    best.Kp, best.Ki, best.cost, best.met.rmse, best.met.max_abs_u);
fprintf("Optimized PI Kp=%.4f Ki=%.4f cost=%.4f rmse=%.4f max|u|=%.2f\n", ...
    Kp_opt, Ki_opt, opt_cost, opt_met.rmse, opt_met.max_abs_u);

% Conservative field candidate: split the difference when the optimizer tries
% to move too aggressively from the proven hardware setting.
Kp_cons = 0.55 * current.Kp + 0.45 * Kp_opt;
Ki_cons = 0.55 * current.Ki + 0.45 * Ki_opt;
[cons_cost, cons_met, cons_y, cons_u] = eval_pi(Kp_cons, Ki_cons, Ad, Bd, Cd, Dd, step_profile, dt, out_max, iout_max);
fprintf("Conservative PI Kp=%.4f Ki=%.4f cost=%.4f rmse=%.4f max|u|=%.2f\n", ...
    Kp_cons, Ki_cons, cons_cost, cons_met.rmse, cons_met.max_abs_u);

fig = figure("Position", [80 80 1300 850]);
tiledlayout(3, 1);

nexttile;
compare(val_data, best_sys);
title(sprintf("Direct u=SpdOut -> y=Yaw model: %s, fit %.1f%%", best_name, best_fit));
grid on;

nexttile;
plot(sim_t, step_profile, "k--", "LineWidth", 1.0); hold on;
plot(sim_t, current_y, "Color", [0.0 0.32 0.78], "LineWidth", 1.1);
plot(sim_t, cons_y, "Color", [0.0 0.55 0.32], "LineWidth", 1.1);
plot(sim_t, opt_y, "Color", [0.85 0.2 0.1], "LineWidth", 1.1);
legend("target", "current", "conservative", "optimized", "Location", "best");
ylabel("yaw angle (rad)");
title("Simulated outer-loop step response on identified speed-loop + integrator plant");
grid on;

nexttile;
plot(sim_t, current_u, "Color", [0.0 0.32 0.78], "LineWidth", 1.0); hold on;
plot(sim_t, cons_u, "Color", [0.0 0.55 0.32], "LineWidth", 1.0);
plot(sim_t, opt_u, "Color", [0.85 0.2 0.1], "LineWidth", 1.0);
legend("current", "conservative", "optimized", "Location", "best");
ylabel("speed cmd (rad/s)");
xlabel("time (s)");
title("Controller output, saturation limit +/-50 rad/s");
grid on;

figpath = fullfile(outdir, "h7_yaw_matlab_sysid_pid_opt.png");
exportgraphics(fig, figpath, "Resolution", 160);

result.best_model = best_name;
result.best_fit_percent = best_fit;
result.best_speed_fit_percent = best_speed_fit;
result.current = current_met;
result.current.Kp = current.Kp;
result.current.Ki = current.Ki;
result.optimized = opt_met;
result.optimized.Kp = Kp_opt;
result.optimized.Ki = Ki_opt;
result.optimized.cost = opt_cost;
result.conservative = cons_met;
result.conservative.Kp = Kp_cons;
result.conservative.Ki = Ki_cons;
result.conservative.cost = cons_cost;
result.figure = figpath;
save(fullfile(outdir, "h7_yaw_matlab_sysid_pid_opt.mat"), "result", "best_sys", "best_speed_sys", "plant");
writelines(jsonencode(result, PrettyPrint=true), fullfile(outdir, "h7_yaw_matlab_sysid_pid_opt.json"));

fprintf("\nSaved figure: %s\n", figpath);
fprintf("Saved result JSON: %s\n", fullfile(outdir, "h7_yaw_matlab_sysid_pid_opt.json"));
