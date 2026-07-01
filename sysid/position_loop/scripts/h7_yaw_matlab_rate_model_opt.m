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

Ts = 0.1;
t = (0:n-1)' * Ts;
r = raw(:, 1);
y = raw(:, 2);
u = raw(:, 4);

% Rate from angle is less biased than raw gyro for this RTT log, and it is
% exactly the signal that integrates into the position loop output.
rate = [0; diff(y) ./ Ts];
rate = movmean(rate, 3);

excited = find(abs(u) > 0.08 | abs(diff([r(1); r])) > 0.2);
first_excited = max(1, min(excited) - 4);
last_excited = min(n, max(excited) + 8);
fit_idx = first_excited:last_excited;

data_rate = iddata(rate(fit_idx), u(fit_idx), Ts, ...
    "InputName", "yaw_speed_cmd", ...
    "OutputName", "yaw_rate_from_angle");
data_rate = detrend(data_rate, 0);

split = floor(0.72 * length(fit_idx));
id_data = data_rate(1:split);
val_data = data_rate(split+1:end);

fprintf("Frames=%d, fit window %.1f..%.1fs (%d samples)\n", n, t(fit_idx(1)), t(fit_idx(end)), numel(fit_idx));
fprintf("rate range %.3f..%.3f rad/s, cmd range %.3f..%.3f rad/s\n", min(rate), max(rate), min(u), max(u));

models = {};
names = {};
fits = [];

for na = 1:4
    for nb = 1:4
        for nk = 1:3
            try
                sys = arx(id_data, [na nb nk]);
                [~, fit, ~] = compare(val_data, sys);
                if isstable(sys)
                    models{end+1} = sys; %#ok<SAGROW>
                    names{end+1} = sprintf("arx_%d_%d_%d", na, nb, nk); %#ok<SAGROW>
                    fits(end+1) = fit; %#ok<SAGROW>
                end
            catch
            end
        end
    end
end

for nb = 1:4
    for nf = 1:4
        for nk = 1:3
            try
                opt = oeOptions("Display", "off", "Focus", "simulation");
                sys = oe(id_data, [nb nf nk], opt);
                [~, fit, ~] = compare(val_data, sys);
                if isstable(sys)
                    models{end+1} = sys; %#ok<SAGROW>
                    names{end+1} = sprintf("oe_%d_%d_%d", nb, nf, nk); %#ok<SAGROW>
                    fits(end+1) = fit; %#ok<SAGROW>
                end
            catch
            end
        end
    end
end

[best_open_fit, best_open_i] = max(fits);
fprintf("Best open-loop rate model by validation: %s fit %.2f%%\n", names{best_open_i}, best_open_fit);

function [yy, uu, vv] = sim_outer(sys_rate, r, Ts, Kp, Ki, out_max, iout_max, y0)
    ss_rate = ss(sys_rate);
    [A, B, C, D] = ssdata(ss_rate);
    nx = size(A, 1);
    x = zeros(nx, 1);
    integ = 0;
    yy = zeros(size(r));
    uu = zeros(size(r));
    vv = zeros(size(r));
    yy(1) = y0;
    for k = 1:numel(r)
        yk = yy(max(k-1, 1));
        e = r(k) - yk;
        integ = integ + Ts * e;
        if Ki > 1e-9
            integ = max(min(integ, iout_max / Ki), -iout_max / Ki);
        else
            integ = 0;
        end
        uk = Kp * e + Ki * integ;
        uk = max(min(uk, out_max), -out_max);
        vk = C * x + D * uk;
        x = A * x + B * uk;
        if k > 1
            yy(k) = yy(k-1) + Ts * vk;
        end
        uu(k) = uk;
        vv(k) = vk;
    end
end

function met = metrics(r, y, u, Ts)
    err = r(:) - y(:);
    met.rmse = sqrt(mean(err.^2));
    met.mae = mean(abs(err));
    met.iae = sum(abs(err)) * Ts;
    met.max_abs_err = max(abs(err));
    met.max_abs_u = max(abs(u));
    changes = find(abs(diff(r)) > 0.2) + 1;
    rise_vals = [];
    overs_vals = [];
    settle_vals = [];
    for ii = 1:numel(changes)
        idx = changes(ii);
        old = r(idx-1);
        new = r(idx);
        mag = abs(new - old);
        if mag < 0.7
            continue;
        end
        en = min(numel(r), idx + round(1.5 / Ts));
        seg = y(idx:en);
        if new > old
            peak = max(seg);
        else
            peak = min(seg);
        end
        overs_vals(end+1) = max(0, abs(peak - new) / mag * 100); %#ok<AGROW>
        for off = 1:(en - idx)
            progress = abs(y(idx+off) - old) / mag;
            if progress >= 0.9
                rise_vals(end+1) = off * Ts; %#ok<AGROW>
                break;
            end
        end
        band = 0.06 * mag;
        for off = 1:max(1, en - idx - 3)
            win = y(idx+off:min(en, idx+off+3));
            if all(abs(win - new) <= band)
                settle_vals(end+1) = off * Ts; %#ok<AGROW>
                break;
            end
        end
    end
    met.rise90_median = median_or_nan(rise_vals);
    met.overshoot_median = median_or_nan(overs_vals);
    met.settle_median = median_or_nan(settle_vals);
end

function v = median_or_nan(x)
    if isempty(x)
        v = NaN;
    else
        v = median(x);
    end
end

current_Kp = 7.0;
current_Ki = 1.0;
out_max = 50.0;
iout_max = 15.0;

% Select the model that best reproduces the measured closed-loop trace under
% the known current controller. This is more relevant than open-loop fit alone.
model_score = inf(1, numel(models));
for i = 1:numel(models)
    try
        [yc, uc] = sim_outer(models{i}, r(fit_idx), Ts, current_Kp, current_Ki, out_max, iout_max, y(fit_idx(1)));
        m = metrics(r(fit_idx), yc, uc, Ts);
        model_score(i) = m.rmse + 0.004 * max(0, fits(i) * -1);
    catch
        model_score(i) = inf;
    end
end
[~, chosen_i] = min(model_score);
sys_rate = models{chosen_i};
chosen_name = names{chosen_i};
fprintf("Chosen model by closed-loop reproduction: %s, validation fit %.2f%%\n", chosen_name, fits(chosen_i));
disp(sys_rate);

[current_y, current_u] = sim_outer(sys_rate, r(fit_idx), Ts, current_Kp, current_Ki, out_max, iout_max, y(fit_idx(1)));
current_met = metrics(r(fit_idx), current_y, current_u, Ts);
measured_met = metrics(r(fit_idx), y(fit_idx), u(fit_idx), Ts);

fprintf("Measured trace: RMSE %.3f, median rise90 %.3fs, median overshoot %.1f%%\n", ...
    measured_met.rmse, measured_met.rise90_median, measured_met.overshoot_median);
fprintf("Current simulated: RMSE %.3f, median rise90 %.3fs, median overshoot %.1f%%\n", ...
    current_met.rmse, current_met.rise90_median, current_met.overshoot_median);

% Optimize on the actual target sequence plus a hold period, not on an abstract
% impossible ideal. Penalize output effort and overshoot.
r_opt = [r(fit_idx); zeros(25, 1)];
function cost = objective(x, sys_rate, r_opt, Ts, out_max, iout_max, y0)
    Kp = exp(x(1));
    Ki = exp(x(2)) - 1e-4;
    [yy, uu] = sim_outer(sys_rate, r_opt, Ts, Kp, Ki, out_max, iout_max, y0);
    met = metrics(r_opt, yy, uu, Ts);
    cost = met.iae + 0.03 * sqrt(mean(uu.^2)) + 0.15 * max(0, met.overshoot_median - 8) + 0.10 * met.max_abs_u;
end

best = struct("cost", inf, "Kp", NaN, "Ki", NaN);
for Kp = linspace(2.0, 11.0, 73)
    for Ki = linspace(0.0, 3.0, 61)
        c = objective(log([Kp, Ki + 1e-4]), sys_rate, r_opt, Ts, out_max, iout_max, y(fit_idx(1)));
        if c < best.cost
            best.cost = c;
            best.Kp = Kp;
            best.Ki = Ki;
        end
    end
end

opt = optimset("Display", "off", "MaxIter", 250, "TolX", 1e-4);
x0 = log([best.Kp, best.Ki + 1e-4]);
x = fminsearch(@(z)objective(z, sys_rate, r_opt, Ts, out_max, iout_max, y(fit_idx(1))), x0, opt);
Kp_opt = min(max(exp(x(1)), 0.5), 15.0);
Ki_opt = min(max(exp(x(2)) - 1e-4, 0.0), 6.0);

% Limit the field recommendation to a plausible move from the proven setting.
Kp_cons = min(max(Kp_opt, 4.5), 8.5);
Ki_cons = min(max(Ki_opt, 0.0), 2.0);

[opt_y, opt_u] = sim_outer(sys_rate, r(fit_idx), Ts, Kp_opt, Ki_opt, out_max, iout_max, y(fit_idx(1)));
[cons_y, cons_u] = sim_outer(sys_rate, r(fit_idx), Ts, Kp_cons, Ki_cons, out_max, iout_max, y(fit_idx(1)));
opt_met = metrics(r(fit_idx), opt_y, opt_u, Ts);
cons_met = metrics(r(fit_idx), cons_y, cons_u, Ts);

fprintf("Optimized raw: Kp %.4f Ki %.4f -> RMSE %.3f, rise90 %.3fs, overshoot %.1f%%, max|u| %.2f\n", ...
    Kp_opt, Ki_opt, opt_met.rmse, opt_met.rise90_median, opt_met.overshoot_median, opt_met.max_abs_u);
fprintf("Conservative recommendation: Kp %.4f Ki %.4f -> RMSE %.3f, rise90 %.3fs, overshoot %.1f%%, max|u| %.2f\n", ...
    Kp_cons, Ki_cons, cons_met.rmse, cons_met.rise90_median, cons_met.overshoot_median, cons_met.max_abs_u);

fig = figure("Position", [80 80 1350 900]);
tiledlayout(3, 1);

nexttile;
compare(val_data, sys_rate);
title(sprintf("Yaw rate model %s, validation %.1f%%", chosen_name, fits(chosen_i)));
grid on;

nexttile;
tt = t(fit_idx) - t(fit_idx(1));
plot(tt, r(fit_idx), "k--", "LineWidth", 1.0); hold on;
plot(tt, y(fit_idx), "Color", [0.1 0.1 0.1], "LineWidth", 1.0);
plot(tt, current_y, "Color", [0.0 0.32 0.78], "LineWidth", 1.0);
plot(tt, cons_y, "Color", [0.0 0.55 0.32], "LineWidth", 1.1);
plot(tt, opt_y, "Color", [0.85 0.2 0.1], "LineWidth", 1.0);
legend("target", "measured", "current model", "conservative", "raw optimized", "Location", "best");
ylabel("yaw angle (rad)");
title("Closed-loop yaw position response on identified rate plant");
grid on;

nexttile;
plot(tt, u(fit_idx), "Color", [0.1 0.1 0.1], "LineWidth", 1.0); hold on;
plot(tt, current_u, "Color", [0.0 0.32 0.78], "LineWidth", 1.0);
plot(tt, cons_u, "Color", [0.0 0.55 0.32], "LineWidth", 1.1);
plot(tt, opt_u, "Color", [0.85 0.2 0.1], "LineWidth", 1.0);
legend("measured cmd", "current model", "conservative", "raw optimized", "Location", "best");
ylabel("speed cmd (rad/s)");
xlabel("time (s)");
title("Outer-loop speed command");
grid on;

figpath = fullfile(outdir, "h7_yaw_matlab_rate_model_pid_opt.png");
exportgraphics(fig, figpath, "Resolution", 160);

result = struct();
result.model = chosen_name;
result.rate_validation_fit_percent = fits(chosen_i);
result.measured = measured_met;
result.current = current_met;
result.current.Kp = current_Kp;
result.current.Ki = current_Ki;
result.optimized = opt_met;
result.optimized.Kp = Kp_opt;
result.optimized.Ki = Ki_opt;
result.conservative = cons_met;
result.conservative.Kp = Kp_cons;
result.conservative.Ki = Ki_cons;
result.figure = figpath;

jsonpath = fullfile(outdir, "h7_yaw_matlab_rate_model_pid_opt.json");
writelines(jsonencode(result, PrettyPrint=true), jsonpath);
save(fullfile(outdir, "h7_yaw_matlab_rate_model_pid_opt.mat"), "result", "sys_rate");

fprintf("Saved figure: %s\n", figpath);
fprintf("Saved JSON: %s\n", jsonpath);
