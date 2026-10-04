% Extracted from the Simulink model simulink/lane_keeping_fpga_in_the_loop.slx
% Block: LANE KEEPING PROCESS/PURE_PURSUIT_CONTROL
% The model is the source of truth; this copy exists so the code can be read on GitHub.

function [rgb_out, valid, target_x_px, target_y_px, delta, alpha, rho] = PurePursuit_From_Fitted_Center(rgb_in, validC, aC, bC, cC)
%#codegen
%
% =========================================================
% PURE PURSUIT CONTROL FROM FITTED BLUE CENTERLINE
% =========================================================
%
% Input:
%   rgb_in  : immagine RGB 256x256, tipicamente rgb_fit
%   validC  : true se la centerline è valida
%   aC,bC,cC: coefficienti della centerline:
%
%               x(y) = aC*y^2 + bC*y + cC
%
% Output:
%   rgb_out      : immagine debug
%   valid        : true se il target lookahead è valido
%   target_x_px  : colonna target [px]
%   target_y_px  : riga target [px]
%   delta        : angolo sterzo [rad]
%   alpha        : angolo del lookahead point [rad]
%   rho        : curvatura Pure Pursuit [1/m]
%
% Convenzioni:
%   X positivo avanti
%   Y positivo sinistra
%   delta > 0 sterza a sinistra
%
% Immagine:
%   riga cresce verso il basso
%   colonna cresce verso destra

[H,W,~] = size(rgb_in);

% =========================================================
% Output iniziali
% =========================================================
rgb_out = rgb_in;

valid = false;

target_x_px = int32(round(double(W)/2.0));
target_y_px = int32(H);

delta = 0.0;
alpha = 0.0;
rho = 0.0;

if ~validC
    return;
end

% =========================================================
% Parametri veicolo / immagine
% =========================================================
L = 3;                 % [m] passo veicolo
delta_max = 1;        % [rad] circa 30 deg

pixels_per_meter_x = 25.6;   % [px/m] scala laterale
pixels_per_meter_y = 25.6;   % [px/m] scala longitudinale (BEV 10MX10M = 256pxX256px)

d_la = 5.0;           % [m] distanza lookahead desiderata

% Posizione ego in pixel
ego_x_px = double(W)/2.0;
ego_y_px = double(H);

% =========================================================
% 1) Cerca sulla parabola il punto più vicino a d_la
% =========================================================
%
% Modello immagine:
%   xC(r) = aC*r^2 + bC*r + cC
%
% Conversione metrica:
%   X = (ego_y_px - r)/pixels_per_meter_y
%   Y = -(xC - ego_x_px)/pixels_per_meter_x
%
% Distanza:
%   d = sqrt(X^2 + Y^2)
%
% Cerchiamo il punto della centerline con:
%   d ~= d_la
%

best_err = 1.0e9;
best_r = int32(H);
best_c = int32(round(ego_x_px));

found = false;

for r = 1:H

    y = double(r);

    xC = aC*y*y + bC*y + cC;
    c_px = int32(round(xC));

    if c_px >= 1 && c_px <= W

        X_m = (ego_y_px - y) / pixels_per_meter_y;
        Y_m = -(xC - ego_x_px) / pixels_per_meter_x;

        % Consideriamo solo punti davanti al veicolo
        if X_m > 0.05

            d_m = sqrt(X_m*X_m + Y_m*Y_m);
            err = abs(d_m - d_la);

            if err < best_err
                best_err = err;
                best_r = int32(r);
                best_c = c_px;
                found = true;
            end
        end
    end
end

if ~found
    return;
end

target_x_px = best_c;
target_y_px = best_r;

% =========================================================
% 2) Coordinate metriche del lookahead point
% =========================================================
r_la = double(target_y_px);
c_la = double(target_x_px);

X_la = (ego_y_px - r_la) / pixels_per_meter_y;      % [m]
Y_la = -(c_la - ego_x_px) / pixels_per_meter_x;     % [m]

d_la = sqrt(X_la*X_la + Y_la*Y_la);

if d_la < 0.1
    return;
end

% =========================================================
% 3) Pure Pursuit
% =========================================================
%
% Angolo del punto lookahead:
%
%   alpha = atan2(Y_la, X_la)
%
% Curvatura arco: (1/raggio)
%
%   rho = 2*sin(alpha)/d_la
%
% Poiché:
%
%   sin(alpha) = Y_la/d_la
%
% allora equivalentemente:
%
%   rho = 2*Y_la/d_la^2
%
% Sterzo cinematic bicycle:
%
%   delta = atan(L*rho)
%

alpha = atan2(Y_la, X_la);

rho = 2.0 * sin(alpha) / d_la;

delta = atan(L * rho);

% Saturazione sterzo
if delta > delta_max
    delta = delta_max;
end

if delta < -delta_max
    delta = -delta_max;
end

valid = true;

% =========================================================
% 4) Debug grafico
% =========================================================

% Disegno centerline verde
for r = 1:H

    y = double(r);
    xC = aC*y*y + bC*y + cC;
    c_px = int32(round(xC));

    if c_px >= 1 && c_px <= W
        rgb_out(r,c_px,1) = uint8(0);
        rgb_out(r,c_px,2) = uint8(255);
        rgb_out(r,c_px,3) = uint8(0);
    end
end

% Disegno ego in rosso
ego_c_i = int32(round(ego_x_px));
ego_r_i = int32(round(ego_y_px));

for dr = -2:2
    for dc = -2:2
        rr = ego_r_i + dr;
        cc = ego_c_i + dc;

        if rr >= 1 && rr <= H && cc >= 1 && cc <= W
            rgb_out(rr,cc,1) = uint8(255);
            rgb_out(rr,cc,2) = uint8(0);
            rgb_out(rr,cc,3) = uint8(0);
        end
    end
end

% Disegno target lookahead in giallo
for dr = -3:3
    for dc = -3:3
        rr = target_y_px + dr;
        cc = target_x_px + dc;

        if rr >= 1 && rr <= H && cc >= 1 && cc <= W
            rgb_out(rr,cc,1) = uint8(255);
            rgb_out(rr,cc,2) = uint8(255);
            rgb_out(rr,cc,3) = uint8(0);
        end
    end
end

% Disegno segmento ego-target in magenta, discretizzato semplice
Nline = int32(60);

for i = 0:Nline

    lambda = double(i) / double(Nline);

    cc_f = ego_x_px + lambda*(double(target_x_px) - ego_x_px);
    rr_f = ego_y_px + lambda*(double(target_y_px) - ego_y_px);

    cc = int32(round(cc_f));
    rr = int32(round(rr_f));

    if rr >= 1 && rr <= H && cc >= 1 && cc <= W
        rgb_out(rr,cc,1) = uint8(255);
        rgb_out(rr,cc,2) = uint8(0);
        rgb_out(rr,cc,3) = uint8(255);
    end
end

end
