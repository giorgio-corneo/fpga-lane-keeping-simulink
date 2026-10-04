% Extracted from the Simulink model simulink/lane_keeping_fpga_in_the_loop.slx
% Block: LANE KEEPING PROCESS/rotazione corsia/fitting_lnes
% The model is the source of truth; this copy exists so the code can be read on GitHub.

function [lane_img, validL, aL, bL, cL, validR, aR, bR, cR] = Fit_Left_Right_Lanes(lane_mask)
%#codegen

[H,W] = size(lane_mask);
lane_img = lane_mask;

center_fixed = double(W)/2.0;

validL = false;
validR = false;

aL = 0.0;
bL = 0.0;
cL = 0.0;

aR = 0.0;
bR = 0.0;
cR = 0.0;

[validL1, validR1, aL1, bL1, cL1, aR1, bR1, cR1] = ...
    fit_lanes_internal(lane_mask, false, ...
                       0.0, 0.0, 0.0, ...
                       0.0, 0.0, 0.0, ...
                       center_fixed);

if ~(validL1 && validR1)

    validL = validL1;
    validR = validR1;

    aL = aL1;
    bL = bL1;
    cL = cL1;

    aR = aR1;
    bR = bR1;
    cR = cR1;

    return;
end

[validL2, validR2, aL2, bL2, cL2, aR2, bR2, cR2] = ...
    fit_lanes_internal(lane_mask, true, ...
                       aL1, bL1, cL1, ...
                       aR1, bR1, cR1, ...
                       center_fixed);

if validL2 && validR2

    validL = validL2;
    validR = validR2;

    aL = aL2;
    bL = bL2;
    cL = cL2;

    aR = aR2;
    bR = bR2;
    cR = cR2;

else

    validL = validL1;
    validR = validR1;

    aL = aL1;
    bL = bL1;
    cL = cL1;

    aR = aR1;
    bR = bR1;
    cR = cR1;

end

end


function [validL, validR, aL, bL, cL, aR, bR, cR] = ...
    fit_lanes_internal(lane_mask, use_corrected_center, ...
                       aLprev, bLprev, cLprev, ...
                       aRprev, bRprev, cRprev, ...
                       center_fixed)
%#codegen

persistent center_mem initialized

[H,W] = size(lane_mask);

if isempty(initialized)
    center_mem = center_fixed;
    initialized = true;
end

validL = false;
validR = false;

aL = 0.0;
bL = 0.0;
cL = 0.0;

aR = 0.0;
bR = 0.0;
cR = 0.0;

% =========================================================
% Parametri fisici
% =========================================================
pixels_per_meter_x = 30.0;
lane_width_m       = 3.5;

lane_width_px      = lane_width_m * pixels_per_meter_x;
half_lane_width_px = 0.5 * lane_width_px;

% Tolleranza per capire se distanza = 1 corsia o 2 corsie
width_tol_px = 0.35 * lane_width_px;

% Gate: accetto punti entro 1 corsia dal centro stimato
center_gate_px = lane_width_px;

% Media mobile centro
alpha_center = 0.20;

% =========================================================
% Accumulatori sinistra
% =========================================================
nL = 0.0;
SyL = 0.0;
Sy2L = 0.0;
Sy3L = 0.0;
Sy4L = 0.0;
SxL = 0.0;
SxyL = 0.0;
Sxy2L = 0.0;

% =========================================================
% Accumulatori destra
% =========================================================
nR = 0.0;
SyR = 0.0;
Sy2R = 0.0;
Sy3R = 0.0;
Sy4R = 0.0;
SxR = 0.0;
SxyR = 0.0;
Sxy2R = 0.0;

for r = 1:H

    y = double(r);

    if use_corrected_center

        xLprev = aLprev*y*y + bLprev*y + cLprev;
        xRprev = aRprev*y*y + bRprev*y + cRprev;

        measured_width = xRprev - xLprev;

        is_one_lane = abs(measured_width - lane_width_px) <= width_tol_px;
        is_two_lane = abs(measured_width - 2.0*lane_width_px) <= width_tol_px;

        if is_one_lane

            % Caso corretto: due linee della stessa corsia
            center_fit = 0.5*(xLprev + xRprev);

        elseif is_two_lane

            % Caso esterno: le due linee sono separate da circa 2 corsie
            dist_to_left  = abs(center_mem - xLprev);
            dist_to_right = abs(center_mem - xRprev);

            if dist_to_right < dist_to_left
                % linea più vicina è la destra:
                % centro corsia = mezzo lane width a sinistra della destra
                center_fit = xRprev - half_lane_width_px;
            else
                % linea più vicina è la sinistra:
                % centro corsia = mezzo lane width a destra della sinistra
                center_fit = xLprev + half_lane_width_px;
            end

        else

            % Caso incerto: mantengo memoria
            center_fit = center_mem;

        end

        % Media mobile
        center_mem = (1.0 - alpha_center)*center_mem + alpha_center*center_fit;

        center_dyn = center_mem;

        if center_dyn < 1.0
            center_dyn = 1.0;
        end

        if center_dyn > double(W)
            center_dyn = double(W);
        end

    else

        center_dyn = center_fixed;

    end

    for col = 1:W

        if lane_mask(r,col) > 0

            x = double(col);

            y2 = y*y;
            y3 = y2*y;
            y4 = y2*y2;

            assign_left = false;
            assign_right = false;

            if ~use_corrected_center

                if x < center_dyn
                    assign_left = true;
                else
                    assign_right = true;
                end

            else

                dist_from_center = abs(x - center_dyn);

                if dist_from_center <= center_gate_px

                    if x < center_dyn
                        assign_left = true;
                    else
                        assign_right = true;
                    end

                end
            end

            if assign_left

                nL = nL + 1.0;

                SyL   = SyL   + y;
                Sy2L  = Sy2L  + y2;
                Sy3L  = Sy3L  + y3;
                Sy4L  = Sy4L  + y4;

                SxL   = SxL   + x;
                SxyL  = SxyL  + x*y;
                Sxy2L = Sxy2L + x*y2;

            elseif assign_right

                nR = nR + 1.0;

                SyR   = SyR   + y;
                Sy2R  = Sy2R  + y2;
                Sy3R  = Sy3R  + y3;
                Sy4R  = SyR + y4;   % CORRETTO SOTTO
                Sy4R  = Sy4R - SyR + y4;

                SxR   = SxR   + x;
                SxyR  = SxyR  + x*y;
                Sxy2R = Sxy2R + x*y2;

            end
        end
    end
end

if nL >= 6.0

    [okL, aaL, bbL, ccL] = solve_parabola_ls( ...
        SyL, Sy2L, Sy3L, Sy4L, ...
        SxL, SxyL, Sxy2L, nL);

    if okL
        aL = aaL;
        bL = bbL;
        cL = ccL;
        validL = true;
    end
end

if nR >= 6.0

    [okR, aaR, bbR, ccR] = solve_parabola_ls( ...
        SyR, Sy2R, Sy3R, Sy4R, ...
        SxR, SxyR, Sxy2R, nR);

    if okR
        aR = aaR;
        bR = bbR;
        cR = ccR;
        validR = true;
    end
end

end


function [ok, a, b, c] = solve_parabola_ls( ...
    Sy, Sy2, Sy3, Sy4, Sx, Sxy, Sxy2, n)
%#codegen

ok = false;

a = 0.0;
b = 0.0;
c = 0.0;

A11 = Sy4;
A12 = Sy3;
A13 = Sy2;

A21 = Sy3;
A22 = Sy2;
A23 = Sy;

A31 = Sy2;
A32 = Sy;
A33 = n;

B1 = Sxy2;
B2 = Sxy;
B3 = Sx;

detA = A11*(A22*A33 - A23*A32) ...
     - A12*(A21*A33 - A23*A31) ...
     + A13*(A21*A32 - A22*A31);

if abs(detA) > 1e-9

    detA1 = B1*(A22*A33 - A23*A32) ...
          - A12*(B2*A33 - A23*B3) ...
          + A13*(B2*A32 - A22*B3);

    detA2 = A11*(B2*A33 - A23*B3) ...
          - B1*(A21*A33 - A23*A31) ...
          + A13*(A21*B3 - B2*A31);

    detA3 = A11*(A22*B3 - B2*A32) ...
          - A12*(A21*B3 - B2*A31) ...
          + B1*(A21*A32 - A22*A31);

    a = detA1 / detA;
    b = detA2 / detA;
    c = detA3 / detA;

    ok = true;
end

end
