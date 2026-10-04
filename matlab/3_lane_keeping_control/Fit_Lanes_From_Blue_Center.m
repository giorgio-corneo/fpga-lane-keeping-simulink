% Extracted from the Simulink model simulink/lane_keeping_fpga_in_the_loop.slx
% Block: LANE KEEPING PROCESS/rotazione corsia/MATLAB Function
% The model is the source of truth; this copy exists so the code can be read on GitHub.

function [rgb_fit, validL, aL, bL, cL, validC, aC, bC, cC, validR, aR, bR, cR] = Fit_Lanes_From_Blue_Center(rgb_in)
%#codegen
%
% Modello:
%   x(y) = a*y^2 + b*y + c

[H,W,~] = size(rgb_in);

rgb_fit = zeros(H,W,3,'uint8');

validL = false; validC = false; validR = false;
aL = 0.0; bL = 0.0; cL = 0.0;
aC = 0.0; bC = 0.0; cC = 0.0;
aR = 0.0; bR = 0.0; cR = 0.0;

min_points = int32(20);
thickness = int32(1);

% copia input
for r = 1:H
    for c = 1:W
        rgb_fit(r,c,1) = rgb_in(r,c,1);
        rgb_fit(r,c,2) = rgb_in(r,c,2);
        rgb_fit(r,c,3) = rgb_in(r,c,3);
    end
end

% =========================================================
% 1) Fit centerline dai punti blu
% =========================================================
S0C = 0.0; S1C = 0.0; S2C = 0.0; S3C = 0.0; S4C = 0.0;
T0C = 0.0; T1C = 0.0; T2C = 0.0;
nC = int32(0);

for r = 1:H
    y = double(r);
    y2 = y*y;
    y3 = y2*y;
    y4 = y2*y2;

    for c = 1:W
        Rv = rgb_in(r,c,1);
        Gv = rgb_in(r,c,2);
        Bv = rgb_in(r,c,3);

        is_blue = Rv < 80 && Gv < 80 && Bv > 150;

        if is_blue
            x = double(c);

            S0C = S0C + 1.0;
            S1C = S1C + y;
            S2C = S2C + y2;
            S3C = S3C + y3;
            S4C = S4C + y4;

            T0C = T0C + x;
            T1C = T1C + y*x;
            T2C = T2C + y2*x;

            nC = nC + 1;
        end
    end
end

if nC < min_points
    return;
end

[okC, aC_tmp, bC_tmp, cC_tmp] = solve_quad_fit(S0C,S1C,S2C,S3C,S4C,T0C,T1C,T2C);

if ~okC
    return;
end

validC = true;
aC = aC_tmp;
bC = bC_tmp;
cC = cC_tmp;

% =========================================================
% 2) Usando la centerline, cerco i primi bianchi SX/DX
% =========================================================
S0L = 0.0; S1L = 0.0; S2L = 0.0; S3L = 0.0; S4L = 0.0;
T0L = 0.0; T1L = 0.0; T2L = 0.0;
nL = int32(0);

S0R = 0.0; S1R = 0.0; S2R = 0.0; S3R = 0.0; S4R = 0.0;
T0R = 0.0; T1R = 0.0; T2R = 0.0;
nR = int32(0);

for r = 1:H

    y = double(r);
    y2 = y*y;
    y3 = y2*y;
    y4 = y2*y2;

    xC = aC*y*y + bC*y + cC;
    colC = int32(round(xC));

    if colC < 1
        colC = int32(1);
    end
    if colC > W
        colC = int32(W);
    end

    % -------------------------
    % primo bianco a sinistra
    % -------------------------
    foundL = false;
    xL = int32(0);

    for c = colC:-1:1
        Rv = rgb_in(r,c,1);
        Gv = rgb_in(r,c,2);
        Bv = rgb_in(r,c,3);

        is_white = Rv > 200 && Gv > 200 && Bv > 200;

        if ~foundL && is_white
            xL = int32(c);
            foundL = true;
        end
    end

    if foundL
        x = double(xL);

        S0L = S0L + 1.0;
        S1L = S1L + y;
        S2L = S2L + y2;
        S3L = S3L + y3;
        S4L = S4L + y4;

        T0L = T0L + x;
        T1L = T1L + y*x;
        T2L = T2L + y2*x;

        nL = nL + 1;
    end

    % -------------------------
    % primo bianco a destra
    % -------------------------
    foundR = false;
    xR = int32(0);

    for c = colC:W
        Rv = rgb_in(r,c,1);
        Gv = rgb_in(r,c,2);
        Bv = rgb_in(r,c,3);

        is_white = Rv > 200 && Gv > 200 && Bv > 200;

        if ~foundR && is_white
            xR = int32(c);
            foundR = true;
        end
    end

    if foundR
        x = double(xR);

        S0R = S0R + 1.0;
        S1R = S1R + y;
        S2R = S2R + y2;
        S3R = S3R + y3;
        S4R = S4R + y4;

        T0R = T0R + x;
        T1R = T1R + y*x;
        T2R = T2R + y2*x;

        nR = nR + 1;
    end
end

% =========================================================
% 3) Fit sinistra e destra
% =========================================================
if nL >= min_points
    [okL, aa, bb, cc] = solve_quad_fit(S0L,S1L,S2L,S3L,S4L,T0L,T1L,T2L);
    if okL
        validL = true;
        aL = aa; bL = bb; cL = cc;
    end
end

if nR >= min_points
    [okR, aa, bb, cc] = solve_quad_fit(S0R,S1R,S2R,S3R,S4R,T0R,T1R,T2R);
    if okR
        validR = true;
        aR = aa; bR = bb; cR = cc;
    end
end

% =========================================================
% 4) Disegno fit finale
% =========================================================
rgb_fit(:,:,:) = uint8(0);

for r = 1:H
    y = double(r);

    if validL
        x = aL*y*y + bL*y + cL;
        col = int32(round(x));
        for dc = -thickness:thickness
            cc = col + dc;
            if cc >= 1 && cc <= W
                rgb_fit(r,cc,1) = uint8(255);
                rgb_fit(r,cc,2) = uint8(255);
                rgb_fit(r,cc,3) = uint8(255);
            end
        end
    end

    if validC
        x = aC*y*y + bC*y + cC;
        col = int32(round(x));
        for dc = -thickness:thickness
            cc = col + dc;
            if cc >= 1 && cc <= W
                rgb_fit(r,cc,1) = uint8(0);
                rgb_fit(r,cc,2) = uint8(0);
                rgb_fit(r,cc,3) = uint8(255);
            end
        end
    end

    if validR
        x = aR*y*y + bR*y + cR;
        col = int32(round(x));
        for dc = -thickness:thickness
            cc = col + dc;
            if cc >= 1 && cc <= W
                rgb_fit(r,cc,1) = uint8(255);
                rgb_fit(r,cc,2) = uint8(255);
                rgb_fit(r,cc,3) = uint8(255);
            end
        end
    end
end

end


function [ok, a, b, c] = solve_quad_fit(S0,S1,S2,S3,S4,T0,T1,T2)
%#codegen

ok = false;
a = 0.0; b = 0.0; c = 0.0;

A11 = S4; A12 = S3; A13 = S2;
A21 = S3; A22 = S2; A23 = S1;
A31 = S2; A32 = S1; A33 = S0;

B1 = T2;
B2 = T1;
B3 = T0;

detA = A11*(A22*A33 - A23*A32) ...
     - A12*(A21*A33 - A23*A31) ...
     + A13*(A21*A32 - A22*A31);

if abs(detA) < 1e-9
    return;
end

detA_a = B1*(A22*A33 - A23*A32) ...
       - A12*(B2*A33 - A23*B3) ...
       + A13*(B2*A32 - A22*B3);

detA_b = A11*(B2*A33 - A23*B3) ...
       - B1*(A21*A33 - A23*A31) ...
       + A13*(A21*B3 - B2*A31);

detA_c = A11*(A22*B3 - B2*A32) ...
       - A12*(A21*B3 - B2*A31) ...
       + B1*(A21*A32 - A22*A31);

a = detA_a / detA;
b = detA_b / detA;
c = detA_c / detA;

ok = true;

end
