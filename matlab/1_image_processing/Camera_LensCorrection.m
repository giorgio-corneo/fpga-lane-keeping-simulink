% Extracted from the Simulink model simulink/lane_keeping_fpga_in_the_loop.slx
% Block: Image processing subsystem/lens corretion
% The model is the source of truth; this copy exists so the code can be read on GitHub.

    function undistorted = Camera_LensCorrection(rgb)
%#codegen
% Correzione distorsione lente su frame RGB
%
% Input:
%   rgb : frame RGB uint8 HxWx3
%   fx, fy : focali [px]
%   cx, cy : centro ottico [px]
%   k1,k2,k3 : distorsione radiale
%   t1,t2 : distorsione tangenziale

fx = 950;
fy = 950;
cx = 540;
cy = 960;
k1 = 0;
k2 = 0;
k3 = 0;
p1 = 0;
p2 = 0;
%
% Output:
%   undistorted : frame RGB corretto

[H,W,~] = size(rgb);

undistorted = zeros(H,W,3,'uint8');

for v_out = 1:H
    for u_out = 1:W

        % =====================================================
        % 1) Coordinate pixel corrette -> coordinate normalizzate
        % =====================================================
        x = (double(u_out) - cx) / fx;
        y = (double(v_out) - cy) / fy;

        % =====================================================
        % 2) Modello di distorsione
        %    Da pixel ideale/corretto -> pixel distorto
        % =====================================================
        r2 = x*x + y*y;
        r4 = r2*r2;
        r6 = r4*r2;

        radial = 1 + k1*r2 + k2*r4 + k3*r6;

        x_dist = x*radial + 2*p1*x*y + p2*(r2 + 2*x*x);
        y_dist = y*radial + p1*(r2 + 2*y*y) + 2*p2*x*y;

        % =====================================================
        % 3) Coordinate normalizzate distorte -> pixel input
        % =====================================================
        u_in = fx*x_dist + cx;
        v_in = fy*y_dist + cy;

        ui = int32(round(u_in));
        vi = int32(round(v_in));

        % =====================================================
        % 4) Campionamento immagine originale
        % =====================================================
        if ui >= 1 && ui <= W && vi >= 1 && vi <= H
            undistorted(v_out,u_out,1) = rgb(vi,ui,1);
            undistorted(v_out,u_out,2) = rgb(vi,ui,2);
            undistorted(v_out,u_out,3) = rgb(vi,ui,3);
        end
    end
end

end
