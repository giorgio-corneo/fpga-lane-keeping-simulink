% Extracted from the Simulink model simulink/lane_keeping_fpga_in_the_loop.slx
% Block: LANE KEEPING PROCESS/rotazione corsia/draw_lines
% The model is the source of truth; this copy exists so the code can be read on GitHub.

function [rgb_out, psi_img] = Draw_Lanes_With_Gray(gray, validL, aL, bL, cL, validR, aR, bR, cR)
%#codegen

[H,W] = size(gray);

rgb_out = zeros(H,W,3,'uint8');
psi_img = 0.0;
%curve_gray = zeros(H,W,'uint8');
%center_gray = zeros(H,W,'uint8');

for r = 1:H
    for c = 1:W
        val = gray(r,c);
        rgb_out(r,c,1) = val;
        rgb_out(r,c,2) = val;
        rgb_out(r,c,3) = val;
    end
end

thickness_lane   = int32(1);
thickness_center = int32(1);

for r = 1:H

    y = double(r);

    %xL = 0.0;
    %xR = 0.0;

    % =====================================================
    % Linea sinistra
    % =====================================================
    if validL
        xL = aL*y*y + bL*y + cL;
        colL = int32(round(xL));

        for dc = -thickness_lane:thickness_lane
            cc = colL + dc;

            if cc >= 1 && cc <= W
                % corsia bianca
                rgb_out(r,cc,1) = uint8(255);
                rgb_out(r,cc,2) = uint8(255);
                rgb_out(r,cc,3) = uint8(255);

                %curve_gray(r,cc) = uint8(255);
            end
        end
    end

    % =====================================================
    % Linea destra
    % =====================================================
    if validR
        xR = aR*y*y + bR*y + cR;
        colR = int32(round(xR));

        for dc = -thickness_lane:thickness_lane
            cc = colR + dc;

            if cc >= 1 && cc <= W
                % corsia bianca
                rgb_out(r,cc,1) = uint8(255);
                rgb_out(r,cc,2) = uint8(255);
                rgb_out(r,cc,3) = uint8(255);

                %curve_gray(r,cc) = uint8(255);
            end
        end
    end

       % =====================================================
        % Centerline blu
        % =====================================================
        if validL && validR
        
            % coefficienti centerline
        aC = 0.5*(aL + aR);
        bC = 0.5*(bL + bR);
        cC = 0.5*(cL + cR);
        
        % riga più bassa
        y0 = double(H);
        
        % derivata dx/dy alla base
        mC0 = 2.0*aC*y0 + bC;
        
        % angolo rispetto alla verticale immagine [rad]
        psi_img = atan(mC0);
        
        xC = aC*y*y + bC*y + cC;
        colC = int32(round(xC));
    
        for dc = -thickness_center:thickness_center
            cc = colC + dc;
    
            if cc >= 1 && cc <= W
                rgb_out(r,cc,1) = uint8(0);
                rgb_out(r,cc,2) = uint8(0);
                rgb_out(r,cc,3) = uint8(255);
    
                %center_gray(r,cc) = uint8(255);
            end
        end
    end

end

end
