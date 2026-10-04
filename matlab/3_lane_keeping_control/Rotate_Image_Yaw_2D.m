% Extracted from the Simulink model simulink/lane_keeping_fpga_in_the_loop.slx
% Block: LANE KEEPING PROCESS/rotazione corsia/process_center_line
% The model is the source of truth; this copy exists so the code can be read on GitHub.

function rgb_out = Rotate_Image_Yaw_2D(rgb_in, yaw)
%#codegen
%
% yaw [rad]
% positivo = rotazione antioraria nell'immagine

[H,W,~] = size(rgb_in);

rgb_out = zeros(H,W,3,'uint8');

cx = double(W)/2.0;
cy = double(H)/2.0;

c = cos(yaw);
s = sin(yaw);

% =========================================================
% Inverse mapping:
% per ogni pixel output trovo da dove leggere nell'input
% =========================================================
for r_out = 1:H
    for c_out = 1:W

        x_out = double(c_out) - cx;
        y_out = double(r_out) - cy;

        % rotazione inversa: -yaw
        x_in =  c*x_out + s*y_out;
        y_in = -s*x_out + c*y_out;

        c_in = int32(round(x_in + cx));
        r_in = int32(round(y_in + cy));

        if r_in >= 1 && r_in <= H && c_in >= 1 && c_in <= W
            rgb_out(r_out,c_out,1) = rgb_in(r_in,c_in,1);
            rgb_out(r_out,c_out,2) = rgb_in(r_in,c_in,2);
            rgb_out(r_out,c_out,3) = rgb_in(r_in,c_in,3);
        end
    end
end

end
