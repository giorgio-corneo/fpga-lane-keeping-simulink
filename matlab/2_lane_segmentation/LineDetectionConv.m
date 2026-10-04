% Extracted from the Simulink model simulink/lane_keeping_fpga_in_the_loop.slx
% Block: DETECTION LINE /detecion line
% The model is the source of truth; this copy exists so the code can be read on GitHub.

function bw = LineDetectionConv(gray)
%#codegen
% gray: 128x128 uint8
% bw  : 128x128 uint8 binaria

bw = zeros(256,256,'uint8');

% Kernel Laplaciano 8-neighbors
K = int16([ 1  1  1;
            1 -8  1;
            1  1  1]);

th = int16(190);   % soglia iniziale da tarare

for i = 2:255
    for j = 2:255
        conv_val = int16(0);

        conv_val = conv_val + int16(gray(i-1,j-1)) * K(1,1);
        conv_val = conv_val + int16(gray(i-1,j  )) * K(1,2);
        conv_val = conv_val + int16(gray(i-1,j+1)) * K(1,3);

        conv_val = conv_val + int16(gray(i  ,j-1)) * K(2,1);
        conv_val = conv_val + int16(gray(i  ,j  )) * K(2,2);
        conv_val = conv_val + int16(gray(i  ,j+1)) * K(2,3);

        conv_val = conv_val + int16(gray(i+1,j-1)) * K(3,1);
        conv_val = conv_val + int16(gray(i+1,j  )) * K(3,2);
        conv_val = conv_val + int16(gray(i+1,j+1)) * K(3,3);

        % valore assoluto
        if conv_val < 0
            conv_val = -conv_val;
        end

        % threshold
        if conv_val >= th
            bw(i,j) = uint8(255);
        else
            bw(i,j) = uint8(0);
        end
    end
end
end
