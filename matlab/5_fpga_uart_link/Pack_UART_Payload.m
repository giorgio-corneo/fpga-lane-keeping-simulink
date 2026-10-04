% Extracted from the Simulink model simulink/lane_keeping_fpga_in_the_loop.slx
% Block: CONVERSIONE PER UART/packetizer uart
% The model is the source of truth; this copy exists so the code can be read on GitHub.

function tx = Pack_UART_Payload(img)
%#codegen
% img: 256x256x3 uint8
% tx : 65536*3x1 uint8

H = 256;
W = 256;

tx = zeros(65536*3,1,'uint8');

k = 1;

for i = 1:H
    for j = 1:W
        tx(k) = img(i,j,1); k = k + 1;
        tx(k) = img(i,j,2); k = k + 1;
        tx(k) = img(i,j,3); k = k + 1;
    end
end
