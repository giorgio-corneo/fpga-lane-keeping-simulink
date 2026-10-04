% Extracted from the Simulink model simulink/lane_keeping_fpga_in_the_loop.slx
% Block: CONVERSIONE PER UART/depacketizer uart
% The model is the source of truth; this copy exists so the code can be read on GitHub.

function img = Unpack_UART_Payload(rx)
%#codegen
% rx  : 65536x1 uint8
% img : 256x256 uint8

H = 256;
W = 256;

img = zeros(H, W, 'uint8');

k = 1;
for i = 1:H
    for j = 1:W
        img(i,j) = 2*rx(k);%usiamo 2* per fare shift e dare 8 bit al colore
        k = k + 1;
    end
end
end
