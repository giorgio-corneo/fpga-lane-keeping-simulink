% Extracted from the Simulink model simulink/lane_keeping_fpga_in_the_loop.slx
% Block: CONVERSIONE PER UART/RGB8_to_RGB7
% The model is the source of truth; this copy exists so the code can be read on GitHub.

function img7 = RGB8_to_RGB7(img8)
%#codegen
img7 = bitshift(img8, -1);
end
