% Extracted from the Simulink model simulink/lane_keeping_fpga_in_the_loop.slx
% Block: DETECTION LINE /RGB2GREY
% The model is the source of truth; this copy exists so the code can be read on GitHub.

function gray = RGB_to_GRAY(img)
%#codegen
% img  : 256x256x3 uint8
% gray : 256x256 uint8
%
% g = (R + G + B)/3

R = uint16(img(:,:,1));
G = uint16(img(:,:,2));
B = uint16(img(:,:,3));

gray16 = floor((R + G + B) / 3);
gray = uint8(gray16);
end
