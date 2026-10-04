% Extracted from the Simulink model simulink/lane_keeping_fpga_in_the_loop.slx
% Block: Bicycle Model/bicycle model 
% The model is the source of truth; this copy exists so the code can be read on GitHub.

function [Xdot,Ydot,yaw_rate, Beta_hat]= fcn(delta,yaw, V_cog, L, l_r)


Beta_hat = atan(l_r/L*tan(delta));

Xdot = V_cog*cos(yaw+Beta_hat);
Ydot = V_cog*sin(yaw+Beta_hat);

yaw_rate = V_cog/L*tan(delta)*cos(Beta_hat);
