% Extracted from the Simulink model simulink/lane_keeping_fpga_in_the_loop.slx
% Block: Image processing subsystem/ROI_and_BEV
% The model is the source of truth; this copy exists so the code can be read on GitHub.

function bev = BEV_From_RGB(rgb, X_CAMERA, Z_CAMERA, ROLL_CAMERA, PITCH_CAMERA, YAW_CAMERA)
%#codegen
% Input : rgb [1080 x 1920 x 3] uint8
% Output: bev [256 x 256 x 3] uint8
%avanti veicolo X  -> profondità camera Zc
%sinistra veicolo Y -> -Xc
%alto veicolo Z     -> -Yc
% ROLL PITCH E YAW CAMERA è GIà IN RADIANTI

A = [ 0  1  0;
      0  0 -1;
      1  0  0 ];

% =========================================================
% 1) Dimensioni immagine originale
% =========================================================
Himg = 1080;
Wimg = 1920;

% =========================================================
% 2) Dimensioni BEV
% =========================================================
Hbev = 256;
Wbev = 256;

bev = zeros(Hbev, Wbev, 3, 'uint8');

% =========================================================
% 3) Intrinseci camera
% =========================================================
fx = 950;
fy = 950;
cx = 960;
cy = 540;

K = [fx  0 cx 0;
      0 fy cy 0;
      0  0  1 0];

% =========================================================
% 4) Posizione camera rispetto al veicolo/mondo
% =========================================================
C = [1.2;
     0;
     1.6];

% =========================================================
% 5) Rotazioni camera [Roll Pitch Yaw] in gradi
% =========================================================
roll  = 0 * pi/180;
pitch = 7.875 * pi/180;
yaw   = -0.4 * pi/180 + YAW_CAMERA; %è l'angolo di imbardata che deve raddrizzare il bev

cr = cos(roll);  sr = sin(roll);
cp = cos(pitch); sp = sin(pitch);
cyaw = cos(yaw); syaw = sin(yaw);

Rx = [1  0   0;
      0  cr sr;
      0  -sr  cr];

Ry = [ cp 0 -sp;
       0  1 0;
      sp 0 cp];

Rz = [cyaw syaw 0;
      -syaw  cyaw 0;
      0     0    1];

% Convenzione usata:YAW-ROLL-PITCH
R_vehicle = Ry * Rx * Rz;

%rotazione assi telecamera = world
R = A*R_vehicle;

% Traslazione mondo -> camera
t = -R * C;

% Matrice di proiezione
P = K * [R t;
        0 0 0 1];

% =========================================================
% 6) Area reale rappresentata nella BEV
% =========================================================
% X = direzione avanti [m]
% Y = direzione laterale [m]
Xmin = 5 ;
Xmax = 15;

Ymin = -5 ;
Ymax = 5 ;

% =========================================================
% 7) Inverse mapping: BEV -> mondo -> immagine
% =========================================================
for rb = 1:Hbev

    % rb = 1 parte lontana, rb = Hbev parte vicina
    Xw = Xmax - (rb-1) * (Xmax - Xmin) / (Hbev-1);
    
    for cb = 1:Wbev

        Yw = Ymin + (cb-1) * (Ymax - Ymin) / (Wbev-1);
        

        % Piano strada
        Zw = 0.0;

        Ow = [Xw; Yw; Zw; 1.0];

        q = P * Ow;

        s = q(3);%zc

        if s > 0

            u = q(1) / s;%xc/zc
            v = q(2) / s;%yc/zc
        
            ui = int32(round(u));
            vi = int32(round(v));
        
            if ui >= 1 && ui <= Wimg && vi >= 1 && vi <= Himg
                bev(rb, cb, 1) = rgb(vi, ui, 1);
                bev(rb, cb, 2) = rgb(vi, ui, 2);
                bev(rb, cb, 3) = rgb(vi, ui, 3);
            else
                % Davanti alla camera ma fuori immagine
                bev(rb, cb, 1) = 255;   % rosso
                bev(rb, cb, 2) = 0;
                bev(rb, cb, 3) = 0;
            end
        
        else
            % Dietro la camera oppure profondità negativa/nulla
            bev(rb, cb, 1) = 0;
            bev(rb, cb, 2) = 0;
            bev(rb, cb, 3) = 255;       % blu
        end
    end
end
