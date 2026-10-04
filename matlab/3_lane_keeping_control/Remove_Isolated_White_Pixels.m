% Extracted from the Simulink model simulink/lane_keeping_fpga_in_the_loop.slx
% Block: LANE KEEPING PROCESS/pulizia rumore
% The model is the source of truth; this copy exists so the code can be read on GitHub.

function img_out = Remove_Isolated_White_Pixels(img_in)
% Mantiene validi solo i pixel bianchi che hanno almeno
% un pixel bianco adiacente negli 8 vicini.
%
% Input:
%   img_in  = immagine grayscale oppure binaria
%
% Output:
%   img_out = immagine filtrata

% Dimensioni immagine
[H, W] = size(img_in);

% Output inizializzato nero
img_out = zeros(H, W, 'like', img_in);

% Soglia per decidere se un pixel è bianco
threshold = 128;

% Scorro tutti i pixel interni
for r = 3:H-2
    for c = 3:W-2
        
        % Pixel centrale bianco?
        if img_in(r,c) > threshold
            
            % Controllo i 24 vicini
            hasNeighbor = false;
            
            for dr = -2:2
                for dc = -2:2
                    
                    % Salto il pixel centrale
                    if ~(dr == 0 && dc == 0)
                        if img_in(r+dr, c+dc) > threshold
                            hasNeighbor = true;
                        end
                    end
                    
                end
            end
            
            % Se ha almeno un vicino bianco, lo mantengo
            if hasNeighbor
                img_out(r,c) = img_in(r,c);
            end
            
        end
    end
end

end
