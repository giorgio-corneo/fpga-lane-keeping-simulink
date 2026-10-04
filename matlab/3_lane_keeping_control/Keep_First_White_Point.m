% Extracted from the Simulink model simulink/lane_keeping_fpga_in_the_loop.slx
% Block: LANE KEEPING PROCESS/lane_mask_corsia
% The model is the source of truth; this copy exists so the code can be read on GitHub.

function lane_mask = Keep_First_White_Point(binary_img)
%#codegen
% Input : binary_img [H x W] uint8
% Output: lane_mask [H x W] uint8
%
% Scopo:
%   Estrarre le due linee di corsia anche se entrambe si trovano
%   nella stessa metà dell'immagine.
%
% Convenzioni:
%   r = riga immagine, cresce verso il basso
%   c = colonna immagine, cresce verso destra
%   centro corsia = media delle due linee oppure linea singola +/- mezza corsia

[H,W] = size(binary_img);

lane_mask = zeros(H,W,'uint8');

% =========================================================
% Memoria centro dinamico tra frame
% =========================================================
persistent center_mem initialized

if isempty(initialized)
    center_mem = int32(W/2);
    initialized = true;
end

% =========================================================
% Parametri
% =========================================================
pixels_per_meter_x = 30.0;    % [px/m]
lane_width_m       = 3.5;     % [m]

lane_width_px = int32(round(lane_width_m * pixels_per_meter_x));
half_lane_px  = int32(round(0.5 * double(lane_width_px)));

% Accetto linee entro questa distanza dal centro stimato
% Più grande di una corsia perché la macchina può essere disallineata.
search_gate_px = int32(round(2.2 * double(lane_width_px)));

% Tolleranza sulla larghezza corsia
width_tol_px = 0.45 * double(lane_width_px);

% Salto massimo ammesso tra una riga e la successiva
max_shift = int32(round(0.18 * double(W)));

% Aggiornamento lento del centro per evitare oscillazioni
alpha_center = 0.10;

% Numero massimo di blob candidati per riga
MAX_CAND = 20;

% Memoria intra-frame
prev_left  = int32(0);
prev_right = int32(0);

has_left  = false;
has_right = false;

center = center_mem;

% =========================================================
% Scansione dal basso verso l'alto
% =========================================================
for r = H:-1:1

    % Saturazione centro
    if center < 1
        center = int32(1);
    end
    if center > W
        center = int32(W);
    end

    % =====================================================
    % 1) Trova tutti i blob bianchi sulla riga r
    % =====================================================
    cand = zeros(1,MAX_CAND,'int32');
    cand_count = int32(0);

    in_blob = false;
    blob_start = int32(0);

    for c = 1:W

        is_white = binary_img(r,c) > 0;

        if is_white && ~in_blob
            in_blob = true;
            blob_start = int32(c);
        end

        if (~is_white || c == W) && in_blob

            if is_white && c == W
                blob_end = int32(c);
            else
                blob_end = int32(c - 1);
            end

            blob_center = int32(round(0.5 * ...
                (double(blob_start) + double(blob_end))));

            if cand_count < MAX_CAND
                cand_count = cand_count + 1;
                cand(cand_count) = blob_center;
            end

            in_blob = false;
        end
    end

    % Se non ho candidati, mantengo centro precedente
    if cand_count == 0
        continue;
    end

    % =====================================================
    % 2) Selezione delle due linee migliori
    % =====================================================
    best_left  = int32(0);
    best_right = int32(0);
    found_pair = false;

    best_pair_cost = 1.0e12;

    % -----------------------------------------------------
    % Caso A: cerco una coppia con distanza circa lane_width_px
    % -----------------------------------------------------
    for i = 1:cand_count
        ci = cand(i);

        % Scarto candidati troppo lontani dal centro dinamico
        if abs(ci - center) > search_gate_px
            continue;
        end

        for j = 1:cand_count
            if j <= i
                continue;
            end

            cj = cand(j);

            if abs(cj - center) > search_gate_px
                continue;
            end

            % Ordino: left < right
            if ci < cj
                c_left  = ci;
                c_right = cj;
            else
                c_left  = cj;
                c_right = ci;
            end

            width_px = c_right - c_left;

            % Deve essere circa una larghezza corsia
            width_error = abs(double(width_px) - double(lane_width_px));

            if width_error > width_tol_px
                continue;
            end

            pair_center = int32(round(0.5 * ...
                (double(c_left) + double(c_right))));

            % Costo geometrico
            cost_width  = width_error;
            cost_center = abs(double(pair_center - center));

            % Costo di continuità se ho memoria
            cost_cont = 0.0;

            if has_left
                if abs(c_left - prev_left) > max_shift
                    cost_cont = cost_cont + 1.0e6;
                else
                    cost_cont = cost_cont + abs(double(c_left - prev_left));
                end
            end

            if has_right
                if abs(c_right - prev_right) > max_shift
                    cost_cont = cost_cont + 1.0e6;
                else
                    cost_cont = cost_cont + abs(double(c_right - prev_right));
                end
            end

            total_cost = 4.0*cost_width + 1.0*cost_center + 2.0*cost_cont;

            if total_cost < best_pair_cost
                best_pair_cost = total_cost;
                best_left  = c_left;
                best_right = c_right;
                found_pair = true;
            end
        end
    end

    % =====================================================
    % 3) Se non trovo coppia, scelgo la singola linea migliore
    % =====================================================
    found_single = false;
    best_single = int32(0);
    best_single_cost = 1.0e12;

    if ~found_pair

        for i = 1:cand_count
            ci = cand(i);

            if abs(ci - center) > search_gate_px
                continue;
            end

            % Costo base: vicinanza al centro dinamico
            cost = abs(double(ci - center));

            % Se ho una linea precedente, privilegio la continuità
            if has_left && has_right
                dL = abs(ci - prev_left);
                dR = abs(ci - prev_right);
                cost = cost + 2.0 * double(min(dL,dR));
            elseif has_left
                cost = cost + 2.0 * double(abs(ci - prev_left));
            elseif has_right
                cost = cost + 2.0 * double(abs(ci - prev_right));
            end

            if cost < best_single_cost
                best_single_cost = cost;
                best_single = ci;
                found_single = true;
            end
        end
    end

    % =====================================================
    % 4) Scrittura output e aggiornamento centro
    % =====================================================
    center_new = center;

    if found_pair

        % Disegno entrambe le linee
        lane_mask(r,best_left)  = uint8(255);
        lane_mask(r,best_right) = uint8(255);

        prev_left  = best_left;
        prev_right = best_right;

        has_left  = true;
        has_right = true;

        center_new = int32(round(0.5 * ...
            (double(best_left) + double(best_right))));

    elseif found_single

        % Disegno la singola linea trovata
        lane_mask(r,best_single) = uint8(255);

        % Devo decidere se è linea sinistra o destra della corsia.
        % Ipotesi:
        %   se la linea è a sinistra del centro stimato, allora è left;
        %   se è a destra, allora è right.
        %
        % Nota: anche se entrambe le linee stanno nella stessa metà immagine,
        % questa scelta resta coerente rispetto al centro dinamico stimato.

        if best_single < center

            % Linea sinistra vista
            prev_left = best_single;
            has_left = true;

            center_new = best_single + half_lane_px;

        else

            % Linea destra vista
            prev_right = best_single;
            has_right = true;

            center_new = best_single - half_lane_px;

        end

    else

        center_new = center;

    end

    % =====================================================
    % 5) Filtro del centro dinamico
    % =====================================================
    center = int32(round((1.0 - alpha_center)*double(center) + ...
                         alpha_center*double(center_new)));

    if center < 1
        center = int32(1);
    end
    if center > W
        center = int32(W);
    end

end

% =========================================================
% Salvo memoria per frame successivo
% =========================================================
center_mem = center;

end
