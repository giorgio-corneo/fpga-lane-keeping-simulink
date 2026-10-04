library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity depacketizer is
    generic (
        HEADER: INTEGER :=16#FF#;
        FOOTER: INTEGER :=16#F1#;
  
        FIFO_WIDTH : integer := 9;
        FIFO_DEPTH : integer := 15
    );
    port (
        clk   : in std_logic;
        aresetn : in std_logic;

        s_axis_tdata : in std_logic_vector(FIFO_WIDTH-2 downto 0);
        s_axis_tvalid : in std_logic; 
        s_axis_tready : out std_logic; 

        m_axis_tdata : out std_logic_vector(FIFO_WIDTH-2 downto 0);
        m_axis_tvalid : out std_logic; 
        m_axis_tready : in std_logic; 
        m_axis_tlast : out std_logic   
    );
end entity depacketizer;

architecture rtl of depacketizer is

    component FIFO is
        Generic(
            FIFO_WIDTH : integer := 9;
            FIFO_DEPTH : integer := 15 
        );
        Port(
            reset : in std_logic;
            clk   : in std_logic;
            
            din   : in std_logic_vector(FIFO_WIDTH-1 DOWNTO 0);
            dout  : out std_logic_vector(FIFO_WIDTH-1 DOWNTO 0);
            
            rd_en : in std_logic;
            wr_en : in std_logic;
                    
            full  : out std_logic;
            empty : out std_logic
        );
    end component;
    
    --segnali per FIFO
    signal  fifo_out_data, fifo_in_data : std_logic_vector(fifo_width-1 downto 0); 
    signal  write, read : std_logic;
    
    signal full_mem  : std_logic := '1';  
    signal empty_mem : std_logic := '1';  
    signal tlast_flag : std_logic;
    signal  fifo_data_mem : std_logic_vector(FIFO_WIDTH-1 DOWNTO 0); 
    
    signal  header_value : std_logic_vector(s_axis_tdata'RANGE);--conversione in std_logic di 16#FF#
    signal  footer_value : std_logic_vector(s_axis_tdata'RANGE);--conversione in std_logic di 16#F1# 
    
    type state_type is (state0,stateff,state1); --stato di sttesa dell'header, stato in cui lo trovo e stato in cui scrivo in fifo
    signal next_state, current_state : state_type := state0;
      
begin

    FIFO_inst: FIFO
        Generic map(
            FIFO_WIDTH => FIFO_WIDTH,
            FIFO_DEPTH => FIFO_DEPTH 
        )
        Port map(
            reset => aresetn,
            clk => clk,
            
            din => fifo_in_data,
            dout => fifo_out_data,
            
            rd_en => read,
            wr_en => write,
                    
            full => full_mem,
            empty => empty_mem
        );
    
    header_value <= std_logic_vector(to_unsigned(16#FF#,fifo_width-1));
    footer_value <= std_logic_vector(to_unsigned(16#F1#,fifo_width-1));
    
    -- Gestione della lettura dalla FIFO
    read <= m_axis_tready and not empty_mem; --inizio a svuotare la fifo e passare i dati al modulo successivo solo se quetso è pronto e se la mia memoria non è vuota
    
    s_axis_tready <= not full_mem;-- sono pronto a ricevere dati dall UARt solo quando la fifo non è piena
    
    m_axis_tlast <= fifo_out_data(8);
    m_axis_tdata <= fifo_out_data(7 downto 0);
    fifo_in_data(fifo_width-1)<= '1' when s_axis_tdata = footer_value else '0';
    
    --FSM MEALY processes
    sync_logic: process(aresetn,clk)
        begin
            if aresetn = '0' then
                current_state <= state0; --stato iniziale 
            elsif rising_edge(clk) then 
                current_state <= next_state; 
                fifo_in_data(fifo_width-2 downto 0) <= fifo_data_mem(FIFO_WIDTH-2 downto 0);  -- scrivo nella fifo ciò che ho letto al clk precedente                       
                m_axis_tvalid <= read;
            end if;
    end process sync_logic;
    
    transition_logic: process(s_axis_tvalid,s_axis_tdata,current_state)
        begin
            case current_state is
                when state0 => --stato di attesa di un header
                    if s_axis_tvalid = '1' then
                        if s_axis_tdata = header_value then
                            next_state <= stateff;                         
                        end if; 
                    else 
                        next_state <= state0;                   
                    end if;
                    fifo_data_mem <= (others=>'0');-- pongo a zero per controllo 
                    write <= '0';
                    tlast_flag  <= '0';
                    
                when stateff => --stato in cui ho letto un header e gestisco se andare in S1 o tornare in attesa se ricevo un footer valido
                    if s_axis_tvalid = '1' then -- passo allo stato S1 e alzo write solo se il dato in ingresso è valid
                        if s_axis_tdata = footer_value then
                            next_state <= state0; --torno allo stato di attesa così non scrivo niente nella fifo
                            fifo_data_mem <= (others=>'0');-- pongo a zero per controllo                         
                        else 
                            next_state <= state1;
                            fifo_data_mem <= '0' & s_axis_tdata;-- passo zero(tlast) e il dato letto che scrivo nella fifo al prossimo colpo di clk
                        end if; 
                    else 
                        next_state <= stateff; 
                        fifo_data_mem <= (others=>'0');   
                    end if; 
                    write <= '0'; 
                    tlast_flag  <= '0';
                       
                when state1 => 
                    if s_axis_tvalid = '1' then
                        if s_axis_tdata = footer_value then
                            next_state <= state0; 
                            tlast_flag  <= '1';                                                             
                        else 
                            next_state <= state1;
                            fifo_data_mem <= '0' & s_axis_tdata;
                            tlast_flag  <= '0';
                        end if; 
                        write <= '1';
                    else 
                        next_state <= state1; 
                        write <= '0'; 
                        tlast_flag  <= '0';
                    end if;                                   
            end case;
    end process transition_logic;

end architecture;