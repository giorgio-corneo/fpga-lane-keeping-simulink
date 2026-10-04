-- FIFO con uscita classica

library IEEE;
	use IEEE.STD_LOGIC_1164.ALL;
	use IEEE.NUMERIC_STD.ALL;

entity FIFO is
	Generic(
		FIFO_WIDTH : integer := 9;
		FIFO_DEPTH : integer := 15
	);
	Port(
		reset 	: in std_logic;
		clk 	: in std_logic;

		din 	: in std_logic_vector(FIFO_WIDTH-1 DOWNTO 0);
		dout 	: out std_logic_vector(FIFO_WIDTH-1 DOWNTO 0);

		rd_en 	: in std_logic;
		wr_en 	: in std_logic;

		full 	: out std_logic;
		empty 	: out std_logic
	);
end FIFO;

architecture Behavioral of FIFO is
	type memArray_type is array (0 TO FIFO_DEPTH-1) of std_logic_vector(din'RANGE);
	signal memArray : memArray_type;

	signal wr_ptr, rd_ptr : INTEGER range 0 to FIFO_DEPTH-1 := 0;
	signal count : INTEGER range 0 to FIFO_DEPTH := 0;

	signal full_int, empty_int : std_logic;
begin

	full_int 	<= '1' when count = FIFO_DEPTH else '0';
	empty_int 	<= '1' when count = 0 else '0';

	empty 	<= empty_int;
	full 	<= full_int;


	process(clk, reset)
	begin
		if reset = '0' then --reset attivo basso nel depack
			count 	<= 0;
			wr_ptr 	<= 0;
			rd_ptr 	<= 0;
		elsif rising_edge(clk) then

			-- COUNT SECTION
			if wr_en = '1' and rd_en = '0' then
				if full_int = '0' then
					count <= count + 1;
				end if;
			elsif wr_en = '0' and rd_en = '1' then
				if empty_int = '0' then
					count <= count -1;
				end if;
			elsif wr_en = '1' and rd_en = '1' then
				if empty_int = '1' then
					count <= count + 1;
				elsif full_int = '1' then
					count <= count - 1;
				end if;
			end if;

			-- Write pointer
			if wr_en = '1' and full_int = '0' then
				if wr_ptr = FIFO_DEPTH-1 then
					wr_ptr <= 0;
				else
					wr_ptr <= wr_ptr + 1;
				end if;
			end if;

			-- Read pointer
			if rd_en = '1' and empty_int = '0' then
				if rd_ptr = FIFO_DEPTH-1 then
					rd_ptr <= 0;
				else
					rd_ptr <= rd_ptr + 1;
				end if;
			end if;

			-- Din
			if wr_en = '1' and full_int = '0' then
				memArray(wr_ptr)	<= din;
			end if;

			-- Dout
			if rd_en = '1' and empty_int = '0' then
				dout <= memArray(rd_ptr);
			end if;

		end if;
	end process;
end Behavioral;
