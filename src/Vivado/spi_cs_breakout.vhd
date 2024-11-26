----------------------------------------------------------------------------------
-- Engineer: Spiropoulos Vasilis 
-- 
-- Create Date: 26.05.2024 22:50:09
-- Design Name: spi_triple_inhibit
-- Module Name: spi_triple_inhibit - Behavioral
-- Project Name: Instrument Panel Cluster Controller
-- Target Devices: Xilinx XC7A35T-1CPG236C
-- Tool Versions: Xilinx Vivado 2019.1
-- Description: This module decodes the 3-bit input bus "sig_in" into
-- three individual output signals: "sig_out_0", "sig_out_1", and "sig_out_2".
-- Each output signal corresponds to a specific bit from the input bus
-- and reflects its value.
----------------------------------------------------------------------------------


library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity spi_cs_breakout is
    Port ( sig_in : in STD_LOGIC_VECTOR (2 downto 0);
           sig_out_0 : out STD_LOGIC;
           sig_out_1 : out STD_LOGIC;
           sig_out_2 : out STD_LOGIC);
end spi_cs_breakout;

architecture Behavioral of spi_cs_breakout is

begin

sig_out_0 <= sig_in(0);
sig_out_1 <= sig_in(1);
sig_out_2 <= sig_in(2);

end Behavioral;
