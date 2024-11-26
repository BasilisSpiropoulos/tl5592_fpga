----------------------------------------------------------------------------------
-- Engineer: Spiropoulos Vasilis
-- 
-- Create Date: 11.05.2024 12:09:07
-- Design Name: bus_breakout
-- Module Name: bus_breakout - Behavioral
-- Project Name: Instrument Panel Cluster Controller
-- Target Devices: Xilinx XC7A35T-1CPG236C
-- Tool Versions: Xilinx Vivado 2019.1
-- Description: It outputs the least significant bit (LSB) of "sig_in" as the
-- single-bit signal "sig_out". Additionally, it outputs the remaining 15 bits
-- of sig_in, starting from the second bit to the most significant bit (MSB),
-- as the signal sig_out_bus. This operation effectively shifts the original
-- input bus, omitting the first bit and making it available separately as sig_out.
----------------------------------------------------------------------------------


library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity bus_breakout is
    Port ( sig_in : in STD_LOGIC_VECTOR (15 downto 0);
           sig_out : out STD_LOGIC;
           sig_out_bus : out STD_LOGIC_VECTOR (14 downto 0));
end bus_breakout;

architecture Behavioral of bus_breakout is

begin

sig_out <= sig_in(0);

sig_out_bus(14 downto 0) <= sig_in(15 downto 1);

end Behavioral;
