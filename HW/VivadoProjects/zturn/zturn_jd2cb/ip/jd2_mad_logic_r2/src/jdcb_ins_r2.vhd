-- Copyright (C) 2016, Devin Hughes, JD Squared
-- All rights reserved
--
-- This program is is licensed under a disjunctive dual license giving you
-- the choice of one of the two following sets of free software/open source
-- licensing terms:
--
--    * GNU General Public License (GPL), version 2.0 or later
--    * 3-clause BSD License
--
--
-- The GNU GPL License:
--
--     This program is free software; you can redistribute it and/or modify
--     it under the terms of the GNU General Public License as published by
--     the Free Software Foundation; either version 2 of the License, or
--     (at your option) any later version.
--
--     This program is distributed in the hope that it will be useful,
--     but WITHOUT ANY WARRANTY; without even the implied warranty of
--     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
--     GNU General Public License for more details.
--
--     You should have received a copy of the GNU General Public License
--     along with this program; if not, write to the Free Software
--     Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
--
--
-- The 3-clause BSD License:
--
--     Redistribution and use in source and binary forms, with or without
--     modification, are permitted provided that the following conditions
--     are met:
--
--         * Redistributions of source code must retain the above copyright
--           notice, this list of conditions and the following disclaimer.
--
--         * Redistributions in binary form must reproduce the above
--           copyright notice, this list of conditions and the following
--           disclaimer in the documentation and/or other materials
--           provided with the distribution.
--
--         * Neither the name of the copyright holder nor the names of its
--           contributors may be used to endorse or promote products
--           derived from this software without specific prior written
--           permission.
--
--
-- Disclaimer:
--
--     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
--     "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
--     LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
--     FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
--     COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
--     INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
--     BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
--     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
--     CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
--     LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
--     ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
--     POSSIBILITY OF SUCH DAMAGE.
--

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity jdcb_ins_r2 is
	generic (
		WIDTH	: integer	:= 72;
		NUM_DEB_STAGES : integer := 10 -- Number of stages of debounce before output is latched
	);
	port (
	    clk : in std_logic; -- Expect 100MHz clock here
	    INS : in std_logic_vector(WIDTH - 1 downto 0); -- Raw Inputs
        LOG_INS : out std_logic_vector(WIDTH - 1 downto 0) -- Logic applied inputs
	);
end jdcb_ins_r2;

architecture arch_imp of jdcb_ins_r2 is
    signal deb_clk : std_logic := '0';
    signal half_period : unsigned(15 downto 0) := x"30D4"; -- 4 kHz debounce clock
    signal timer_cnt: unsigned(15 downto 0) := (others => '0');

    signal aux_deb : std_logic_vector(1 downto 0);
    signal lim_deb : std_logic_vector(3 downto 0);
	signal probe_deb : std_logic_vector(2 downto 0);
	signal eprobe: std_logic := '0';

	signal arcok: std_logic := '0';

    signal motor_fb_valid : std_logic;    -- When motors are turned off, ignore amp fb
    signal mtr_pwr_del : std_logic;
begin    
	--
	-- Motor Logic 
	--	

	-- Feedbacks are filtered and inverted
    motor_fb_valid <= (mtr_pwr_del AND INS(20)); -- Don't delay when powering off motors
    LOG_INS(0) <= motor_fb_valid AND (NOT(INS(0))); -- M1
	LOG_INS(4) <= motor_fb_valid AND (NOT(INS(4))); -- M2
	LOG_INS(8) <= motor_fb_valid AND (NOT(INS(8))); -- M3
    LOG_INS(12) <= motor_fb_valid AND (NOT(INS(12))); -- M4
    LOG_INS(16) <= motor_fb_valid AND (NOT(INS(16))); -- M5
	LOG_INS(65) <= motor_fb_valid AND (NOT(INS(4)) OR NOT(INS(8)));

    motor_pwr_del : entity work.sig_Delay
        generic map(
		    CLOCK_RATE => 100000000,
		    DELAY_TIME_INV => 5,    -- 200 ms delay
            NUM_TIMER_BITS => 32
        )
        port map (
            clk => deb_clk,
            input => INS(20),
            output => mtr_pwr_del
        );

	-- Motor Outputs are passed through
	LOG_INS(3 downto 1) <= INS(3 downto 1);    
	LOG_INS(7 downto 5) <= INS(7 downto 5);
    LOG_INS(11 downto 9) <= INS(11 downto 9);
    LOG_INS(15 downto 13) <= INS(15 downto 13);
    LOG_INS(19 downto 17) <= INS(19 downto 17);

	--
	-- Control Outputs are passed through 
	--	
	LOG_INS(34 downto 20) <= INS(34 downto 20);

	--
	-- E-Stop and Torch Break changed to positive logic
	--
	LOG_INS(36 downto 35) <= NOT(INS(36 downto 35));
	LOG_INS(64) <= NOT(INS(35)) OR (NOT(INS(36)) AND NOT(INS(63)));  -- Logical E-Stop
	LOG_INS(63) <= INS(63);

	--
	-- ARC Okay input is filtered.
	-- When dry run is active, or if the machine
	-- does not have an arc okay indicator, we force
	-- arc okay true.
	--
	LOG_INS(37) <= arcok OR INS(47) or NOT(INS(69));
    arcok_deb: entity work.inp_deb
        generic map (NUM_STAGES => NUM_DEB_STAGES)
        port map (
            clk => deb_clk,
            input => INS(37),
            output => arcok
        );

	--
	-- Probes are filtered. 
	-- Switch probes need to be flipped to positive logic.
	-- Electrical probe is controlled by the override control output.
	--
	LOG_INS(66) <= (eprobe OR NOT(probe_deb(1)) OR NOT(probe_deb(2))); -- Combined probe signal
	eprobe <= (probe_deb(0) AND (NOT(INS(67)))); -- block E-probe when overridden	
	LOG_INS(38) <= eprobe;
	LOG_INS(40 downto 39) <= NOT(probe_deb(2 downto 1)); -- Z Switch(1) and Tool probe (2) 
	LOG_INS(67) <= INS(67);	-- E-probe override signal	
	gen_probe : for i in 0 to 2 generate
        probex: entity work.inp_deb
        generic map (NUM_STAGES => NUM_DEB_STAGES)
        port map (
            clk => deb_clk,
            input => INS(i + 38),
            output => probe_deb(i)
        );
    end generate gen_probe;

	-- 
	-- Limit Switches are filtered and changed to positive logic
	--
    LOG_INS(44 downto 41) <= NOT(lim_deb(3 downto 0));
    gen_lim : for i in 0 to 3 generate
        limx: entity work.inp_deb
        generic map (NUM_STAGES => NUM_DEB_STAGES)
        port map (
            clk => deb_clk,
            input => INS(i + 41),
            output => lim_deb(i)
        );
    end generate gen_lim;

	--
	-- Aux inputs filtered and changed to positive logic
	--
	LOG_INS(46 downto 45) <= NOT(aux_deb(1 downto 0));
    gen_aux : for i in 0 to 1 generate
        auxx: entity work.inp_deb
        generic map (NUM_STAGES => NUM_DEB_STAGES)
        port map (
            clk => deb_clk,
            input => INS(i + 45),
            output => aux_deb(i)
        );
    end generate gen_aux;

	--
	-- Fault inputs are changed to positive logic
	--
	LOG_INS(62 downto 49) <= NOT(INS(62 downto 49));

	--
	-- TP3 is empty
	--
	LOG_INS(48) <= INS(48);

	--
	-- Dry run passes through
	--
	LOG_INS(47) <= INS(47);

	-- Extra control signals 71 downto 68
	LOG_INS(70 downto 68) <= INS(70 downto 68);
	LOG_INS(71) <= INS(71) AND INS(66) AND INS(65) AND INS(64);

	-- 
	-- Debounce clock generation
	--
    clkdiv : process (clk, timer_cnt)
    begin
        if rising_edge(clk) then
            if (timer_cnt = x"0001") then
                timer_cnt <= half_period;
                deb_clk <= NOT(deb_clk);
            else
                timer_cnt <= timer_cnt - 1;
            end if;
        end if; -- (clk)
    end process;
end arch_imp;
