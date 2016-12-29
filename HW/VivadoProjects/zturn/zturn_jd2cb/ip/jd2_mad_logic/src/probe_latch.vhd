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

entity probe_latch is
	port (
	    clk : in std_logic;
	    arm_latch : in std_logic;
	    sw_probe_in, el_probe_in : in std_logic; -- Raw Probe Inputs
		el_probe_over : in std_logic;
        el_probe_out, sw_probe_out, probe_out : out std_logic -- Logic Applied Probe Inputs
	);
end probe_latch;

architecture arch_imp of probe_latch is
	type sm_type is (start, idle, armed, wait_for_clear);
	signal current_state, next_state : sm_type := start;
    signal sw_probe_deb, el_probe_deb : std_logic;
    signal sw_probe_latch, el_probe_latch : std_logic;
    signal latched : std_logic;
begin    
    probe_out <= sw_probe_latch OR el_probe_latch;  
    el_probe_out <= (el_probe_latch AND (NOT(el_probe_over)));
    sw_probe_out <= sw_probe_latch;
    
    calc_latch : process(current_state, el_probe_latch, el_probe_in, sw_probe_latch, sw_probe_in)
	begin
		-- default off
		latched <= '0';
		if(current_state = armed) then
			if (el_probe_latch = '0' and sw_probe_latch = '0') then                
				-- el probe has preference
	            if (el_probe_in = '1' or sw_probe_in = '1') then
	                latched <= '1';
				end if;
			elsif(el_probe_latch = '1') then
				if(el_probe_in =  '0') then
					latched <= '1';
				end if;
			elsif(sw_probe_latch = '1') then
				if(sw_probe_in = '0') then
					latched <= '1';
				end if;
			end if;
        end if;
	end process;

    latch_changed : process(clk, current_state, el_probe_latch, el_probe_in, sw_probe_latch, sw_probe_in)
    begin
        if (rising_edge(clk)) then
            if(current_state = start) then
                el_probe_latch <= el_probe_in;
                sw_probe_latch <= sw_probe_in;
            elsif (current_state = armed) then 
				if (el_probe_latch = '0' and sw_probe_latch = '0') then                
					-- el probe has preference
		            if (el_probe_in = '1') then
		                el_probe_latch <= el_probe_in;
		            elsif (sw_probe_in = '1') then
		                sw_probe_latch <= sw_probe_in;
		            end if;
				elsif(el_probe_latch = '1') then
					if(el_probe_in = '0') then
						el_probe_latch <= el_probe_in;
					end if;
				elsif(sw_probe_latch = '1') then
					if(sw_probe_in = '0') then
						sw_probe_latch <= sw_probe_in;
					end if;
				end if;
            end if;
        end if;
    end process;
    
    update_state : process(clk, next_state)
    begin
        if(rising_edge(clk)) then
            current_state <= next_state;
        end if;
    end process update_state;

    calc_state : process(current_state, arm_latch, latched)
    begin
        -- default to hold state
        next_state <= current_state;
        case current_state is
            when start =>
                -- read the default state of the pins
                next_state <= idle;
            when idle =>
                if(arm_latch = '1') then
                    -- Arm the probe
                    next_state <= armed;
                end if;
            when armed =>
                if(latched = '1') then
                    next_state <= wait_for_clear;
                elsif(arm_latch = '0') then
                    next_state <= idle;
                end if;
            when wait_for_clear =>
                if(arm_latch = '0') then
                    next_state <= idle;
                end if;  
            when others => next_state <= idle;
        end case;
    end process calc_state;
end arch_imp;
