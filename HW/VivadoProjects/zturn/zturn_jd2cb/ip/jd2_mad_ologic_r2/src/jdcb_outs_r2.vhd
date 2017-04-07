-- Copyright (C) 2017, Devin Hughes, JD Squared
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

entity jdcb_outs_r2 is
	generic (
		WIDTH	: integer	:= 72
	);
	port (
	    OUTS : in std_logic_vector(WIDTH - 1 downto 0); -- Raw Outputs
        LOG_OUTS : out std_logic_vector(WIDTH - 1 downto 0) -- Logic applied outputs
	);
end jdcb_outs_r2;

architecture arch_imp of jdcb_outs_r2 is

begin    
	-- 
	-- E-Probe disabled when overridden or when torch is active
	--
	LOG_OUTS(22) <= OUTS(22) AND NOT(OUTS(21) OR OUTS(68)) AND NOT(OUTS(48));

	--
	-- Torch disabled when dry-run active
	--	
	LOG_OUTS(21) <= (OUTS(21) OR OUTS(68)) AND NOT(OUTS(47));

	--
	-- Aux Outputs disabled when dry-run active
	--
    gen_aux : for i in 0 to 10 generate
		LOG_OUTS(i+24) <= OUTS(i+24) AND NOT(OUTS(47));        
    end generate gen_aux;

	--
	-- All others pass through
	--
	LOG_OUTS(71 downto 35) <= OUTS(71 downto 35);
	LOG_OUTS(23) <= OUTS(23);	
	LOG_OUTS(20 downto 0) <= OUTS(20 downto 0);
end arch_imp;
