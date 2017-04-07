library IEEE;
use IEEE.std_logic_1164.all;  -- defines std_logic types
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;

-- Copyright (C) 2007, Peter C. Wallace, Mesa Electronics
-- http://www.mesanet.com
--
-- Ported to Z-Turn JD2CB board: Copyright (C) 2016, Devin Hughes, JD Squared
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
--   * Redistributions of source code must retain the above copyright
--     notice, this list of conditions and the following disclaimer.
--
--   * Redistributions in binary form must reproduce the above
--     copyright notice, this list of conditions and the following
--     disclaimer in the documentation and/or other materials
--     provided with the distribution.
--
--   * Neither the name of Mesa Electronics nor the names of its
--     contributors may be used to endorse or promote products
--     derived from this software without specific prior written
--     permission.
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

use work.IDROMConst.all;

package PIN_ZJD2CB_R2_72 is
	constant ModuleID : ModuleIDType :=(
    (HM2DPLLTag,		x"00",	ClockLowTag,    	x"01",	HM2DPLLBaseRateAddr&PadT,	    HM2DPLLNumRegs,		    x"00",	HM2DPLLMPBitMask),
		(WatchDogTag,	x"00",	ClockLowTag,	    x"01",	WatchDogTimeAddr&PadT,			WatchDogNumRegs,		x"00",	WatchDogMPBitMask),
		(IOPortTag,		x"00",	ClockLowTag,	    x"03",	PortAddr&PadT,				    IOPortNumRegs,			x"00",	IOPortMPBitMask),
		(StepGenTag,	x"02",	ClockLowTag,	    x"05",	StepGenRateAddr&PadT,		    StepGenNumRegs,		   	x"00",	StepGenMPBitMask),
		(FWIDTag,     	x"00",  ClockLowTag,   		x"01",  FWIDAddr&PadT,        			FWIDNumRegs,            x"00",  FWIDMPBitMask),
		(NullTag,		x"00",	NullTag,			x"00",	NullAddr&PadT,					x"00",					x"00",	x"00000000"),
		(NullTag,		x"00",	NullTag,			x"00",	NullAddr&PadT,					x"00",					x"00",	x"00000000"),
		(NullTag,		x"00",	NullTag,			x"00",	NullAddr&PadT,					x"00",					x"00",	x"00000000"),
		(NullTag,		x"00",	NullTag,			x"00",	NullAddr&PadT,					x"00",					x"00",	x"00000000"),
		(NullTag,		x"00",	NullTag,			x"00",	NullAddr&PadT,					x"00",					x"00",	x"00000000"),
		(NullTag,		x"00",	NullTag,			x"00",	NullAddr&PadT,					x"00",					x"00",	x"00000000"),
		(NullTag,		x"00",	NullTag,			x"00",	NullAddr&PadT,					x"00",					x"00",	x"00000000"),
		(NullTag,		x"00",	NullTag,			x"00",	NullAddr&PadT,					x"00",					x"00",	x"00000000"),
		(NullTag,		x"00",	NullTag,			x"00",	NullAddr&PadT,					x"00",					x"00",	x"00000000"),
		(NullTag,		x"00",	NullTag,			x"00",	NullAddr&PadT,					x"00",					x"00",	x"00000000"),
		(NullTag,		x"00",	NullTag,			x"00",	NullAddr&PadT,					x"00",					x"00",	x"00000000"),
		(NullTag,		x"00",	NullTag,			x"00",	NullAddr&PadT,					x"00",					x"00",	x"00000000"),
		(NullTag,		x"00",	NullTag,			x"00",	NullAddr&PadT,					x"00",					x"00",	x"00000000"),
		(NullTag,		x"00",	NullTag,			x"00",	NullAddr&PadT,					x"00",					x"00",	x"00000000"),
		(NullTag,		x"00",	NullTag,			x"00",	NullAddr&PadT,					x"00",					x"00",	x"00000000"),
		(NullTag,		x"00",	NullTag,			x"00",	NullAddr&PadT,					x"00",					x"00",	x"00000000"),
		(NullTag,		x"00",	NullTag,			x"00",	NullAddr&PadT,					x"00",					x"00",	x"00000000"),
		(NullTag,		x"00",	NullTag,			x"00",	NullAddr&PadT,					x"00",					x"00",	x"00000000"),
		(NullTag,		x"00",	NullTag,			x"00",	NullAddr&PadT,					x"00",					x"00",	x"00000000"),
		(NullTag,		x"00",	NullTag,			x"00",	NullAddr&PadT,					x"00",					x"00",	x"00000000"),
		(NullTag,		x"00",	NullTag,			x"00",	NullAddr&PadT,					x"00",					x"00",	x"00000000"),
		(NullTag,		x"00",	NullTag,			x"00",	NullAddr&PadT,					x"00",					x"00",	x"00000000"),
		(NullTag,		x"00",	NullTag,			x"00",	NullAddr&PadT,					x"00",					x"00",	x"00000000"),
		(NullTag,		x"00",	NullTag,			x"00",	NullAddr&PadT,					x"00",					x"00",	x"00000000"),
		(NullTag,		x"00",	NullTag,			x"00",	NullAddr&PadT,					x"00",					x"00",	x"00000000"),
		(NullTag,		x"00",	NullTag,			x"00",	NullAddr&PadT,					x"00",					x"00",	x"00000000"),
		(NullTag,		x"00",	NullTag,			x"00",	NullAddr&PadT,					x"00",					x"00",	x"00000000")
		);


	constant PinDesc : PinDescType :=(
    -- 	Base func  sec unit sec func 	 sec pin			    -- external Conn
	    IOPortTag & x"00" & NullTag & NullPin,			        -- I/O 00   PIN 2-31 M1-FB GPIO
	    IOPortTag & x"00" & StepGenTag & StepGenStepPin,	    -- I/O 01   PIN 2-33 M1-STP
	    IOPortTag & x"00" & StepGenTag & StepGenDirPin,       	-- I/O 02   PIN 2-35 M1-DIR
	    IOPortTag & x"00" & NullTag & NullPin,			        -- I/O 03   PIN 2-37 M1-EN GPIO

        IOPortTag & x"00" & NullTag & NullPin,			        -- I/O 04   PIN 2-27 M2-FB GPIO
        IOPortTag & x"01" & StepGenTag & StepGenStepPin,        -- I/O 05   PIN 2-41 M2-STP
        IOPortTag & x"01" & StepGenTag & StepGenDirPin,         -- I/O 06   PIN 2-43 M2-DIR
        IOPortTag & x"00" & NullTag & NullPin,                  -- I/O 07   PIN 2-45 M2-EN GPIO

        IOPortTag & x"00" & NullTag & NullPin,                  -- I/O 08   PIN 2-25 M3-FB GPIO
        IOPortTag & x"02" & StepGenTag & StepGenStepPin,        -- I/O 09   PIN 2-47 M3-STP
        IOPortTag & x"02" & StepGenTag & StepGenDirPin,         -- I/O 10   PIN 2-53 M3-DIR
        IOPortTag & x"00" & NullTag & NullPin,                  -- I/O 11   PIN 2-55 M3-EN GPIO

        IOPortTag & x"00" & NullTag & NullPin,                  -- I/O 12   PIN 2-23 M4-FB GPIO
        IOPortTag & x"03" & StepGenTag & StepGenStepPin,        -- I/O 13   PIN 2-57 M4-STP
        IOPortTag & x"03" & StepGenTag & StepGenDirPin,         -- I/O 14   PIN 2-61 M4-DIR
        IOPortTag & x"00" & NullTag & NullPin,                  -- I/O 15   PIN 2-63 M4-EN GPIO

        IOPortTag & x"00" & NullTag & NullPin,                  -- I/O 16   PIN 2-21 M5-FB GPIO
        IOPortTag & x"04" & StepGenTag & StepGenStepPin,        -- I/O 17   PIN 2-65 M5-STP
        IOPortTag & x"04" & StepGenTag & StepGenDirPin,         -- I/O 18   PIN 2-67 M5-DIR
        IOPortTag & x"00" & NullTag & NullPin,                  -- I/O 19   PIN 2-71 M5-EN GPIO

        IOPortTag & x"00" & NullTag & NullPin,                  -- I/O 20   PIN 1-30 MOTOR-PWR GPIO
        IOPortTag & x"00" & NullTag & NullPin,                  -- I/O 21   PIN 1-63 TORCH-START GPIO
        IOPortTag & x"00" & NullTag & NullPin,                  -- I/O 22   PIN 1-37 Z-PROBE-MC GPIO
        IOPortTag & x"00" & NullTag & NullPin,                  -- I/O 23   PIN 1-25 LASER GPIO

        IOPortTag & x"00" & NullTag & NullPin,                  -- I/O 24   PIN 1-35 OUT1 GPIO
        IOPortTag & x"00" & NullTag & NullPin,                  -- I/O 25   PIN 1-31 OUT2 GPIO
        IOPortTag & x"00" & NullTag & NullPin,                  -- I/O 26   PIN 1-32 OUT3 GPIO
        IOPortTag & x"00" & NullTag & NullPin,                  -- I/O 27   PIN 1-62 OUT4 GPIO
        IOPortTag & x"00" & NullTag & NullPin,                  -- I/O 28   PIN 1-60 OUT5 GPIO
        IOPortTag & x"00" & NullTag & NullPin,                  -- I/O 29   PIN 1-58 OUT6 GPIO
        IOPortTag & x"00" & NullTag & NullPin,                  -- I/O 30   PIN 1-56 OUT7 GPIO
        IOPortTag & x"00" & NullTag & NullPin,                  -- I/O 31   PIN 1-51 OUT8 GPIO
        IOPortTag & x"00" & NullTag & NullPin,                  -- I/O 32   PIN 1-49 OUT9 GPIO
        IOPortTag & x"00" & NullTag & NullPin,                  -- I/O 33   PIN 1-47 OUT10 GPIO
        IOPortTag & x"00" & NullTag & NullPin,                  -- I/O 34   PIN 1-45 OUT11 GPIO

        IOPortTag & x"00" & NullTag & NullPin,                  -- I/O 35   PIN 2-60 ESTOP GPIO
		IOPortTag & x"00" & NullTag & NullPin,                  -- I/O 36   PIN 2-68 TORCH-BRK GPIO
		IOPortTag & x"00" & NullTag & NullPin,                  -- I/O 37   PIN 1-65 ARC-OK GPIO
		IOPortTag & x"00" & NullTag & NullPin,                  -- I/O 38   PIN 2-74 Z-PROBE GPIO
		IOPortTag & x"00" & NullTag & NullPin,                  -- I/O 39   PIN 2-80 Z-PROBE-SW GPIO
		IOPortTag & x"00" & NullTag & NullPin,                  -- I/O 40   PIN 2-54 TOOL-PROBE GPIO

		IOPortTag & x"00" & NullTag & NullPin,                  -- I/O 41   PIN 2-24 LIM1 GPIO
		IOPortTag & x"00" & NullTag & NullPin,                  -- I/O 42   PIN 2-26 LIM2 GPIO
		IOPortTag & x"00" & NullTag & NullPin,                  -- I/O 43   PIN 2-30 LIM3 GPIO
		IOPortTag & x"00" & NullTag & NullPin,                  -- I/O 44   PIN 2-38 LIM4 GPIO
		IOPortTag & x"00" & NullTag & NullPin,                  -- I/O 45   PIN 2-44 AUXI1 GPIO
		IOPortTag & x"00" & NullTag & NullPin,                  -- I/O 46   PIN 2-50 AUXI2 GPIO

		IOPortTag & x"00" & NullTag & NullPin,                  -- I/O 47   PIN 2-64 TP2 GPIO
		IOPortTag & x"00" & NullTag & NullPin,                  -- I/O 48   PIN 2-66 TP3 GPIO

		IOPortTag & x"00" & NullTag & NullPin,                  -- I/O 49   PIN 1-28 MOTOR-PWR-FLT GPIO
		IOPortTag & x"00" & NullTag & NullPin,                  -- I/O 50   PIN 1-42 Z-RPOBE-MC-FLT GPIO
		IOPortTag & x"00" & NullTag & NullPin,                  -- I/O 51   PIN 1-27 LASER-FLT GPIO

		IOPortTag & x"00" & NullTag & NullPin,                  -- I/O 52   PIN 1-38 OUT1-FLT GPIO
		IOPortTag & x"00" & NullTag & NullPin,                  -- I/O 53   PIN 1-34 OUT2-FLT GPIO
		IOPortTag & x"00" & NullTag & NullPin,                  -- I/O 54   PIN 2-76 OUT3-FLT GPIO
		IOPortTag & x"00" & NullTag & NullPin,                  -- I/O 55   PIN 2-78 OUT4-FLT GPIO

		IOPortTag & x"00" & NullTag & NullPin,                  -- I/O 56   PIN 2-73 OUT5-FLT GPIO
		IOPortTag & x"00" & NullTag & NullPin,                  -- I/O 57   PIN 2-75 OUT6-FLT GPIO
		IOPortTag & x"00" & NullTag & NullPin,                  -- I/O 58   PIN 2-77 OUT7-FLT GPIO
		IOPortTag & x"00" & NullTag & NullPin,                  -- I/O 59   PIN 1-54 OUT8-FLT GPIO

		IOPortTag & x"00" & NullTag & NullPin,                  -- I/O 60   PIN 1-52 OUT9-FLT GPIO
		IOPortTag & x"00" & NullTag & NullPin,                  -- I/O 61   PIN 1-48 OUT10-FLT GPIO
		IOPortTag & x"00" & NullTag & NullPin,                  -- I/O 62   PIN 1-44 OUT11-FLT GPIO
		
		IOPortTag & x"00" & NullTag & NullPin,                  -- I/O 63   PIN 2-36 LOGIC(TBREAK-OVER) GPIO
		IOPortTag & x"00" & NullTag & NullPin,                  -- I/O 64   PIN 2-34 LOGIC(E-STOP) GPIO
		IOPortTag & x"00" & NullTag & NullPin,                  -- I/O 65   PIN 2-40 LOGIC(M2FB & M3FB) GPIO
		IOPortTag & x"00" & NullTag & NullPin,                  -- I/O 66   PIN 2-46 LOGIC(PROBES) GPIO
		IOPortTag & x"00" & NullTag & NullPin,                  -- I/O 67   PIN 2-48 LOGIC(EPROBE-OVER) GPIO
		IOPortTag & x"00" & NullTag & NullPin,                  -- I/O 68   PIN 2-28 LOGIC(EXTRA1) GPIO
		IOPortTag & x"00" & NullTag & NullPin,                  -- I/O 69   PIN 2-56 LOGIC(EXTRA2) GPIO
 		IOPortTag & x"00" & NullTag & NullPin,                  -- I/O 70   PIN 2-56 LOGIC(EXTRA3) GPIO
		IOPortTag & x"00" & NullTag & NullPin,                  -- I/O 71   PIN 2-56 LOGIC(EXTRA4) GPIO

		emptypin,emptypin,emptypin,emptypin,emptypin,emptypin,emptypin,emptypin,
		emptypin,emptypin,emptypin,emptypin,emptypin,emptypin,emptypin,emptypin,
		emptypin,emptypin,emptypin,emptypin,emptypin,emptypin,emptypin,emptypin,
		emptypin,emptypin,emptypin,emptypin,emptypin,emptypin,emptypin,emptypin,
		emptypin,emptypin,emptypin,emptypin,emptypin,emptypin,emptypin,emptypin,
		emptypin,emptypin,emptypin,emptypin,emptypin,emptypin,emptypin,emptypin,
		emptypin,emptypin,emptypin,emptypin,emptypin,emptypin,emptypin,emptypin,
		emptypin,emptypin,emptypin,emptypin,emptypin,emptypin,emptypin,emptypin,
		emptypin,emptypin,emptypin,emptypin,emptypin,emptypin,emptypin,emptypin);

end package PIN_ZJD2CB_R2_72;
