-- Ins and Outs overlap at the moment, i.e., there
-- are only 32 total io, and the ins will reflect
-- the state of the outs as well as inputs.
-- Expects 100MHz clock
-- Serial communication is half duplex with this component
-- the master.

-- Control reg bit 16 is component ready signal
-- Control reg bit 0 is software reset
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity btint_controller is
    generic(
        PP_BUF_ADDR_WIDTH : natural := 6
    );
	port
	(
	    rst_n   : in std_logic;
      	cnt_rst_n : out std_logic;
        clk     : in std_logic;
        pp_buf_lock : in std_logic; -- Identifies which buffer is being written to by rx component
        pp_addr : out std_logic_vector(PP_BUF_ADDR_WIDTH - 1 downto 0); -- PP Buffer address bus
        pp_data1 : in std_logic_vector(7 downto 0); -- PP Buffer data bus 1
        pp_data2 : in std_logic_vector(7 downto 0); -- PP Buffer data bus 2
		pkt_tx_data_len : out std_logic_vector(1 downto 0);
        pkt_tx_data : out std_logic_vector(47 downto 0);
        pkt_tx_we : out std_logic;
        pkt_tx_busy : in std_logic;
		uart_tx_busy : in std_logic;
		uart_tx_en : out std_logic;	-- Active high serial transmit enable
		uart_rx_en : out std_logic;	-- Active low serial receive enable
        addr_wr : in std_logic_vector(15 downto 0);
        idata : in std_logic_vector(31 downto 0);
        wr : in std_logic;
        wr_strobe : in std_logic_vector(3 downto 0);
        addr_rd : in std_logic_vector(15 downto 0);
        odata : out std_logic_vector(31 downto 0);
        sync : in std_logic
	);
end entity;

architecture beh of btint_controller is
	type sm_type is (start, send, wait_for_tx, 
					wait_for_resp, parse_resp_cmd,
                   	parse_cal1, parse_data1,
                   	parse_32_1, parse_32_2,
                   	parse_32_3, parse_32_4,
                   	latch_reg,
                   	wait_for_sync);
	type com_sm_type is (send_qry_b, send_qry_m, send_qry_data, send_set_mult, send_set_minvolt);
	signal com_state, com_next_state : com_sm_type := send_qry_b;
	signal current_state, next_state : sm_type := start;
    signal pp_buf_lock_s : std_logic := '0';
    signal pp_buf_change : std_logic := '0';
    signal pp_buf_sel_s : std_logic := '0';
    signal pp_data : std_logic_vector(7 downto 0);
    signal int_rst_n : std_logic := '1';
    signal timed_out : std_logic := '0';
    signal cons_timeouts : unsigned(2 downto 0) := (others => '0');
    signal timeout_tmr : unsigned(31 downto 0) := (others => '0');
    signal reg_add : std_logic_vector(7 downto 0) := (others => '0');
    signal reg_32_tmp : std_logic_vector(31 downto 0) := (others => '0');
    signal pkt_tx_we_s : std_logic;
    signal sync_prev, sync_pulse, clear_sync : std_logic;
	signal txen : std_logic;

    -- Registers
    signal reg_control_upper : std_logic_vector(15 downto 0) := (others => '0');
    signal reg_control_lower : std_logic_vector(15 downto 0) := (others => '0');
    signal reg_cal_b : std_logic_vector(31 downto 0) := (others => '0');
    signal reg_cal_m : std_logic_vector(31 downto 0) := (others => '0');
    signal reg_cal_multiplier : std_logic_vector(31 downto 0) := (others => '0');
    signal reg_cal_minvolt : std_logic_vector(31 downto 0) := (others => '0');
    signal reg_cal_amp : std_logic_vector(31 downto 0) := (others => '0');
    signal reg_cal_airp : std_logic_vector(31 downto 0) := (others => '0');
    signal reg_cal_mode : std_logic_vector(31 downto 0) := (others => '0');
    signal reg_outs : std_logic_vector(31 downto 0) := (others => '0');
    signal reg_ins : std_logic_vector(31 downto 0) := (others => '0');
    signal reg_adc_value : std_logic_vector(31 downto 0) := (others => '0');
    signal reg_err_cnt : std_logic_vector(31 downto 0) := (others => '0');
    signal reg_timerr_cnt : std_logic_vector(31 downto 0) := (others => '0');
    signal byte_index : integer;

	-- Dirty flags for registers to let the main state machine know it needs
	-- to sync the data.
	-- Bit 0 - multiplier register
	-- Bit 1 - minvolt register
	-- Bit 2 - amperage register
	-- Bit 3 - air pressure register
	-- Bit 4 - plasma mode register
	signal reg_is_dirty : std_logic_vector(4 downto 0);
	signal reg_is_dirty_s : std_logic_vector(4 downto 0);
	signal reg_is_dirty_r : std_logic_vector(4 downto 0);

    -- Constants
    constant TIMEOUT_RETRIES : unsigned(2 downto 0) := to_unsigned(2,3);
    constant TIMEOUT_COUNT_VAL_DATA : unsigned(31 downto 0) := to_unsigned(400000, 32);
    constant TIMEOUT_COUNT_VAL_ST : unsigned (31 downto 0) := to_unsigned(1000000, 32);
    constant TIMEOUT_COUNT_VAL_SYNC : unsigned (31 downto 0) := to_unsigned(200000, 32);
    constant PKT_QRY_DATA_CMD : std_logic_vector(7 downto 0) := x"02";
    constant PKT_SETQRY_DATA_CMD : std_logic_vector(7 downto 0) := x"03";
    constant PKT_QRY_CAL_CMD : std_logic_vector(7 downto 0) := x"08";
	constant PKT_SETQRY_CAL_CMD : std_logic_vector(7 downto 0) := x"0C";
    constant MAGIC_NUM_VAL : std_logic_vector(31 downto 0) := x"12345678";
    constant PKT_QRY_DATA_ADC_ADD : std_logic_vector(7 downto 0) := x"00";
    constant PKT_QRY_CAL_B_ADD : std_logic_vector(7 downto 0) := x"01";
    constant PKT_QRY_CAL_M_ADD : std_logic_vector(7 downto 0) := x"02";
    constant PKT_QRY_CAL_MULTIPLIER_ADD : std_logic_vector(7 downto 0) := x"03";
    constant PKT_QRY_CAL_MINVOLT_ADD : std_logic_vector(7 downto 0) := x"04";
    constant PKT_QRY_CAL_AMP_ADD : std_logic_vector(7 downto 0) := x"05";
    constant PKT_QRY_CAL_AIRP_ADD : std_logic_vector(7 downto 0) := x"06";
    constant PKT_QRY_CAL_MODE_ADD : std_logic_vector(7 downto 0) := x"07";
	constant PKT_TX_LEN_2B : std_logic_vector(1 downto 0) := b"00";
	constant PKT_TX_LEN_4B : std_logic_vector(1 downto 0) := b"01";
	constant PKT_TX_LEN_6B : std_logic_vector(1 downto 0) := b"10";
    constant PKT_PAD_2B : std_logic_vector(15 downto 0) := x"0000";

    -- Addresses
    constant REG_MAGIC_ADD : std_logic_vector(15 downto 0) := x"0000";
    constant REG_CONTROL_ADD : std_logic_vector(15 downto 0) := x"0010";
    constant REG_B_ADD : std_logic_vector(15 downto 0) := x"0100";
    constant REG_M_ADD : std_logic_vector(15 downto 0) := x"0104";
    constant REG_MULTIPLIER_ADD : std_logic_vector(15 downto 0) := x"0108";
    constant REG_MINVOLT_ADD : std_logic_vector(15 downto 0) := x"010C";

    constant REG_AMP_ADD : std_logic_vector(15 downto 0) := x"0200";
    constant REG_AIRP_ADD : std_logic_vector(15 downto 0) := x"0204";
    constant REG_MODE_ADD : std_logic_vector(15 downto 0) := x"0208";

    constant REG_OUTS_ADD : std_logic_vector(15 downto 0) := x"0400";
    constant REG_INS_ADD : std_logic_vector(15 downto 0) := x"0500";
    constant REG_ADCVAL_ADD : std_logic_vector(15 downto 0) := x"0600";
    constant REG_ERRCNT_ADD : std_logic_vector(15 downto 0) := x"0700";
    constant REG_TIMERRCNT_ADD : std_logic_vector(15 downto 0) := x"0704";

begin
	-- Reset and Synchronization
  	cnt_rst_n <= int_rst_n;
    int_rst_n <= '0' when (rst_n = '0' or current_state = start) else '1';
    clear_sync <= '1' when ((current_state = send) and (sync_pulse = '1')) else '0';

	-- Transmit packet builder control signals
    pkt_tx_we <= pkt_tx_we_s;
    pkt_tx_we_s <= '1' when ((current_state = send) and (pkt_tx_busy = '0')) else '0';

	-- Half Duplex serial enable control. We only enable TX when we speak
	txen <= '1' when (((current_state = send) or (current_state = wait_for_tx)) and
					   ((pkt_tx_busy = '1') or (uart_tx_busy = '1'))) else '0'; 

	uart_tx_en <= txen;
	uart_rx_en <= txen;	-- RX En is active low

	-- Ping pong buffer data selection
    pp_data <= pp_data2 when (pp_buf_sel_s = '1') else pp_data1;


    -- Update sync edge detect
    sync_edge_det : process(int_rst_n, clk, sync, sync_prev)
    begin
        if(int_rst_n = '0') then
            sync_pulse <= '0';
        elsif(rising_edge(clk)) then
            sync_prev <= sync;
            if(sync_prev = '0' and sync = '1') then
                sync_pulse <= '1';
            elsif(clear_sync = '1') then
                sync_pulse <= '0';
            end if;
        end if;
    end process sync_edge_det;

    -- Reading registers mux
    reg_read : process(addr_rd, reg_control_lower, reg_control_upper,
                       reg_cal_b, reg_cal_m, reg_cal_multiplier, 
					   reg_cal_minvolt, reg_cal_amp, reg_cal_airp,
					   reg_cal_mode, reg_is_dirty,
                       reg_outs, reg_ins, reg_adc_value, reg_err_cnt,
                       reg_timerr_cnt)
    begin
        case addr_rd is
            when REG_MAGIC_ADD =>
              odata <= MAGIC_NUM_VAL;
            when REG_CONTROL_ADD =>
                odata <= reg_control_upper & reg_control_lower;
            when REG_B_ADD =>
                odata <= reg_cal_b;
            when REG_M_ADD =>
                odata <= reg_cal_m;
            when REG_MULTIPLIER_ADD =>
                odata <= reg_cal_multiplier;
            when REG_MINVOLT_ADD =>
                odata <= reg_cal_minvolt;
            when REG_AMP_ADD =>
                odata <= reg_cal_amp;
            when REG_AIRP_ADD =>
                odata <= reg_cal_airp;
            when REG_MODE_ADD =>
                odata <= reg_cal_mode;
            when REG_OUTS_ADD =>
                odata <= reg_outs;
            when REG_INS_ADD =>
                odata <= reg_ins;
            when REG_ADCVAL_ADD =>
                odata <= reg_adc_value;
            when REG_ERRCNT_ADD =>
                odata <= reg_err_cnt;
            when REG_TIMERRCNT_ADD =>
                odata <= reg_timerr_cnt;
            when others =>
                odata <= (others => '0');
        end case;
    end process reg_read;

    -- Writing registers process
    reg_write : process(int_rst_n, clk, addr_wr, wr, idata)
    begin
        if(int_rst_n = '0') then
            reg_control_lower <= (others => '0');
            reg_outs <= (others => '0');
            reg_cal_multiplier <= (others => '0');
            reg_cal_minvolt <= (others => '0');
        elsif(rising_edge(clk)) then
            if(wr = '1') then
                case addr_wr is
                    when REG_CONTROL_ADD =>
                        for byte_index in 0 to 1 loop
                          if (wr_strobe(byte_index) = '1') then
                            -- Respective byte enables are asserted as per write strobes
                            reg_control_lower(byte_index*8+7 downto byte_index*8) <= idata(byte_index*8+7 downto byte_index*8);
                          end if;
                        end loop;
					when REG_MULTIPLIER_ADD =>
						for byte_index in 0 to 3 loop
							if (wr_strobe(byte_index) = '1') then
								-- Respective byte enables are asserted as per write strobes
								reg_cal_multiplier(byte_index*8+7 downto byte_index*8) <= idata(byte_index*8+7 downto byte_index*8);
							end if;
						end loop;
					when REG_MINVOLT_ADD =>
						for byte_index in 0 to 3 loop
							if (wr_strobe(byte_index) = '1') then
								-- Respective byte enables are asserted as per write strobes
								reg_cal_minvolt(byte_index*8+7 downto byte_index*8) <= idata(byte_index*8+7 downto byte_index*8);
							end if;
						end loop;
                    when REG_OUTS_ADD =>
                        for byte_index in 0 to 3 loop
                            if (wr_strobe(byte_index) = '1') then
                                -- Respective byte enables are asserted as per write strobes
                                reg_outs(byte_index*8+7 downto byte_index*8) <= idata(byte_index*8+7 downto byte_index*8);
                            end if;
                        end loop;
                    when others =>
                        -- nope, don't write to readonly addresses
                end case;
            else
                reg_control_lower <= reg_control_lower;
                reg_outs <= reg_outs;
            end if;
        end if;
    end process reg_write;

	-- Dirty register flags for syncing to remote board
	reg_is_dirty_s(4 downto 2) <= (others => '0');
	reg_is_dirty_s(1) <= '1' when ((wr = '1') and (addr_wr = REG_MINVOLT_ADD)) else '0';
	reg_is_dirty_s(0) <= '1' when ((wr = '1') and (addr_wr = REG_MULTIPLIER_ADD)) else '0';

	reg_is_dirty_r(4 downto 2) <= (others => '0');
	reg_is_dirty_r(1) <= '1' when ((current_state = latch_reg) and (reg_add = PKT_QRY_CAL_MINVOLT_ADD)
								   AND (reg_32_tmp = reg_cal_minvolt)) else '0';
	reg_is_dirty_r(0) <= '1' when ((current_state = latch_reg) and (reg_add = PKT_QRY_CAL_MULTIPLIER_ADD)
								   AND (reg_32_tmp = reg_cal_multiplier)) else '0';

	reg_is_dirty_proc : process(int_rst_n, clk, reg_is_dirty_s, reg_is_dirty_r, reg_is_dirty)
	begin
		if(int_rst_n = '0') then
			reg_is_dirty <= (others => '0');
		elsif(rising_edge(clk)) then
			for bit_index in 0 to 4 loop
				if(reg_is_dirty_s(bit_index) = '1') then
					reg_is_dirty(bit_index) <= '1';
				elsif(reg_is_dirty_r(bit_index) = '1') then
					reg_is_dirty(bit_index) <= '0';
				else
					reg_is_dirty(bit_index) <= reg_is_dirty(bit_index);
				end if;			
			end loop;		
		end if;
	end process reg_is_dirty_proc;

    -- TX Packet muxing
    tx_pack_sel : process(com_state, reg_outs, reg_cal_multiplier, reg_cal_minvolt)
    begin
        case com_state is
            when send_qry_b =>
                pkt_tx_data <= PKT_PAD_2B & PKT_PAD_2B & PKT_QRY_CAL_CMD & PKT_QRY_CAL_B_ADD;
				pkt_tx_data_len <= PKT_TX_LEN_2B;
          	when send_qry_m =>
                pkt_tx_data <= PKT_PAD_2B & PKT_PAD_2B & PKT_QRY_CAL_CMD & PKT_QRY_CAL_M_ADD;
				pkt_tx_data_len <= PKT_TX_LEN_2B;
          	when send_set_mult => -- Pack little endian
                pkt_tx_data <= PKT_SETQRY_CAL_CMD & PKT_QRY_CAL_MULTIPLIER_ADD & 
								reg_cal_multiplier(7 downto 0) & reg_cal_multiplier(15 downto 8) &
								reg_cal_multiplier(23 downto 16) & reg_cal_multiplier(31 downto 24);
				pkt_tx_data_len <= PKT_TX_LEN_6B;
          	when send_set_minvolt => -- Pack little endian
                pkt_tx_data <= PKT_SETQRY_CAL_CMD & PKT_QRY_CAL_MINVOLT_ADD & 
								reg_cal_minvolt(7 downto 0) & reg_cal_minvolt(15 downto 8) &
								reg_cal_minvolt(23 downto 16) & reg_cal_minvolt(31 downto 24);
				pkt_tx_data_len <= PKT_TX_LEN_6B;
            when send_qry_data =>
                pkt_tx_data <= PKT_PAD_2B & PKT_PAD_2B & PKT_SETQRY_DATA_CMD & reg_outs(7 downto 0);
				pkt_tx_data_len <= PKT_TX_LEN_2B;
            when others =>
                pkt_tx_data <= (others => '0');
				pkt_tx_data_len <= (others => '0');
        end case;
    end process tx_pack_sel;

    -- Temp data latching
    latch_32 : process(int_rst_n, clk, current_state, pp_data, reg_32_tmp)
    begin
        if(int_rst_n = '0') then
            reg_32_tmp <= (others => '0');
        elsif(rising_edge(clk))then
            case current_state is
                when parse_32_1 =>
                    reg_32_tmp(7 downto 0) <= pp_data;
                when parse_32_2 =>
                    reg_32_tmp(15 downto 8) <= pp_data;
                when parse_32_3 =>
                    reg_32_tmp(23 downto 16) <= pp_data;
                when parse_32_4 =>
                    reg_32_tmp(31 downto 24) <= pp_data;
                when others =>
                    reg_32_tmp <= reg_32_tmp;
            end case;
        end if;
    end process latch_32;

    -- Internal register writing from packet parsing, etc.
    upd_int_reg : process(int_rst_n, clk, current_state, com_state, reg_add, pp_data, timed_out, reg_timerr_cnt, reg_32_tmp, reg_err_cnt, reg_control_upper, cons_timeouts)
    begin
        if(int_rst_n = '0') then
            reg_cal_b <= (others => '0');
            reg_cal_m <= (others => '0');
            reg_ins <= (others => '0');
            reg_adc_value <= (others => '0');
            reg_control_upper <= (others => '0');
            reg_err_cnt <= (others => '0');
            reg_timerr_cnt <= (others => '0');
            cons_timeouts <= (others => '0');
            reg_add <= (others => '0');
        elsif (rising_edge(clk)) then
            case current_state is
                when wait_for_resp =>
                    if(timed_out = '1') then
                        reg_timerr_cnt <= std_logic_vector(unsigned(reg_timerr_cnt) + 1);
                        cons_timeouts <= cons_timeouts  + 1;
                    end if;
                when parse_resp_cmd =>
                    cons_timeouts <= (others => '0');
                    if(pp_data /= PKT_QRY_CAL_CMD and pp_data /= PKT_QRY_DATA_CMD) then
                        reg_err_cnt <= std_logic_vector(unsigned(reg_err_cnt) + 1);
                    end if;
                when parse_data1 =>
                    reg_ins <= x"000000" & pp_data;
                    reg_add <= x"00";
                when parse_cal1 =>
                    reg_add <= pp_data;
                when latch_reg =>
                    case reg_add is
                        when PKT_QRY_CAL_B_ADD =>
                            reg_cal_b <= reg_32_tmp;
                        when PKT_QRY_CAL_M_ADD =>
                            reg_cal_m <= reg_32_tmp;
                        when PKT_QRY_DATA_ADC_ADD =>
                            reg_adc_value <= reg_32_tmp;
                        when others =>
                            -- nope, don't save to invalid address
                    end case;
                when others =>

            end case;
			-- Ready flag is true when we establish coms
			if(com_state = send_qry_b or com_state = send_qry_m) then                
                reg_control_upper(0) <= '0';
			else
				reg_control_upper(0) <= '1';			
			end if;
        end if;
    end process upd_int_reg;

    calc_pp_addr : process(current_state)
    begin
        case current_state is
            when parse_resp_cmd =>
                pp_addr <= b"000001";
            when parse_data1 | parse_cal1 =>
                pp_addr <= b"000010";
            when parse_32_1 =>
                pp_addr <= b"000011";
            when parse_32_2 =>
                pp_addr <= b"000100";
            when parse_32_3 =>
                pp_addr <= b"000101";
            when others =>
                pp_addr <= (others => '0');
        end case;
    end process calc_pp_addr;

    upd_timeout : process(int_rst_n, clk, current_state, com_state, timeout_tmr, timed_out)
    begin
        if(int_rst_n = '0') then
            timeout_tmr <= TIMEOUT_COUNT_VAL_ST;
            timed_out <= '0';
        elsif (rising_edge(clk)) then
            timed_out <= '0';
			if(current_state = send) then
				if(com_state = send_qry_data) then
		            timeout_tmr <= TIMEOUT_COUNT_VAL_DATA;
				else
					timeout_tmr <= TIMEOUT_COUNT_VAL_ST;		
				end if;
			elsif(current_state = latch_reg) then
				timeout_tmr <= TIMEOUT_COUNT_VAL_SYNC;
			elsif((current_state = wait_for_resp) or (current_state = wait_for_sync)) then
                if(timeout_tmr > to_unsigned(0,32)) then
                    timeout_tmr <= timeout_tmr - 1;
                else -- we timed out
                    timed_out <= '1';
                end if;
			else
				timeout_tmr <= TIMEOUT_COUNT_VAL_ST;
			end if;
        end if;
    end process upd_timeout;

    update_pp_buf_sig : process(int_rst_n, clk, pp_buf_lock, pp_buf_lock_s)
    begin
        if(int_rst_n = '0') then
            pp_buf_lock_s <= pp_buf_lock;
            pp_buf_sel_s <= '0';
        elsif(rising_edge(clk)) then
            pp_buf_lock_s <= pp_buf_lock;
            pp_buf_change <= '0';
            if(current_state = wait_for_resp) then
              if(pp_buf_lock = '1' and pp_buf_lock_s = '0') then
                pp_buf_sel_s <= '0';
                pp_buf_change <= '1';
              elsif(pp_buf_lock = '0' and pp_buf_lock_s = '1')then
                pp_buf_sel_s <= '1';
                pp_buf_change <= '1';
              end if;
            end if;
        end if;
    end process update_pp_buf_sig;

    update_state : process(rst_n, clk, next_state, com_next_state, reg_control_lower)
    begin
        if((rst_n = '0') or (reg_control_lower(0) = '1')) then
            current_state <= start;
			com_state <= send_qry_b;
        elsif(rising_edge(clk)) then
            current_state <= next_state;
			com_state <= com_next_state;
        end if;
    end process update_state;


	-- State machine calculations. There are two state machines running. 
	-- This one handles the low level actually pumping out a packet 
	-- every frame.
    calc_state : process(current_state, pp_buf_change, timed_out, cons_timeouts, reg_control_lower, pkt_tx_we_s, uart_tx_busy, pkt_tx_busy, pp_data, sync_pulse)
    begin
        next_state <= current_state; -- Hold state by default
        case current_state is
            when start =>
                next_state <= send;
            when send =>
                if(pkt_tx_we_s = '1') then
                    next_state <= wait_for_tx;
                end if;
			when wait_for_tx =>
				if(uart_tx_busy = '0' and pkt_tx_busy = '0') then
					next_state <= wait_for_resp;
				end if;
            when wait_for_resp =>
                if(pp_buf_change = '1') then   -- Change is registered with the select, so the pp delay is built in
                    next_state <= parse_resp_cmd;
                elsif(timed_out = '1') then
                    if(cons_timeouts  > TIMEOUT_RETRIES) then   -- Completely lost coms
                        next_state <= start;
                    else                        -- Missed a packet
                        next_state <= send;
                    end if;
                end if;
            when parse_resp_cmd =>
                if(pp_data = PKT_QRY_CAL_CMD) then
                    next_state <= parse_cal1;
                elsif(pp_data = PKT_QRY_DATA_CMD) then
                    next_state <= parse_data1;
                else
                    next_state <= wait_for_sync;
                end if;
            when parse_cal1 =>
                next_state <= parse_32_1;
            when parse_data1 =>
                next_state <= parse_32_1;
            when parse_32_1 =>
                next_state <= parse_32_2;
            when parse_32_2 =>
                next_state <= parse_32_3;
            when parse_32_3 =>
                next_state <= parse_32_4;
            when parse_32_4 =>
                next_state <= latch_reg;
            when latch_reg =>
				next_state <= wait_for_sync;
			-- All packets are synced to a 1ms communication frame
            when wait_for_sync =>
                if(sync_pulse = '1' or timed_out = '1') then
					next_state <= send;
                end if;
            when others =>
                next_state <= start;
        end case;
    end process calc_state;

	calc_com_state : process(current_state, com_state, sync_pulse, timed_out, reg_is_dirty)
	begin
		com_next_state <= com_state; -- Hold state by default
		-- We can only change the content of packet at frame boundary
		if(current_state = start) then
			com_next_state <= send_qry_b;	-- Catch timeout reset	
		elsif(current_state = wait_for_sync) then
		    if(sync_pulse = '1' or timed_out = '1') then
				case com_state is
					when send_qry_b =>	-- Startup handshake is always ask for b, ask for m.
						com_next_state <= send_qry_m;
					when others =>
					-- Dirty registers have priority after startup	
						if(reg_is_dirty(0) = '1') then
							com_next_state <= send_set_mult;
						elsif(reg_is_dirty(1) = '1') then
							com_next_state <= send_set_minvolt;
						else
							com_next_state <= send_qry_data;
						end if;	
				end case;
		    end if;
		end if;
	end process calc_com_state;
end beh;

