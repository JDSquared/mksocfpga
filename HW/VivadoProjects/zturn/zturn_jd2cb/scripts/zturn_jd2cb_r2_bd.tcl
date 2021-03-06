################################################################
# Check if script is running in correct Vivado version.
################################################################
set scripts_vivado_version 2015.4
set current_vivado_version [version -short]

if { [string first $scripts_vivado_version $current_vivado_version] == -1 } {
   puts ""
   puts "ERROR: This script was generated using Vivado <$scripts_vivado_version> and is being run in <$current_vivado_version> of Vivado. Please run the script in Vivado <$scripts_vivado_version> then open the design in Vivado <$current_vivado_version>. Upgrade the design by running \"Tools => Report => Report IP Status...\", then run write_bd_tcl to create an updated script."

   return 1
}

################################################################
# START
################################################################

# CHECKING IF PROJECT EXISTS
if { [get_projects -quiet] eq "" } {
   puts "ERROR: Please open or create a project!"
   return 1
}

# CHANGE DESIGN NAME HERE
set design_name soc_system

# This script was generated for a remote BD
set str_bd_folder [get_property directory [current_project]]/src
set str_bd_filepath ${str_bd_folder}/${design_name}.bd

# Check if remote design exists on disk
if { [file exists $str_bd_filepath ] == 1 } {
   puts "ERROR: The remote BD file path <$str_bd_filepath> already exists!\n"

   puts "INFO: Please modify the variable <str_bd_folder> to another path or modify the variable <design_name>."

   return 1
}

# Check if design exists in memory
set list_existing_designs [get_bd_designs -quiet $design_name]
if { $list_existing_designs ne "" } {
   puts "ERROR: The design <$design_name> already exists in this project!"
   puts "ERROR: Will not create the remote BD <$design_name> at the folder <$str_bd_folder>.\n"

   puts "INFO: Please modify the variable <design_name>."

   return 1
}

# Check if design exists on disk within project
set list_existing_designs [get_files */${design_name}.bd]
if { $list_existing_designs ne "" } {
   puts "ERROR: The design <$design_name> already exists in this project at location:"
   puts "   $list_existing_designs"
   puts "ERROR: Will not create the remote BD <$design_name> at the folder <$str_bd_folder>.\n"

   puts "INFO: Please modify the variable <design_name>."

   return 1
}

# Now can create the remote BD
create_bd_design -dir $str_bd_folder $design_name
current_bd_design $design_name

##################################################################
# DESIGN PROCs
##################################################################



# Procedure to create entire design; Provide argument to make
# procedure reusable. If parentCell is "", will use root.
proc create_root_design { parentCell } {

  if { $parentCell eq "" } {
     set parentCell [get_bd_cells /]
  }

  # Get object for parentCell
  set parentObj [get_bd_cells $parentCell]
  if { $parentObj == "" } {
     puts "ERROR: Unable to find parent cell <$parentCell>!"
     return
  }

  # Make sure parentObj is hier blk
  set parentType [get_property TYPE $parentObj]
  if { $parentType ne "hier" } {
     puts "ERROR: Parent <$parentObj> has TYPE = <$parentType>. Expected to be <hier>."
     return
  }

  # Save current instance; Restore later
  set oldCurInst [current_bd_instance .]

  # Set parent object as current
  current_bd_instance $parentObj


  # Create interface ports
  set DDR [ create_bd_intf_port -mode Master -vlnv xilinx.com:interface:ddrx_rtl:1.0 DDR ]
  set FIXED_IO [ create_bd_intf_port -mode Master -vlnv xilinx.com:display_processing_system7:fixedio_rtl:1.0 FIXED_IO ]

  # Create ports
  set IOBits [ create_bd_port -dir IO -from 71 -to 0 IOBits ]
  set uart0_rxd [ create_bd_port -dir I -type data uart0_rxd ]
  set uart0_rxd_en [ create_bd_port -dir O -type data uart0_rxd_en ]
  set uart0_txd [ create_bd_port -dir O -type data uart0_txd ]
  set uart0_txd_en [ create_bd_port -dir O -type data uart0_txd_en ]
  set uart1_rxd [ create_bd_port -dir I uart1_rxd ]
  set uart1_rxd_en [ create_bd_port -dir O -from 0 -to 0 uart1_rxd_en ]
  set uart1_txd [ create_bd_port -dir O uart1_txd ]
  set uart1_txd_en [ create_bd_port -dir O uart1_txd_en ]

  # Create instance: HostMot2_ip_wrap_0, and set properties
  set HostMot2_ip_wrap_0 [ create_bd_cell -type ip -vlnv machinekit.io:user:HostMot2_ip_wrap:1.0 HostMot2_ip_wrap_0 ]
  set_property -dict [ list \
CONFIG.IOPorts {3} \
CONFIG.IOWidth {72} \
CONFIG.PortWidth {24} \
 ] $HostMot2_ip_wrap_0

  # Create instance: btint_axi_0, and set properties
  set btint_axi_0 [ create_bd_cell -type ip -vlnv jd2.com:user:btint_axi:2.0 btint_axi_0 ]

  # Create instance: hm2_axilite_int_0, and set properties
  set hm2_axilite_int_0 [ create_bd_cell -type ip -vlnv machinekit.io:user:hm2_axilite_int:1.0 hm2_axilite_int_0 ]

  # Create instance: hm2_io_ts_0, and set properties
  set hm2_io_ts_0 [ create_bd_cell -type ip -vlnv machinekit.io:user:hm2_io_ts:1 hm2_io_ts_0 ]
  set_property -dict [ list \
CONFIG.WIDTH {72} \
 ] $hm2_io_ts_0

  # Create instance: jd2_mad_logic_r2_0, and set properties
  set jd2_mad_logic_r2_0 [ create_bd_cell -type ip -vlnv jd2.com:user:jd2_mad_logic_r2:2 jd2_mad_logic_r2_0 ]
  set_property -dict [ list \
CONFIG.WIDTH {72} \
 ] $jd2_mad_logic_r2_0

  # Create instance: jd2_mad_ologic_r2_0, and set properties
  set jd2_mad_ologic_r2_0 [ create_bd_cell -type ip -vlnv jd2.com:user:jd2_mad_ologic_r2:2 jd2_mad_ologic_r2_0 ]
  set_property -dict [ list \
CONFIG.WIDTH {72} \
 ] $jd2_mad_ologic_r2_0

  # Create instance: processing_system7_0, and set properties
  set processing_system7_0 [ create_bd_cell -type ip -vlnv xilinx.com:ip:processing_system7:5.5 processing_system7_0 ]
  set_property -dict [ list \
CONFIG.PCW_CAN0_CAN0_IO {MIO 14 .. 15} \
CONFIG.PCW_CAN0_PERIPHERAL_ENABLE {1} \
CONFIG.PCW_ENET0_ENET0_IO {MIO 16 .. 27} \
CONFIG.PCW_ENET0_GRP_MDIO_ENABLE {1} \
CONFIG.PCW_ENET0_PERIPHERAL_ENABLE {1} \
CONFIG.PCW_ENET0_RESET_ENABLE {1} \
CONFIG.PCW_ENET0_RESET_IO {MIO 51} \
CONFIG.PCW_EN_CLK1_PORT {1} \
CONFIG.PCW_EN_CLK2_PORT {0} \
CONFIG.PCW_FPGA0_PERIPHERAL_FREQMHZ {100} \
CONFIG.PCW_FPGA1_PERIPHERAL_FREQMHZ {200} \
CONFIG.PCW_FPGA2_PERIPHERAL_FREQMHZ {50} \
CONFIG.PCW_GPIO_EMIO_GPIO_ENABLE {0} \
CONFIG.PCW_GPIO_MIO_GPIO_ENABLE {1} \
CONFIG.PCW_I2C0_PERIPHERAL_ENABLE {0} \
CONFIG.PCW_I2C0_RESET_ENABLE {0} \
CONFIG.PCW_I2C1_I2C1_IO {MIO 12 .. 13} \
CONFIG.PCW_I2C1_PERIPHERAL_ENABLE {1} \
CONFIG.PCW_IRQ_F2P_INTR {1} \
CONFIG.PCW_MIO_0_PULLUP {disabled} \
CONFIG.PCW_MIO_16_SLEW {slow} \
CONFIG.PCW_MIO_17_SLEW {fast} \
CONFIG.PCW_MIO_18_SLEW {fast} \
CONFIG.PCW_MIO_19_SLEW {fast} \
CONFIG.PCW_MIO_20_SLEW {fast} \
CONFIG.PCW_MIO_21_SLEW {fast} \
CONFIG.PCW_MIO_22_SLEW {fast} \
CONFIG.PCW_MIO_23_SLEW {fast} \
CONFIG.PCW_MIO_24_SLEW {fast} \
CONFIG.PCW_MIO_25_SLEW {fast} \
CONFIG.PCW_MIO_26_SLEW {fast} \
CONFIG.PCW_MIO_27_SLEW {fast} \
CONFIG.PCW_MIO_46_PULLUP {disabled} \
CONFIG.PCW_MIO_47_PULLUP {disabled} \
CONFIG.PCW_MIO_50_PULLUP {disabled} \
CONFIG.PCW_MIO_51_PULLUP {disabled} \
CONFIG.PCW_MIO_9_PULLUP {disabled} \
CONFIG.PCW_PRESET_BANK1_VOLTAGE {LVCMOS 1.8V} \
CONFIG.PCW_QSPI_GRP_FBCLK_ENABLE {1} \
CONFIG.PCW_QSPI_GRP_SINGLE_SS_ENABLE {1} \
CONFIG.PCW_QSPI_PERIPHERAL_ENABLE {1} \
CONFIG.PCW_SD0_GRP_CD_ENABLE {1} \
CONFIG.PCW_SD0_GRP_CD_IO {MIO 46} \
CONFIG.PCW_SD0_GRP_WP_ENABLE {1} \
CONFIG.PCW_SD0_GRP_WP_IO {MIO 47} \
CONFIG.PCW_SD0_PERIPHERAL_ENABLE {1} \
CONFIG.PCW_TTC0_PERIPHERAL_ENABLE {1} \
CONFIG.PCW_UART0_PERIPHERAL_ENABLE {1} \
CONFIG.PCW_UART0_UART0_IO {MIO 10 .. 11} \
CONFIG.PCW_UART1_PERIPHERAL_ENABLE {1} \
CONFIG.PCW_UIPARAM_DDR_BOARD_DELAY0 {0.271} \
CONFIG.PCW_UIPARAM_DDR_BOARD_DELAY1 {0.259} \
CONFIG.PCW_UIPARAM_DDR_BOARD_DELAY2 {0.219} \
CONFIG.PCW_UIPARAM_DDR_BOARD_DELAY3 {0.207} \
CONFIG.PCW_UIPARAM_DDR_DQS_TO_CLK_DELAY_0 {0.229} \
CONFIG.PCW_UIPARAM_DDR_DQS_TO_CLK_DELAY_1 {0.250} \
CONFIG.PCW_UIPARAM_DDR_DQS_TO_CLK_DELAY_2 {0.121} \
CONFIG.PCW_UIPARAM_DDR_DQS_TO_CLK_DELAY_3 {0.146} \
CONFIG.PCW_UIPARAM_DDR_PARTNO {MT41J256M16 RE-125} \
CONFIG.PCW_UIPARAM_DDR_TRAIN_DATA_EYE {1} \
CONFIG.PCW_UIPARAM_DDR_TRAIN_READ_GATE {1} \
CONFIG.PCW_UIPARAM_DDR_TRAIN_WRITE_LEVEL {1} \
CONFIG.PCW_UIPARAM_GENERATE_SUMMARY {NONE} \
CONFIG.PCW_USB0_PERIPHERAL_ENABLE {1} \
CONFIG.PCW_USB0_RESET_ENABLE {1} \
CONFIG.PCW_USB0_RESET_IO {MIO 7} \
CONFIG.PCW_USE_FABRIC_INTERRUPT {1} \
CONFIG.PCW_USE_M_AXI_GP1 {0} \
CONFIG.PCW_USE_S_AXI_HP0 {0} \
 ] $processing_system7_0

  # Create instance: processing_system7_0_axi_periph, and set properties
  set processing_system7_0_axi_periph [ create_bd_cell -type ip -vlnv xilinx.com:ip:axi_interconnect:2.1 processing_system7_0_axi_periph ]
  set_property -dict [ list \
CONFIG.NUM_MI {3} \
 ] $processing_system7_0_axi_periph

  # Create instance: rst_processing_system7_0_100M, and set properties
  set rst_processing_system7_0_100M [ create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:5.0 rst_processing_system7_0_100M ]

  # Create instance: xlconcat_0, and set properties
  set xlconcat_0 [ create_bd_cell -type ip -vlnv xilinx.com:ip:xlconcat:2.1 xlconcat_0 ]
  set_property -dict [ list \
CONFIG.NUM_PORTS {1} \
 ] $xlconcat_0

  # Create interface connections
  connect_bd_intf_net -intf_net processing_system7_0_M_AXI_GP0 [get_bd_intf_pins processing_system7_0/M_AXI_GP0] [get_bd_intf_pins processing_system7_0_axi_periph/S00_AXI]
  connect_bd_intf_net -intf_net processing_system7_0_axi_periph_M00_AXI [get_bd_intf_pins btint_axi_0/S_AXI] [get_bd_intf_pins processing_system7_0_axi_periph/M00_AXI]
  connect_bd_intf_net -intf_net processing_system7_0_axi_periph_M01_AXI [get_bd_intf_pins hm2_axilite_int_0/S_AXI] [get_bd_intf_pins processing_system7_0_axi_periph/M01_AXI]
  connect_bd_intf_net -intf_net processing_system7_1_DDR [get_bd_intf_ports DDR] [get_bd_intf_pins processing_system7_0/DDR]
  connect_bd_intf_net -intf_net processing_system7_1_FIXED_IO [get_bd_intf_ports FIXED_IO] [get_bd_intf_pins processing_system7_0/FIXED_IO]

  # Create port connections
  connect_bd_net -net HostMot2_ip_wrap_0_interrupt [get_bd_pins HostMot2_ip_wrap_0/interrupt] [get_bd_pins xlconcat_0/In0]
  connect_bd_net -net HostMot2_ip_wrap_0_ioddrbits [get_bd_pins HostMot2_ip_wrap_0/ioddrbits] [get_bd_pins hm2_io_ts_0/ddr_bits]
  connect_bd_net -net HostMot2_ip_wrap_0_ioodrbits [get_bd_pins HostMot2_ip_wrap_0/ioodrbits] [get_bd_pins hm2_io_ts_0/odr_bits]
  connect_bd_net -net HostMot2_ip_wrap_0_obus [get_bd_pins HostMot2_ip_wrap_0/obus] [get_bd_pins hm2_axilite_int_0/OBUS]
  connect_bd_net -net HostMot2_ip_wrap_0_outbits [get_bd_pins HostMot2_ip_wrap_0/outbits] [get_bd_pins jd2_mad_ologic_r2_0/OUTS]
  connect_bd_net -net Net [get_bd_ports IOBits] [get_bd_pins hm2_io_ts_0/iobits]
  connect_bd_net -net btint_axi_0_UART_RX_EN [get_bd_ports uart0_rxd_en] [get_bd_pins btint_axi_0/UART_RX_EN]
  connect_bd_net -net btint_axi_0_UART_TX_EN [get_bd_ports uart0_txd_en] [get_bd_pins btint_axi_0/UART_TX_EN]
  connect_bd_net -net btint_axi_1_UART_TX [get_bd_ports uart0_txd] [get_bd_pins btint_axi_0/UART_TX]
  connect_bd_net -net hm2_axilite_int_0_ADDR [get_bd_pins HostMot2_ip_wrap_0/addr] [get_bd_pins hm2_axilite_int_0/ADDR]
  connect_bd_net -net hm2_axilite_int_0_IBUS [get_bd_pins HostMot2_ip_wrap_0/ibus] [get_bd_pins hm2_axilite_int_0/IBUS]
  connect_bd_net -net hm2_axilite_int_0_READSTB [get_bd_pins HostMot2_ip_wrap_0/readstb] [get_bd_pins hm2_axilite_int_0/READSTB]
  connect_bd_net -net hm2_axilite_int_0_WRITESTB [get_bd_pins HostMot2_ip_wrap_0/writestb] [get_bd_pins hm2_axilite_int_0/WRITESTB]
  connect_bd_net -net hm2_io_ts_0_i_bits [get_bd_pins hm2_io_ts_0/i_bits] [get_bd_pins jd2_mad_logic_r2_0/INS]
  connect_bd_net -net jd2_mad_logic_r2_0_LOG_INS [get_bd_pins HostMot2_ip_wrap_0/inbits] [get_bd_pins jd2_mad_logic_r2_0/LOG_INS]
  connect_bd_net -net jd2_mad_ologic_r2_0_LOG_OUTS [get_bd_pins hm2_io_ts_0/o_bits] [get_bd_pins jd2_mad_ologic_r2_0/LOG_OUTS]
  connect_bd_net -net processing_system7_0_FCLK_CLK0 [get_bd_pins HostMot2_ip_wrap_0/clklow] [get_bd_pins HostMot2_ip_wrap_0/clkmed] [get_bd_pins btint_axi_0/S_AXI_ACLK] [get_bd_pins hm2_axilite_int_0/S_AXI_ACLK] [get_bd_pins jd2_mad_logic_r2_0/clk] [get_bd_pins processing_system7_0/FCLK_CLK0] [get_bd_pins processing_system7_0/M_AXI_GP0_ACLK] [get_bd_pins processing_system7_0_axi_periph/ACLK] [get_bd_pins processing_system7_0_axi_periph/M00_ACLK] [get_bd_pins processing_system7_0_axi_periph/M01_ACLK] [get_bd_pins processing_system7_0_axi_periph/M02_ACLK] [get_bd_pins processing_system7_0_axi_periph/S00_ACLK] [get_bd_pins rst_processing_system7_0_100M/slowest_sync_clk]
  connect_bd_net -net processing_system7_0_FCLK_CLK1 [get_bd_pins HostMot2_ip_wrap_0/clkhigh] [get_bd_pins processing_system7_0/FCLK_CLK1]
  connect_bd_net -net processing_system7_0_FCLK_RESET0_N [get_bd_pins processing_system7_0/FCLK_RESET0_N] [get_bd_pins rst_processing_system7_0_100M/ext_reset_in]
  connect_bd_net -net rst_processing_system7_0_100M_interconnect_aresetn [get_bd_pins processing_system7_0_axi_periph/ARESETN] [get_bd_pins rst_processing_system7_0_100M/interconnect_aresetn]
  connect_bd_net -net rst_processing_system7_0_100M_peripheral_aresetn [get_bd_pins btint_axi_0/S_AXI_ARESETN] [get_bd_pins hm2_axilite_int_0/S_AXI_ARESETN] [get_bd_pins processing_system7_0_axi_periph/M00_ARESETN] [get_bd_pins processing_system7_0_axi_periph/M01_ARESETN] [get_bd_pins processing_system7_0_axi_periph/M02_ARESETN] [get_bd_pins processing_system7_0_axi_periph/S00_ARESETN] [get_bd_pins rst_processing_system7_0_100M/peripheral_aresetn]
  connect_bd_net -net uart0_rxd_1 [get_bd_ports uart0_rxd] [get_bd_pins btint_axi_0/UART_RX]
  connect_bd_net -net xlconcat_0_dout [get_bd_pins btint_axi_0/sync] [get_bd_pins processing_system7_0/IRQ_F2P] [get_bd_pins xlconcat_0/dout]

  # Create address segments
  create_bd_addr_seg -range 0x10000 -offset 0x43C10000 [get_bd_addr_spaces processing_system7_0/Data] [get_bd_addr_segs btint_axi_0/S_AXI/reg0] SEG_btint_axi_0_reg0
  create_bd_addr_seg -range 0x10000 -offset 0x43C00000 [get_bd_addr_spaces processing_system7_0/Data] [get_bd_addr_segs hm2_axilite_int_0/S_AXI/reg0] SEG_hm2_axilite_int_0_reg0

  # Perform GUI Layout
  regenerate_bd_layout -layout_string {
   guistr: "# # String gsaved with Nlview 6.5.5  2015-06-26 bk=1.3371 VDI=38 GEI=35 GUI=JA:1.8
#  -string -flagsOSRD
preplace port DDR -pg 1 -y 310 -defaultsOSRD
preplace port uart0_rxd -pg 1 -y 850 -defaultsOSRD -right
preplace port uart1_txd_en -pg 1 -y 80 -defaultsOSRD
preplace port uart1_txd -pg 1 -y 60 -defaultsOSRD
preplace port uart1_rxd -pg 1 -y 20 -defaultsOSRD -right
preplace port uart0_txd_en -pg 1 -y 760 -defaultsOSRD
preplace port uart0_rxd_en -pg 1 -y 780 -defaultsOSRD
preplace port FIXED_IO -pg 1 -y 330 -defaultsOSRD
preplace port uart0_txd -pg 1 -y 740 -defaultsOSRD
preplace portBus uart1_rxd_en -pg 1 -y 40 -defaultsOSRD
preplace portBus IOBits -pg 1 -y 590 -defaultsOSRD
preplace inst rst_processing_system7_0_100M -pg 1 -lvl 4 -y 180 -defaultsOSRD
preplace inst xlconcat_0 -pg 1 -lvl 3 -y 540 -defaultsOSRD
preplace inst jd2_mad_ologic_r2_0 -pg 1 -lvl 5 -y 570 -defaultsOSRD
preplace inst hm2_io_ts_0 -pg 1 -lvl 6 -y 580 -defaultsOSRD
preplace inst hm2_axilite_int_0 -pg 1 -lvl 6 -y 410 -defaultsOSRD
preplace inst btint_axi_0 -pg 1 -lvl 6 -y 760 -defaultsOSRD
preplace inst HostMot2_ip_wrap_0 -pg 1 -lvl 2 -y 600 -defaultsOSRD
preplace inst processing_system7_0_axi_periph -pg 1 -lvl 5 -y 150 -defaultsOSRD
preplace inst processing_system7_0 -pg 1 -lvl 4 -y 400 -defaultsOSRD
preplace inst jd2_mad_logic_r2_0 -pg 1 -lvl 1 -y 670 -defaultsOSRD
preplace netloc uart0_rxd_1 1 5 2 1550 850 NJ
preplace netloc processing_system7_0_axi_periph_M00_AXI 1 5 1 1570
preplace netloc HostMot2_ip_wrap_0_interrupt 1 2 1 NJ
preplace netloc processing_system7_0_M_AXI_GP0 1 4 1 1180
preplace netloc processing_system7_0_FCLK_RESET0_N 1 3 2 760 90 1170
preplace netloc hm2_io_ts_0_i_bits 1 0 7 20 450 NJ 450 NJ 450 NJ 560 NJ 510 NJ 510 1840
preplace netloc btint_axi_1_UART_TX 1 6 1 NJ
preplace netloc btint_axi_0_UART_RX_EN 1 6 1 NJ
preplace netloc HostMot2_ip_wrap_0_ioddrbits 1 2 4 NJ 610 NJ 610 NJ 620 1540
preplace netloc rst_processing_system7_0_100M_peripheral_aresetn 1 4 2 1220 320 1530
preplace netloc HostMot2_ip_wrap_0_outbits 1 2 3 NJ 590 NJ 590 NJ
preplace netloc xlconcat_0_dout 1 3 3 750 570 NJ 640 NJ
preplace netloc hm2_axilite_int_0_WRITESTB 1 1 6 290 750 NJ 650 NJ 650 NJ 650 NJ 650 1850
preplace netloc hm2_axilite_int_0_READSTB 1 1 6 270 460 NJ 460 NJ 550 NJ 500 NJ 500 1840
preplace netloc HostMot2_ip_wrap_0_obus 1 2 4 NJ 60 NJ 60 NJ 400 N
preplace netloc jd2_mad_ologic_r2_0_LOG_OUTS 1 5 1 1550
preplace netloc hm2_axilite_int_0_IBUS 1 1 6 280 740 NJ 670 NJ 670 NJ 670 NJ 670 1870
preplace netloc processing_system7_1_DDR 1 4 3 NJ 310 NJ 310 NJ
preplace netloc rst_processing_system7_0_100M_interconnect_aresetn 1 4 1 1190
preplace netloc processing_system7_0_FCLK_CLK0 1 0 6 30 610 250 390 NJ 390 730 80 1200 420 1580
preplace netloc jd2_mad_logic_r2_0_LOG_INS 1 1 1 N
preplace netloc hm2_axilite_int_0_ADDR 1 1 6 260 440 NJ 440 NJ 540 NJ 490 NJ 490 1860
preplace netloc btint_axi_0_UART_TX_EN 1 6 1 NJ
preplace netloc Net 1 6 1 NJ
preplace netloc HostMot2_ip_wrap_0_ioodrbits 1 2 4 NJ 630 NJ 630 NJ 630 1560
preplace netloc processing_system7_1_FIXED_IO 1 4 3 NJ 330 NJ 330 NJ
preplace netloc processing_system7_0_FCLK_CLK1 1 1 4 300 760 NJ 760 NJ 760 1160
preplace netloc processing_system7_0_axi_periph_M01_AXI 1 5 1 1580
levelinfo -pg 1 0 140 420 640 960 1370 1710 1890 -top 0 -bot 870
",
}

  # Restore current instance
  current_bd_instance $oldCurInst

  save_bd_design
}
# End of create_root_design()


##################################################################
# MAIN FLOW
##################################################################

create_root_design ""


