## ===== Clock 100 MHz (Basys3) =====
set_property PACKAGE_PIN W5 [get_ports sys_clock]
set_property IOSTANDARD LVCMOS33 [get_ports sys_clock]
create_clock -add -name sys_clk_pin -period 10.00 -waveform {0 5} [get_ports sys_clock]

## ===== Reset (scegli un pulsante: qui uso btnC come reset) =====
set_property PACKAGE_PIN U18 [get_ports reset]
set_property IOSTANDARD LVCMOS33 [get_ports reset]

## ===== UART USB (FTDI) =====
# RsRx = RX verso FPGA (dalla USB)  -> tipicamente lo colleghi al tuo usb_uart_rxd
set_property PACKAGE_PIN B18 [get_ports usb_uart_rxd]
set_property IOSTANDARD LVCMOS33 [get_ports usb_uart_rxd]

# RsTx = TX dalla FPGA (verso USB)  -> tipicamente lo colleghi al tuo usb_uart_txd
set_property PACKAGE_PIN A18 [get_ports usb_uart_txd]
set_property IOSTANDARD LVCMOS33 [get_ports usb_uart_txd]

## ===== Tre LED (mappo su LED0..LED2) =====
set_property PACKAGE_PIN U16 [get_ports led_ok]
set_property IOSTANDARD LVCMOS33 [get_ports led_ok]

set_property PACKAGE_PIN E19 [get_ports led_uf]
set_property IOSTANDARD LVCMOS33 [get_ports led_uf]

set_property PACKAGE_PIN U19 [get_ports led_of]
set_property IOSTANDARD LVCMOS33 [get_ports led_of]
