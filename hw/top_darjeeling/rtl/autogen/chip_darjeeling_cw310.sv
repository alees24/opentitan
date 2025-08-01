// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// ------------------- W A R N I N G: A U T O - G E N E R A T E D   C O D E !! -------------------//
// PLEASE DO NOT HAND-EDIT THIS FILE. IT HAS BEEN AUTO-GENERATED WITH THE FOLLOWING COMMAND:
//
// util/topgen.py -t hw/top_darjeeling/data/top_darjeeling.hjson \
//                -o hw/top_darjeeling/ \
//                --rnd_cnst_seed \
//                1017106219537032642877583828875051302543807092889754935647094601236425074047


module chip_darjeeling_cw310 #(
  // Path to a VMEM file containing the contents of the boot ROM, which will be
  // baked into the FPGA bitstream.
  parameter BootRomInitFile = "test_rom_fpga_cw310.32.vmem",
  // Path to a VMEM file containing the contents of the emulated OTP, which will be
  // baked into the FPGA bitstream.
  parameter OtpMacroMemInitFile = "otp_img_fpga_cw310.vmem"
) (
  // Dedicated Pads
  inout POR_N, // Manual Pad
  inout JTAG_TCK, // Manual Pad
  inout JTAG_TMS, // Manual Pad
  inout JTAG_TDI, // Manual Pad
  inout JTAG_TDO, // Manual Pad
  inout JTAG_TRST_N, // Manual Pad
  inout SPI_HOST_D0, // Dedicated Pad for spi_host0_sd
  inout SPI_HOST_D1, // Dedicated Pad for spi_host0_sd
  inout SPI_HOST_D2, // Dedicated Pad for spi_host0_sd
  inout SPI_HOST_D3, // Dedicated Pad for spi_host0_sd
  inout SPI_HOST_CLK, // Dedicated Pad for spi_host0_sck
  inout SPI_HOST_CS_L, // Dedicated Pad for spi_host0_csb
  inout SPI_DEV_D0, // Dedicated Pad for spi_device_sd
  inout SPI_DEV_D1, // Dedicated Pad for spi_device_sd
  inout SPI_DEV_D2, // Dedicated Pad for spi_device_sd
  inout SPI_DEV_D3, // Dedicated Pad for spi_device_sd
  inout SPI_DEV_CLK, // Dedicated Pad for spi_device_sck
  inout SPI_DEV_CS_L, // Dedicated Pad for spi_device_csb
  inout SPI_DEV_TPM_CS_L, // Dedicated Pad for spi_device_tpm_csb
  inout UART_RX, // Dedicated Pad for uart0_rx
  inout UART_TX, // Dedicated Pad for uart0_tx
  inout I2C_SCL, // Dedicated Pad for i2c0_scl
  inout I2C_SDA, // Dedicated Pad for i2c0_sda
  inout GPIO0, // Dedicated Pad for gpio_gpio
  inout GPIO1, // Dedicated Pad for gpio_gpio
  inout GPIO2, // Dedicated Pad for gpio_gpio
  inout GPIO3, // Dedicated Pad for gpio_gpio
  inout GPIO4, // Dedicated Pad for gpio_gpio
  inout GPIO5, // Dedicated Pad for gpio_gpio
  inout GPIO6, // Dedicated Pad for gpio_gpio
  inout GPIO7, // Dedicated Pad for gpio_gpio
  inout GPIO8, // Dedicated Pad for gpio_gpio
  inout GPIO9, // Dedicated Pad for gpio_gpio
  inout GPIO10, // Dedicated Pad for gpio_gpio
  inout GPIO11, // Dedicated Pad for gpio_gpio
  inout GPIO12, // Dedicated Pad for gpio_gpio
  inout GPIO13, // Dedicated Pad for gpio_gpio
  inout GPIO14, // Dedicated Pad for gpio_gpio
  inout GPIO15, // Dedicated Pad for gpio_gpio
  inout GPIO16, // Dedicated Pad for gpio_gpio
  inout GPIO17, // Dedicated Pad for gpio_gpio
  inout GPIO18, // Dedicated Pad for gpio_gpio
  inout GPIO19, // Dedicated Pad for gpio_gpio
  inout GPIO20, // Dedicated Pad for gpio_gpio
  inout GPIO21, // Dedicated Pad for gpio_gpio
  inout GPIO22, // Dedicated Pad for gpio_gpio
  inout GPIO23, // Dedicated Pad for gpio_gpio
  inout GPIO24, // Dedicated Pad for gpio_gpio
  inout GPIO25, // Dedicated Pad for gpio_gpio
  inout GPIO26, // Dedicated Pad for gpio_gpio
  inout GPIO27, // Dedicated Pad for gpio_gpio
  inout GPIO28, // Dedicated Pad for gpio_gpio
  inout GPIO29, // Dedicated Pad for gpio_gpio
  inout GPIO30, // Dedicated Pad for gpio_gpio
  inout GPIO31, // Dedicated Pad for gpio_gpio
  inout SOC_GPI0, // Dedicated Pad for soc_proxy_soc_gpi
  inout SOC_GPI1, // Dedicated Pad for soc_proxy_soc_gpi
  inout SOC_GPI2, // Dedicated Pad for soc_proxy_soc_gpi
  inout SOC_GPI3, // Dedicated Pad for soc_proxy_soc_gpi
  inout SOC_GPI4, // Dedicated Pad for soc_proxy_soc_gpi
  inout SOC_GPI5, // Dedicated Pad for soc_proxy_soc_gpi
  inout SOC_GPI6, // Dedicated Pad for soc_proxy_soc_gpi
  inout SOC_GPI7, // Dedicated Pad for soc_proxy_soc_gpi
  inout SOC_GPI8, // Dedicated Pad for soc_proxy_soc_gpi
  inout SOC_GPI9, // Dedicated Pad for soc_proxy_soc_gpi
  inout SOC_GPI10, // Dedicated Pad for soc_proxy_soc_gpi
  inout SOC_GPI11, // Dedicated Pad for soc_proxy_soc_gpi
  inout SOC_GPO0, // Dedicated Pad for soc_proxy_soc_gpo
  inout SOC_GPO1, // Dedicated Pad for soc_proxy_soc_gpo
  inout SOC_GPO2, // Dedicated Pad for soc_proxy_soc_gpo
  inout SOC_GPO3, // Dedicated Pad for soc_proxy_soc_gpo
  inout SOC_GPO4, // Dedicated Pad for soc_proxy_soc_gpo
  inout SOC_GPO5, // Dedicated Pad for soc_proxy_soc_gpo
  inout SOC_GPO6, // Dedicated Pad for soc_proxy_soc_gpo
  inout SOC_GPO7, // Dedicated Pad for soc_proxy_soc_gpo
  inout SOC_GPO8, // Dedicated Pad for soc_proxy_soc_gpo
  inout SOC_GPO9, // Dedicated Pad for soc_proxy_soc_gpo
  inout SOC_GPO10, // Dedicated Pad for soc_proxy_soc_gpo
  inout SOC_GPO11, // Dedicated Pad for soc_proxy_soc_gpo
  inout IO_CLK, // Manual Pad
  inout POR_BUTTON_N, // Manual Pad
  inout IO_CLKOUT, // Manual Pad
  inout IO_TRIGGER, // Manual Pad

  // Muxed Pads
  inout MIO0, // MIO Pad 0
  inout MIO1, // MIO Pad 1
  inout MIO2, // MIO Pad 2
  inout MIO3, // MIO Pad 3
  inout MIO4, // MIO Pad 4
  inout MIO5, // MIO Pad 5
  inout MIO6, // MIO Pad 6
  inout MIO7, // MIO Pad 7
  inout MIO8, // MIO Pad 8
  inout MIO9, // MIO Pad 9
  inout MIO10, // MIO Pad 10
  inout MIO11  // MIO Pad 11
);

  import top_darjeeling_pkg::*;
  import prim_pad_wrapper_pkg::*;

  ////////////////////////////
  // Special Signal Indices //
  ////////////////////////////

  localparam int Tap0PadIdx = 0;
  localparam int Tap1PadIdx = 1;
  localparam int Dft0PadIdx = 2;
  localparam int Dft1PadIdx = 3;
  localparam int TckPadIdx = 4;
  localparam int TmsPadIdx = 5;
  localparam int TrstNPadIdx = 6;
  localparam int TdiPadIdx = 7;
  localparam int TdoPadIdx = 8;

  // DFT and Debug signal positions in the pinout.
  localparam pinmux_pkg::target_cfg_t PinmuxTargetCfg = '{
    tck_idx:           TckPadIdx,
    tms_idx:           TmsPadIdx,
    trst_idx:          TrstNPadIdx,
    tdi_idx:           TdiPadIdx,
    tdo_idx:           TdoPadIdx,
    tap_strap0_idx:    Tap0PadIdx,
    tap_strap1_idx:    Tap1PadIdx,
    dft_strap0_idx:    Dft0PadIdx,
    dft_strap1_idx:    Dft1PadIdx,
    // TODO: check whether there is a better way to pass these USB-specific params
    // The use of these indexes is gated behind a parameter, but to synthesize they
    // need to exist even if the code-path is never used (pinmux.sv:UsbWkupModuleEn).
    // Hence, set to zero.
    usb_dp_idx:        0,
    usb_dn_idx:        0,
    usb_sense_idx:     0,
    // Pad types for attribute WARL behavior
    dio_pad_type: {
      BidirStd, // DIO soc_proxy_soc_gpo
      BidirStd, // DIO soc_proxy_soc_gpo
      BidirStd, // DIO soc_proxy_soc_gpo
      BidirStd, // DIO soc_proxy_soc_gpo
      BidirStd, // DIO soc_proxy_soc_gpo
      BidirStd, // DIO soc_proxy_soc_gpo
      BidirStd, // DIO soc_proxy_soc_gpo
      BidirStd, // DIO soc_proxy_soc_gpo
      BidirStd, // DIO soc_proxy_soc_gpo
      BidirStd, // DIO soc_proxy_soc_gpo
      BidirStd, // DIO soc_proxy_soc_gpo
      BidirStd, // DIO soc_proxy_soc_gpo
      BidirStd, // DIO uart0_tx
      BidirStd, // DIO spi_host0_csb
      BidirStd, // DIO spi_host0_sck
      InputStd, // DIO soc_proxy_soc_gpi
      InputStd, // DIO soc_proxy_soc_gpi
      InputStd, // DIO soc_proxy_soc_gpi
      InputStd, // DIO soc_proxy_soc_gpi
      InputStd, // DIO soc_proxy_soc_gpi
      InputStd, // DIO soc_proxy_soc_gpi
      InputStd, // DIO soc_proxy_soc_gpi
      InputStd, // DIO soc_proxy_soc_gpi
      InputStd, // DIO soc_proxy_soc_gpi
      InputStd, // DIO soc_proxy_soc_gpi
      InputStd, // DIO soc_proxy_soc_gpi
      InputStd, // DIO soc_proxy_soc_gpi
      InputStd, // DIO uart0_rx
      InputStd, // DIO spi_device_tpm_csb
      InputStd, // DIO spi_device_csb
      InputStd, // DIO spi_device_sck
      BidirStd, // DIO gpio_gpio
      BidirStd, // DIO gpio_gpio
      BidirStd, // DIO gpio_gpio
      BidirStd, // DIO gpio_gpio
      BidirStd, // DIO gpio_gpio
      BidirStd, // DIO gpio_gpio
      BidirStd, // DIO gpio_gpio
      BidirStd, // DIO gpio_gpio
      BidirStd, // DIO gpio_gpio
      BidirStd, // DIO gpio_gpio
      BidirStd, // DIO gpio_gpio
      BidirStd, // DIO gpio_gpio
      BidirStd, // DIO gpio_gpio
      BidirStd, // DIO gpio_gpio
      BidirStd, // DIO gpio_gpio
      BidirStd, // DIO gpio_gpio
      BidirStd, // DIO gpio_gpio
      BidirStd, // DIO gpio_gpio
      BidirStd, // DIO gpio_gpio
      BidirStd, // DIO gpio_gpio
      BidirStd, // DIO gpio_gpio
      BidirStd, // DIO gpio_gpio
      BidirStd, // DIO gpio_gpio
      BidirStd, // DIO gpio_gpio
      BidirStd, // DIO gpio_gpio
      BidirStd, // DIO gpio_gpio
      BidirStd, // DIO gpio_gpio
      BidirStd, // DIO gpio_gpio
      BidirStd, // DIO gpio_gpio
      BidirStd, // DIO gpio_gpio
      BidirStd, // DIO gpio_gpio
      BidirStd, // DIO gpio_gpio
      BidirStd, // DIO i2c0_sda
      BidirStd, // DIO i2c0_scl
      BidirStd, // DIO spi_device_sd
      BidirStd, // DIO spi_device_sd
      BidirStd, // DIO spi_device_sd
      BidirStd, // DIO spi_device_sd
      BidirStd, // DIO spi_host0_sd
      BidirStd, // DIO spi_host0_sd
      BidirStd, // DIO spi_host0_sd
      BidirStd  // DIO spi_host0_sd
    },
    mio_pad_type: {
      BidirStd, // MIO Pad 11
      BidirStd, // MIO Pad 10
      BidirStd, // MIO Pad 9
      BidirStd, // MIO Pad 8
      BidirStd, // MIO Pad 7
      BidirStd, // MIO Pad 6
      BidirStd, // MIO Pad 5
      BidirStd, // MIO Pad 4
      BidirStd, // MIO Pad 3
      BidirStd, // MIO Pad 2
      BidirStd, // MIO Pad 1
      BidirStd  // MIO Pad 0
    },
    // Pad scan roles
    dio_scan_role: {
      scan_role_pkg::DioPadSocGpo11ScanRole, // DIO soc_proxy_soc_gpo
      scan_role_pkg::DioPadSocGpo10ScanRole, // DIO soc_proxy_soc_gpo
      scan_role_pkg::DioPadSocGpo9ScanRole, // DIO soc_proxy_soc_gpo
      scan_role_pkg::DioPadSocGpo8ScanRole, // DIO soc_proxy_soc_gpo
      scan_role_pkg::DioPadSocGpo7ScanRole, // DIO soc_proxy_soc_gpo
      scan_role_pkg::DioPadSocGpo6ScanRole, // DIO soc_proxy_soc_gpo
      scan_role_pkg::DioPadSocGpo5ScanRole, // DIO soc_proxy_soc_gpo
      scan_role_pkg::DioPadSocGpo4ScanRole, // DIO soc_proxy_soc_gpo
      scan_role_pkg::DioPadSocGpo3ScanRole, // DIO soc_proxy_soc_gpo
      scan_role_pkg::DioPadSocGpo2ScanRole, // DIO soc_proxy_soc_gpo
      scan_role_pkg::DioPadSocGpo1ScanRole, // DIO soc_proxy_soc_gpo
      scan_role_pkg::DioPadSocGpo0ScanRole, // DIO soc_proxy_soc_gpo
      scan_role_pkg::DioPadUartTxScanRole, // DIO uart0_tx
      scan_role_pkg::DioPadSpiHostCsLScanRole, // DIO spi_host0_csb
      scan_role_pkg::DioPadSpiHostClkScanRole, // DIO spi_host0_sck
      scan_role_pkg::DioPadSocGpi11ScanRole, // DIO soc_proxy_soc_gpi
      scan_role_pkg::DioPadSocGpi10ScanRole, // DIO soc_proxy_soc_gpi
      scan_role_pkg::DioPadSocGpi9ScanRole, // DIO soc_proxy_soc_gpi
      scan_role_pkg::DioPadSocGpi8ScanRole, // DIO soc_proxy_soc_gpi
      scan_role_pkg::DioPadSocGpi7ScanRole, // DIO soc_proxy_soc_gpi
      scan_role_pkg::DioPadSocGpi6ScanRole, // DIO soc_proxy_soc_gpi
      scan_role_pkg::DioPadSocGpi5ScanRole, // DIO soc_proxy_soc_gpi
      scan_role_pkg::DioPadSocGpi4ScanRole, // DIO soc_proxy_soc_gpi
      scan_role_pkg::DioPadSocGpi3ScanRole, // DIO soc_proxy_soc_gpi
      scan_role_pkg::DioPadSocGpi2ScanRole, // DIO soc_proxy_soc_gpi
      scan_role_pkg::DioPadSocGpi1ScanRole, // DIO soc_proxy_soc_gpi
      scan_role_pkg::DioPadSocGpi0ScanRole, // DIO soc_proxy_soc_gpi
      scan_role_pkg::DioPadUartRxScanRole, // DIO uart0_rx
      scan_role_pkg::DioPadSpiDevTpmCsLScanRole, // DIO spi_device_tpm_csb
      scan_role_pkg::DioPadSpiDevCsLScanRole, // DIO spi_device_csb
      scan_role_pkg::DioPadSpiDevClkScanRole, // DIO spi_device_sck
      scan_role_pkg::DioPadGpio31ScanRole, // DIO gpio_gpio
      scan_role_pkg::DioPadGpio30ScanRole, // DIO gpio_gpio
      scan_role_pkg::DioPadGpio29ScanRole, // DIO gpio_gpio
      scan_role_pkg::DioPadGpio28ScanRole, // DIO gpio_gpio
      scan_role_pkg::DioPadGpio27ScanRole, // DIO gpio_gpio
      scan_role_pkg::DioPadGpio26ScanRole, // DIO gpio_gpio
      scan_role_pkg::DioPadGpio25ScanRole, // DIO gpio_gpio
      scan_role_pkg::DioPadGpio24ScanRole, // DIO gpio_gpio
      scan_role_pkg::DioPadGpio23ScanRole, // DIO gpio_gpio
      scan_role_pkg::DioPadGpio22ScanRole, // DIO gpio_gpio
      scan_role_pkg::DioPadGpio21ScanRole, // DIO gpio_gpio
      scan_role_pkg::DioPadGpio20ScanRole, // DIO gpio_gpio
      scan_role_pkg::DioPadGpio19ScanRole, // DIO gpio_gpio
      scan_role_pkg::DioPadGpio18ScanRole, // DIO gpio_gpio
      scan_role_pkg::DioPadGpio17ScanRole, // DIO gpio_gpio
      scan_role_pkg::DioPadGpio16ScanRole, // DIO gpio_gpio
      scan_role_pkg::DioPadGpio15ScanRole, // DIO gpio_gpio
      scan_role_pkg::DioPadGpio14ScanRole, // DIO gpio_gpio
      scan_role_pkg::DioPadGpio13ScanRole, // DIO gpio_gpio
      scan_role_pkg::DioPadGpio12ScanRole, // DIO gpio_gpio
      scan_role_pkg::DioPadGpio11ScanRole, // DIO gpio_gpio
      scan_role_pkg::DioPadGpio10ScanRole, // DIO gpio_gpio
      scan_role_pkg::DioPadGpio9ScanRole, // DIO gpio_gpio
      scan_role_pkg::DioPadGpio8ScanRole, // DIO gpio_gpio
      scan_role_pkg::DioPadGpio7ScanRole, // DIO gpio_gpio
      scan_role_pkg::DioPadGpio6ScanRole, // DIO gpio_gpio
      scan_role_pkg::DioPadGpio5ScanRole, // DIO gpio_gpio
      scan_role_pkg::DioPadGpio4ScanRole, // DIO gpio_gpio
      scan_role_pkg::DioPadGpio3ScanRole, // DIO gpio_gpio
      scan_role_pkg::DioPadGpio2ScanRole, // DIO gpio_gpio
      scan_role_pkg::DioPadGpio1ScanRole, // DIO gpio_gpio
      scan_role_pkg::DioPadGpio0ScanRole, // DIO gpio_gpio
      scan_role_pkg::DioPadI2cSdaScanRole, // DIO i2c0_sda
      scan_role_pkg::DioPadI2cSclScanRole, // DIO i2c0_scl
      scan_role_pkg::DioPadSpiDevD3ScanRole, // DIO spi_device_sd
      scan_role_pkg::DioPadSpiDevD2ScanRole, // DIO spi_device_sd
      scan_role_pkg::DioPadSpiDevD1ScanRole, // DIO spi_device_sd
      scan_role_pkg::DioPadSpiDevD0ScanRole, // DIO spi_device_sd
      scan_role_pkg::DioPadSpiHostD3ScanRole, // DIO spi_host0_sd
      scan_role_pkg::DioPadSpiHostD2ScanRole, // DIO spi_host0_sd
      scan_role_pkg::DioPadSpiHostD1ScanRole, // DIO spi_host0_sd
      scan_role_pkg::DioPadSpiHostD0ScanRole // DIO spi_host0_sd
    },
    mio_scan_role: {
      scan_role_pkg::MioPadMio11ScanRole,
      scan_role_pkg::MioPadMio10ScanRole,
      scan_role_pkg::MioPadMio9ScanRole,
      scan_role_pkg::MioPadMio8ScanRole,
      scan_role_pkg::MioPadMio7ScanRole,
      scan_role_pkg::MioPadMio6ScanRole,
      scan_role_pkg::MioPadMio5ScanRole,
      scan_role_pkg::MioPadMio4ScanRole,
      scan_role_pkg::MioPadMio3ScanRole,
      scan_role_pkg::MioPadMio2ScanRole,
      scan_role_pkg::MioPadMio1ScanRole,
      scan_role_pkg::MioPadMio0ScanRole
    }
  };

  ////////////////////////
  // Signal definitions //
  ////////////////////////


  pad_attr_t [pinmux_reg_pkg::NMioPads-1:0] mio_attr;
  pad_attr_t [pinmux_reg_pkg::NDioPads-1:0] dio_attr;
  logic [pinmux_reg_pkg::NMioPads-1:0] mio_out;
  logic [pinmux_reg_pkg::NMioPads-1:0] mio_oe;
  logic [pinmux_reg_pkg::NMioPads-1:0] mio_in;
  logic [pinmux_reg_pkg::NMioPads-1:0] mio_in_raw;
  logic [83-1:0] dio_in_raw;
  logic [pinmux_reg_pkg::NDioPads-1:0] dio_out;
  logic [pinmux_reg_pkg::NDioPads-1:0] dio_oe;
  logic [pinmux_reg_pkg::NDioPads-1:0] dio_in;

  logic unused_mio_in_raw;
  logic unused_dio_in_raw;
  assign unused_mio_in_raw = ^mio_in_raw;
  assign unused_dio_in_raw = ^dio_in_raw;

  // Manual pads
  logic manual_in_por_n, manual_out_por_n, manual_oe_por_n;
  logic manual_in_jtag_tck, manual_out_jtag_tck, manual_oe_jtag_tck;
  logic manual_in_jtag_tms, manual_out_jtag_tms, manual_oe_jtag_tms;
  logic manual_in_jtag_tdi, manual_out_jtag_tdi, manual_oe_jtag_tdi;
  logic manual_in_jtag_tdo, manual_out_jtag_tdo, manual_oe_jtag_tdo;
  logic manual_in_jtag_trst_n, manual_out_jtag_trst_n, manual_oe_jtag_trst_n;
  logic manual_in_io_clk, manual_out_io_clk, manual_oe_io_clk;
  logic manual_in_por_button_n, manual_out_por_button_n, manual_oe_por_button_n;
  logic manual_in_io_clkout, manual_out_io_clkout, manual_oe_io_clkout;
  logic manual_in_io_trigger, manual_out_io_trigger, manual_oe_io_trigger;

  pad_attr_t manual_attr_por_n;
  pad_attr_t manual_attr_jtag_tck;
  pad_attr_t manual_attr_jtag_tms;
  pad_attr_t manual_attr_jtag_tdi;
  pad_attr_t manual_attr_jtag_tdo;
  pad_attr_t manual_attr_jtag_trst_n;
  pad_attr_t manual_attr_io_clk;
  pad_attr_t manual_attr_por_button_n;
  pad_attr_t manual_attr_io_clkout;
  pad_attr_t manual_attr_io_trigger;

  /////////////////////////
  // Stubbed pad tie-off //
  /////////////////////////

  // Only signals going to non-custom pads need to be tied off.
  logic [91:0] unused_sig;

  //////////////////////
  // Padring Instance //
  //////////////////////

  ast_pkg::ast_clks_t ast_base_clks;


  padring #(
    // Padring specific counts may differ from pinmux config due
    // to custom, stubbed or added pads.
    .NDioPads(83),
    .NMioPads(12),
    .DioPadType ({
      BidirStd, // IO_TRIGGER
      BidirStd, // IO_CLKOUT
      InputStd, // POR_BUTTON_N
      InputStd, // IO_CLK
      BidirStd, // SOC_GPO11
      BidirStd, // SOC_GPO10
      BidirStd, // SOC_GPO9
      BidirStd, // SOC_GPO8
      BidirStd, // SOC_GPO7
      BidirStd, // SOC_GPO6
      BidirStd, // SOC_GPO5
      BidirStd, // SOC_GPO4
      BidirStd, // SOC_GPO3
      BidirStd, // SOC_GPO2
      BidirStd, // SOC_GPO1
      BidirStd, // SOC_GPO0
      InputStd, // SOC_GPI11
      InputStd, // SOC_GPI10
      InputStd, // SOC_GPI9
      InputStd, // SOC_GPI8
      InputStd, // SOC_GPI7
      InputStd, // SOC_GPI6
      InputStd, // SOC_GPI5
      InputStd, // SOC_GPI4
      InputStd, // SOC_GPI3
      InputStd, // SOC_GPI2
      InputStd, // SOC_GPI1
      InputStd, // SOC_GPI0
      BidirStd, // GPIO31
      BidirStd, // GPIO30
      BidirStd, // GPIO29
      BidirStd, // GPIO28
      BidirStd, // GPIO27
      BidirStd, // GPIO26
      BidirStd, // GPIO25
      BidirStd, // GPIO24
      BidirStd, // GPIO23
      BidirStd, // GPIO22
      BidirStd, // GPIO21
      BidirStd, // GPIO20
      BidirStd, // GPIO19
      BidirStd, // GPIO18
      BidirStd, // GPIO17
      BidirStd, // GPIO16
      BidirStd, // GPIO15
      BidirStd, // GPIO14
      BidirStd, // GPIO13
      BidirStd, // GPIO12
      BidirStd, // GPIO11
      BidirStd, // GPIO10
      BidirStd, // GPIO9
      BidirStd, // GPIO8
      BidirStd, // GPIO7
      BidirStd, // GPIO6
      BidirStd, // GPIO5
      BidirStd, // GPIO4
      BidirStd, // GPIO3
      BidirStd, // GPIO2
      BidirStd, // GPIO1
      BidirStd, // GPIO0
      BidirStd, // I2C_SDA
      BidirStd, // I2C_SCL
      BidirStd, // UART_TX
      InputStd, // UART_RX
      InputStd, // SPI_DEV_TPM_CS_L
      InputStd, // SPI_DEV_CS_L
      InputStd, // SPI_DEV_CLK
      BidirStd, // SPI_DEV_D3
      BidirStd, // SPI_DEV_D2
      BidirStd, // SPI_DEV_D1
      BidirStd, // SPI_DEV_D0
      BidirStd, // SPI_HOST_CS_L
      BidirStd, // SPI_HOST_CLK
      BidirStd, // SPI_HOST_D3
      BidirStd, // SPI_HOST_D2
      BidirStd, // SPI_HOST_D1
      BidirStd, // SPI_HOST_D0
      InputStd, // JTAG_TRST_N
      BidirStd, // JTAG_TDO
      InputStd, // JTAG_TDI
      InputStd, // JTAG_TMS
      InputStd, // JTAG_TCK
      InputStd  // POR_N
    }),
    .MioPadType ({
      BidirStd, // MIO11
      BidirStd, // MIO10
      BidirStd, // MIO9
      BidirStd, // MIO8
      BidirStd, // MIO7
      BidirStd, // MIO6
      BidirStd, // MIO5
      BidirStd, // MIO4
      BidirStd, // MIO3
      BidirStd, // MIO2
      BidirStd, // MIO1
      BidirStd  // MIO0
    })
  ) u_padring (
  // This is only used for scan and DFT purposes
    .clk_scan_i   ( 1'b0                  ),
    .scanmode_i   ( prim_mubi_pkg::MuBi4False ),
    .dio_in_raw_o ( dio_in_raw ),
    // Chip IOs
    .dio_pad_io ({
      IO_TRIGGER,
      IO_CLKOUT,
      POR_BUTTON_N,
      IO_CLK,
      SOC_GPO11,
      SOC_GPO10,
      SOC_GPO9,
      SOC_GPO8,
      SOC_GPO7,
      SOC_GPO6,
      SOC_GPO5,
      SOC_GPO4,
      SOC_GPO3,
      SOC_GPO2,
      SOC_GPO1,
      SOC_GPO0,
      SOC_GPI11,
      SOC_GPI10,
      SOC_GPI9,
      SOC_GPI8,
      SOC_GPI7,
      SOC_GPI6,
      SOC_GPI5,
      SOC_GPI4,
      SOC_GPI3,
      SOC_GPI2,
      SOC_GPI1,
      SOC_GPI0,
      GPIO31,
      GPIO30,
      GPIO29,
      GPIO28,
      GPIO27,
      GPIO26,
      GPIO25,
      GPIO24,
      GPIO23,
      GPIO22,
      GPIO21,
      GPIO20,
      GPIO19,
      GPIO18,
      GPIO17,
      GPIO16,
      GPIO15,
      GPIO14,
      GPIO13,
      GPIO12,
      GPIO11,
      GPIO10,
      GPIO9,
      GPIO8,
      GPIO7,
      GPIO6,
      GPIO5,
      GPIO4,
      GPIO3,
      GPIO2,
      GPIO1,
      GPIO0,
      I2C_SDA,
      I2C_SCL,
      UART_TX,
      UART_RX,
      SPI_DEV_TPM_CS_L,
      SPI_DEV_CS_L,
      SPI_DEV_CLK,
      SPI_DEV_D3,
      SPI_DEV_D2,
      SPI_DEV_D1,
      SPI_DEV_D0,
      SPI_HOST_CS_L,
      SPI_HOST_CLK,
      SPI_HOST_D3,
      SPI_HOST_D2,
      SPI_HOST_D1,
      SPI_HOST_D0,
      JTAG_TRST_N,
      JTAG_TDO,
      JTAG_TDI,
      JTAG_TMS,
      JTAG_TCK,
      POR_N
    }),

    .mio_pad_io ({
      MIO11,
      MIO10,
      MIO9,
      MIO8,
      MIO7,
      MIO6,
      MIO5,
      MIO4,
      MIO3,
      MIO2,
      MIO1,
      MIO0
    }),

    // Core-facing
    .dio_in_o ({
        manual_in_io_trigger,
        manual_in_io_clkout,
        manual_in_por_button_n,
        manual_in_io_clk,
        dio_in[DioSocProxySocGpo11],
        dio_in[DioSocProxySocGpo10],
        dio_in[DioSocProxySocGpo9],
        dio_in[DioSocProxySocGpo8],
        dio_in[DioSocProxySocGpo7],
        dio_in[DioSocProxySocGpo6],
        dio_in[DioSocProxySocGpo5],
        dio_in[DioSocProxySocGpo4],
        dio_in[DioSocProxySocGpo3],
        dio_in[DioSocProxySocGpo2],
        dio_in[DioSocProxySocGpo1],
        dio_in[DioSocProxySocGpo0],
        dio_in[DioSocProxySocGpi11],
        dio_in[DioSocProxySocGpi10],
        dio_in[DioSocProxySocGpi9],
        dio_in[DioSocProxySocGpi8],
        dio_in[DioSocProxySocGpi7],
        dio_in[DioSocProxySocGpi6],
        dio_in[DioSocProxySocGpi5],
        dio_in[DioSocProxySocGpi4],
        dio_in[DioSocProxySocGpi3],
        dio_in[DioSocProxySocGpi2],
        dio_in[DioSocProxySocGpi1],
        dio_in[DioSocProxySocGpi0],
        dio_in[DioGpioGpio31],
        dio_in[DioGpioGpio30],
        dio_in[DioGpioGpio29],
        dio_in[DioGpioGpio28],
        dio_in[DioGpioGpio27],
        dio_in[DioGpioGpio26],
        dio_in[DioGpioGpio25],
        dio_in[DioGpioGpio24],
        dio_in[DioGpioGpio23],
        dio_in[DioGpioGpio22],
        dio_in[DioGpioGpio21],
        dio_in[DioGpioGpio20],
        dio_in[DioGpioGpio19],
        dio_in[DioGpioGpio18],
        dio_in[DioGpioGpio17],
        dio_in[DioGpioGpio16],
        dio_in[DioGpioGpio15],
        dio_in[DioGpioGpio14],
        dio_in[DioGpioGpio13],
        dio_in[DioGpioGpio12],
        dio_in[DioGpioGpio11],
        dio_in[DioGpioGpio10],
        dio_in[DioGpioGpio9],
        dio_in[DioGpioGpio8],
        dio_in[DioGpioGpio7],
        dio_in[DioGpioGpio6],
        dio_in[DioGpioGpio5],
        dio_in[DioGpioGpio4],
        dio_in[DioGpioGpio3],
        dio_in[DioGpioGpio2],
        dio_in[DioGpioGpio1],
        dio_in[DioGpioGpio0],
        dio_in[DioI2c0Sda],
        dio_in[DioI2c0Scl],
        dio_in[DioUart0Tx],
        dio_in[DioUart0Rx],
        dio_in[DioSpiDeviceTpmCsb],
        dio_in[DioSpiDeviceCsb],
        dio_in[DioSpiDeviceSck],
        dio_in[DioSpiDeviceSd3],
        dio_in[DioSpiDeviceSd2],
        dio_in[DioSpiDeviceSd1],
        dio_in[DioSpiDeviceSd0],
        dio_in[DioSpiHost0Csb],
        dio_in[DioSpiHost0Sck],
        dio_in[DioSpiHost0Sd3],
        dio_in[DioSpiHost0Sd2],
        dio_in[DioSpiHost0Sd1],
        dio_in[DioSpiHost0Sd0],
        manual_in_jtag_trst_n,
        manual_in_jtag_tdo,
        manual_in_jtag_tdi,
        manual_in_jtag_tms,
        manual_in_jtag_tck,
        manual_in_por_n
      }),
    .dio_out_i ({
        manual_out_io_trigger,
        manual_out_io_clkout,
        manual_out_por_button_n,
        manual_out_io_clk,
        dio_out[DioSocProxySocGpo11],
        dio_out[DioSocProxySocGpo10],
        dio_out[DioSocProxySocGpo9],
        dio_out[DioSocProxySocGpo8],
        dio_out[DioSocProxySocGpo7],
        dio_out[DioSocProxySocGpo6],
        dio_out[DioSocProxySocGpo5],
        dio_out[DioSocProxySocGpo4],
        dio_out[DioSocProxySocGpo3],
        dio_out[DioSocProxySocGpo2],
        dio_out[DioSocProxySocGpo1],
        dio_out[DioSocProxySocGpo0],
        dio_out[DioSocProxySocGpi11],
        dio_out[DioSocProxySocGpi10],
        dio_out[DioSocProxySocGpi9],
        dio_out[DioSocProxySocGpi8],
        dio_out[DioSocProxySocGpi7],
        dio_out[DioSocProxySocGpi6],
        dio_out[DioSocProxySocGpi5],
        dio_out[DioSocProxySocGpi4],
        dio_out[DioSocProxySocGpi3],
        dio_out[DioSocProxySocGpi2],
        dio_out[DioSocProxySocGpi1],
        dio_out[DioSocProxySocGpi0],
        dio_out[DioGpioGpio31],
        dio_out[DioGpioGpio30],
        dio_out[DioGpioGpio29],
        dio_out[DioGpioGpio28],
        dio_out[DioGpioGpio27],
        dio_out[DioGpioGpio26],
        dio_out[DioGpioGpio25],
        dio_out[DioGpioGpio24],
        dio_out[DioGpioGpio23],
        dio_out[DioGpioGpio22],
        dio_out[DioGpioGpio21],
        dio_out[DioGpioGpio20],
        dio_out[DioGpioGpio19],
        dio_out[DioGpioGpio18],
        dio_out[DioGpioGpio17],
        dio_out[DioGpioGpio16],
        dio_out[DioGpioGpio15],
        dio_out[DioGpioGpio14],
        dio_out[DioGpioGpio13],
        dio_out[DioGpioGpio12],
        dio_out[DioGpioGpio11],
        dio_out[DioGpioGpio10],
        dio_out[DioGpioGpio9],
        dio_out[DioGpioGpio8],
        dio_out[DioGpioGpio7],
        dio_out[DioGpioGpio6],
        dio_out[DioGpioGpio5],
        dio_out[DioGpioGpio4],
        dio_out[DioGpioGpio3],
        dio_out[DioGpioGpio2],
        dio_out[DioGpioGpio1],
        dio_out[DioGpioGpio0],
        dio_out[DioI2c0Sda],
        dio_out[DioI2c0Scl],
        dio_out[DioUart0Tx],
        dio_out[DioUart0Rx],
        dio_out[DioSpiDeviceTpmCsb],
        dio_out[DioSpiDeviceCsb],
        dio_out[DioSpiDeviceSck],
        dio_out[DioSpiDeviceSd3],
        dio_out[DioSpiDeviceSd2],
        dio_out[DioSpiDeviceSd1],
        dio_out[DioSpiDeviceSd0],
        dio_out[DioSpiHost0Csb],
        dio_out[DioSpiHost0Sck],
        dio_out[DioSpiHost0Sd3],
        dio_out[DioSpiHost0Sd2],
        dio_out[DioSpiHost0Sd1],
        dio_out[DioSpiHost0Sd0],
        manual_out_jtag_trst_n,
        manual_out_jtag_tdo,
        manual_out_jtag_tdi,
        manual_out_jtag_tms,
        manual_out_jtag_tck,
        manual_out_por_n
      }),
    .dio_oe_i ({
        manual_oe_io_trigger,
        manual_oe_io_clkout,
        manual_oe_por_button_n,
        manual_oe_io_clk,
        dio_oe[DioSocProxySocGpo11],
        dio_oe[DioSocProxySocGpo10],
        dio_oe[DioSocProxySocGpo9],
        dio_oe[DioSocProxySocGpo8],
        dio_oe[DioSocProxySocGpo7],
        dio_oe[DioSocProxySocGpo6],
        dio_oe[DioSocProxySocGpo5],
        dio_oe[DioSocProxySocGpo4],
        dio_oe[DioSocProxySocGpo3],
        dio_oe[DioSocProxySocGpo2],
        dio_oe[DioSocProxySocGpo1],
        dio_oe[DioSocProxySocGpo0],
        dio_oe[DioSocProxySocGpi11],
        dio_oe[DioSocProxySocGpi10],
        dio_oe[DioSocProxySocGpi9],
        dio_oe[DioSocProxySocGpi8],
        dio_oe[DioSocProxySocGpi7],
        dio_oe[DioSocProxySocGpi6],
        dio_oe[DioSocProxySocGpi5],
        dio_oe[DioSocProxySocGpi4],
        dio_oe[DioSocProxySocGpi3],
        dio_oe[DioSocProxySocGpi2],
        dio_oe[DioSocProxySocGpi1],
        dio_oe[DioSocProxySocGpi0],
        dio_oe[DioGpioGpio31],
        dio_oe[DioGpioGpio30],
        dio_oe[DioGpioGpio29],
        dio_oe[DioGpioGpio28],
        dio_oe[DioGpioGpio27],
        dio_oe[DioGpioGpio26],
        dio_oe[DioGpioGpio25],
        dio_oe[DioGpioGpio24],
        dio_oe[DioGpioGpio23],
        dio_oe[DioGpioGpio22],
        dio_oe[DioGpioGpio21],
        dio_oe[DioGpioGpio20],
        dio_oe[DioGpioGpio19],
        dio_oe[DioGpioGpio18],
        dio_oe[DioGpioGpio17],
        dio_oe[DioGpioGpio16],
        dio_oe[DioGpioGpio15],
        dio_oe[DioGpioGpio14],
        dio_oe[DioGpioGpio13],
        dio_oe[DioGpioGpio12],
        dio_oe[DioGpioGpio11],
        dio_oe[DioGpioGpio10],
        dio_oe[DioGpioGpio9],
        dio_oe[DioGpioGpio8],
        dio_oe[DioGpioGpio7],
        dio_oe[DioGpioGpio6],
        dio_oe[DioGpioGpio5],
        dio_oe[DioGpioGpio4],
        dio_oe[DioGpioGpio3],
        dio_oe[DioGpioGpio2],
        dio_oe[DioGpioGpio1],
        dio_oe[DioGpioGpio0],
        dio_oe[DioI2c0Sda],
        dio_oe[DioI2c0Scl],
        dio_oe[DioUart0Tx],
        dio_oe[DioUart0Rx],
        dio_oe[DioSpiDeviceTpmCsb],
        dio_oe[DioSpiDeviceCsb],
        dio_oe[DioSpiDeviceSck],
        dio_oe[DioSpiDeviceSd3],
        dio_oe[DioSpiDeviceSd2],
        dio_oe[DioSpiDeviceSd1],
        dio_oe[DioSpiDeviceSd0],
        dio_oe[DioSpiHost0Csb],
        dio_oe[DioSpiHost0Sck],
        dio_oe[DioSpiHost0Sd3],
        dio_oe[DioSpiHost0Sd2],
        dio_oe[DioSpiHost0Sd1],
        dio_oe[DioSpiHost0Sd0],
        manual_oe_jtag_trst_n,
        manual_oe_jtag_tdo,
        manual_oe_jtag_tdi,
        manual_oe_jtag_tms,
        manual_oe_jtag_tck,
        manual_oe_por_n
      }),
    .dio_attr_i ({
        manual_attr_io_trigger,
        manual_attr_io_clkout,
        manual_attr_por_button_n,
        manual_attr_io_clk,
        dio_attr[DioSocProxySocGpo11],
        dio_attr[DioSocProxySocGpo10],
        dio_attr[DioSocProxySocGpo9],
        dio_attr[DioSocProxySocGpo8],
        dio_attr[DioSocProxySocGpo7],
        dio_attr[DioSocProxySocGpo6],
        dio_attr[DioSocProxySocGpo5],
        dio_attr[DioSocProxySocGpo4],
        dio_attr[DioSocProxySocGpo3],
        dio_attr[DioSocProxySocGpo2],
        dio_attr[DioSocProxySocGpo1],
        dio_attr[DioSocProxySocGpo0],
        dio_attr[DioSocProxySocGpi11],
        dio_attr[DioSocProxySocGpi10],
        dio_attr[DioSocProxySocGpi9],
        dio_attr[DioSocProxySocGpi8],
        dio_attr[DioSocProxySocGpi7],
        dio_attr[DioSocProxySocGpi6],
        dio_attr[DioSocProxySocGpi5],
        dio_attr[DioSocProxySocGpi4],
        dio_attr[DioSocProxySocGpi3],
        dio_attr[DioSocProxySocGpi2],
        dio_attr[DioSocProxySocGpi1],
        dio_attr[DioSocProxySocGpi0],
        dio_attr[DioGpioGpio31],
        dio_attr[DioGpioGpio30],
        dio_attr[DioGpioGpio29],
        dio_attr[DioGpioGpio28],
        dio_attr[DioGpioGpio27],
        dio_attr[DioGpioGpio26],
        dio_attr[DioGpioGpio25],
        dio_attr[DioGpioGpio24],
        dio_attr[DioGpioGpio23],
        dio_attr[DioGpioGpio22],
        dio_attr[DioGpioGpio21],
        dio_attr[DioGpioGpio20],
        dio_attr[DioGpioGpio19],
        dio_attr[DioGpioGpio18],
        dio_attr[DioGpioGpio17],
        dio_attr[DioGpioGpio16],
        dio_attr[DioGpioGpio15],
        dio_attr[DioGpioGpio14],
        dio_attr[DioGpioGpio13],
        dio_attr[DioGpioGpio12],
        dio_attr[DioGpioGpio11],
        dio_attr[DioGpioGpio10],
        dio_attr[DioGpioGpio9],
        dio_attr[DioGpioGpio8],
        dio_attr[DioGpioGpio7],
        dio_attr[DioGpioGpio6],
        dio_attr[DioGpioGpio5],
        dio_attr[DioGpioGpio4],
        dio_attr[DioGpioGpio3],
        dio_attr[DioGpioGpio2],
        dio_attr[DioGpioGpio1],
        dio_attr[DioGpioGpio0],
        dio_attr[DioI2c0Sda],
        dio_attr[DioI2c0Scl],
        dio_attr[DioUart0Tx],
        dio_attr[DioUart0Rx],
        dio_attr[DioSpiDeviceTpmCsb],
        dio_attr[DioSpiDeviceCsb],
        dio_attr[DioSpiDeviceSck],
        dio_attr[DioSpiDeviceSd3],
        dio_attr[DioSpiDeviceSd2],
        dio_attr[DioSpiDeviceSd1],
        dio_attr[DioSpiDeviceSd0],
        dio_attr[DioSpiHost0Csb],
        dio_attr[DioSpiHost0Sck],
        dio_attr[DioSpiHost0Sd3],
        dio_attr[DioSpiHost0Sd2],
        dio_attr[DioSpiHost0Sd1],
        dio_attr[DioSpiHost0Sd0],
        manual_attr_jtag_trst_n,
        manual_attr_jtag_tdo,
        manual_attr_jtag_tdi,
        manual_attr_jtag_tms,
        manual_attr_jtag_tck,
        manual_attr_por_n
      }),

    .mio_in_o (mio_in[11:0]),
    .mio_out_i (mio_out[11:0]),
    .mio_oe_i (mio_oe[11:0]),
    .mio_attr_i (mio_attr[11:0]),
    .mio_in_raw_o (mio_in_raw[11:0])
  );




  //////////////////////////////////
  // AST - Common for all targets //
  //////////////////////////////////

  // pwrmgr interface
  pwrmgr_pkg::pwr_ast_req_t base_ast_pwr;
  pwrmgr_pkg::pwr_ast_rsp_t ast_base_pwr;
  pwrmgr_pkg::pwr_boot_status_t pwrmgr_boot_status;

  // assorted ast status
  ast_pkg::ast_pwst_t ast_pwst;

  // TLUL interface
  tlul_pkg::tl_h2d_t base_ast_bus;
  tlul_pkg::tl_d2h_t ast_base_bus;

  // synchronization clocks / rests
  clkmgr_pkg::clkmgr_out_t clkmgr_aon_clocks;
  rstmgr_pkg::rstmgr_out_t rstmgr_aon_resets;

  // external clock
  logic ext_clk;

  // monitored clock
  logic sck_monitor;

  // debug policy bus
  soc_dbg_ctrl_pkg::soc_dbg_policy_t soc_dbg_policy_bus;

  // observe interface
  logic [7:0] otp_obs;
  ast_pkg::ast_obs_ctrl_t obs_ctrl;

  // otp power sequence
  otp_macro_pkg::otp_ast_req_t otp_macro_pwr_seq;
  otp_macro_pkg::otp_ast_rsp_t otp_macro_pwr_seq_h;

  // OTP DFT configuration
  otp_macro_pkg::otp_cfg_t otp_cfg;
  assign otp_cfg = otp_macro_pkg::OTP_CFG_DEFAULT;

  // entropy source interface
  logic es_rng_enable, es_rng_valid;
  logic [ast_pkg::EntropyStreams-1:0] es_rng_bit;

  // alerts interface
  ast_pkg::ast_alert_rsp_t ast_alert_rsp;
  ast_pkg::ast_alert_req_t ast_alert_req;
  assign ast_alert_rsp = '0;

  // clock bypass req/ack
  prim_mubi_pkg::mubi4_t io_clk_byp_req;
  prim_mubi_pkg::mubi4_t io_clk_byp_ack;
  prim_mubi_pkg::mubi4_t all_clk_byp_req;
  prim_mubi_pkg::mubi4_t all_clk_byp_ack;
  prim_mubi_pkg::mubi4_t hi_speed_sel;
  prim_mubi_pkg::mubi4_t div_step_down_req;

  // DFT connections
  logic scan_en;
  lc_ctrl_pkg::lc_tx_t lc_dft_en;

  // Jitter enable
  prim_mubi_pkg::mubi4_t jen;

  // reset domain connections
  import rstmgr_pkg::PowerDomains;
  import rstmgr_pkg::DomainAonSel;
  import rstmgr_pkg::Domain0Sel;

  // Memory configuration connections
  ast_pkg::spm_rm_t ast_ram_1p_cfg;
  ast_pkg::spm_rm_t ast_rf_cfg;
  ast_pkg::spm_rm_t ast_rom_cfg;
  ast_pkg::dpm_rm_t ast_ram_2p_fcfg;
  ast_pkg::dpm_rm_t ast_ram_2p_lcfg;

  // conversion from ast structure to memory centric structures
  prim_ram_1p_pkg::ram_1p_cfg_t ram_1p_cfg;
  assign ram_1p_cfg = '{
    ram_cfg: '{
                test:   ast_ram_1p_cfg.test,
                cfg_en: ast_ram_1p_cfg.marg_en,
                cfg:    ast_ram_1p_cfg.marg
              },
    rf_cfg:  '{
                test:   ast_rf_cfg.test,
                cfg_en: ast_rf_cfg.marg_en,
                cfg:    ast_rf_cfg.marg
              }
  };

  logic unused_usb_ram_2p_cfg;
  assign unused_usb_ram_2p_cfg = ^{ast_ram_2p_fcfg.marg_en_a,
                                   ast_ram_2p_fcfg.marg_a,
                                   ast_ram_2p_fcfg.test_a,
                                   ast_ram_2p_fcfg.marg_en_b,
                                   ast_ram_2p_fcfg.marg_b,
                                   ast_ram_2p_fcfg.test_b};

  // this maps as follows:
  // assign spi_ram_2p_cfg = {10'h000, ram_2p_cfg_i.a_ram_lcfg, ram_2p_cfg_i.b_ram_lcfg};
  prim_ram_2p_pkg::ram_2p_cfg_t spi_ram_2p_cfg;
  assign spi_ram_2p_cfg = '{
    a_ram_lcfg: '{
                   test:   ast_ram_2p_lcfg.test_a,
                   cfg_en: ast_ram_2p_lcfg.marg_en_a,
                   cfg:    ast_ram_2p_lcfg.marg_a
                 },
    b_ram_lcfg: '{
                   test:   ast_ram_2p_lcfg.test_b,
                   cfg_en: ast_ram_2p_lcfg.marg_en_b,
                   cfg:    ast_ram_2p_lcfg.marg_b
                 },
    default: '0
  };

  prim_rom_pkg::rom_cfg_t rom_ctrl0_cfg;
  prim_rom_pkg::rom_cfg_t rom_ctrl1_cfg;

  assign rom_ctrl0_cfg = '{
    test: ast_rom_cfg.test,
    cfg_en: ast_rom_cfg.marg_en,
    cfg: ast_rom_cfg.marg
  };
  assign rom_ctrl1_cfg = '{
    test: ast_rom_cfg.test,
    cfg_en: ast_rom_cfg.marg_en,
    cfg: ast_rom_cfg.marg
  };

  //////////////////////////////////
  // AST - Custom for targets     //
  //////////////////////////////////


  assign ast_base_pwr.main_pok = ast_pwst.main_pok;

  logic [rstmgr_pkg::PowerDomains-1:0] por_n;
  assign por_n = {ast_pwst.main_pok, ast_pwst.aon_pok};

  // TODO: Hook this up when FPGA pads are updated
  assign ext_clk = '0;
  assign pad2ast = '0;

  logic clk_main, clk_usb_48mhz, clk_aon, rst_n, srst_n;
  clkgen_xil7series # (
    .AddClkBuf(0)
  ) clkgen (
    .clk_i(manual_in_io_clk),
    .rst_ni(manual_in_por_n),
    .srst_ni(srst_n),
    .clk_main_o(clk_main),
    .clk_48MHz_o(clk_usb_48mhz),
    .clk_aon_o(clk_aon),
    .rst_no(rst_n)
  );

  logic [31:0] fpga_info;
  usr_access_xil7series u_info (
    .info_o(fpga_info)
  );

  ast_pkg::clks_osc_byp_t clks_osc_byp;
  assign clks_osc_byp = '{
    usb: clk_usb_48mhz,
    sys: clk_main,
    io:  clk_main,
    aon: clk_aon
  };


  prim_mubi_pkg::mubi4_t ast_init_done;

  ast #(
    .AdcChannels(ast_pkg::AdcChannels),
    .AdcDataWidth(ast_pkg::AdcDataWidth),
    .UsbCalibWidth(ast_pkg::UsbCalibWidth),
    .Ast2PadOutWidth(ast_pkg::Ast2PadOutWidth),
    .Pad2AstInWidth(ast_pkg::Pad2AstInWidth)
  ) u_ast (
    // external POR
    .por_ni                ( rst_n ),

    // USB IO Pull-up Calibration Setting
    .usb_io_pu_cal_o       ( ),

    // clocks' oschillator bypass for FPGA
    .clk_osc_byp_i         ( clks_osc_byp ),

    // adc
    .adc_a0_ai             ( '0 ),
    .adc_a1_ai             ( '0 ),

    // Direct short to PAD
    .ast2pad_t0_ao         (  ),
    .ast2pad_t1_ao         (  ),

    // clocks and resets supplied for detection
    .sns_clks_i            ( clkmgr_aon_clocks    ),
    .sns_rsts_i            ( rstmgr_aon_resets    ),
    .sns_spi_ext_clk_i     ( sck_monitor          ),
    // tlul
    .tl_i                  ( base_ast_bus ),
    .tl_o                  ( ast_base_bus ),
    // init done indication
    .ast_init_done_o       ( ast_init_done ),
    // buffered clocks & resets
    .clk_ast_tlul_i (clkmgr_aon_clocks.clk_io_div4_infra),
    .clk_ast_adc_i (clkmgr_aon_clocks.clk_aon_peri),
    .clk_ast_alert_i (clkmgr_aon_clocks.clk_io_div4_secure),
    .clk_ast_rng_i (clkmgr_aon_clocks.clk_main_secure),
    .rst_ast_tlul_ni (rstmgr_aon_resets.rst_lc_io_div4_n[rstmgr_pkg::Domain0Sel]),
    .rst_ast_adc_ni (rstmgr_aon_resets.rst_lc_aon_n[rstmgr_pkg::DomainAonSel]),
    .rst_ast_alert_ni (rstmgr_aon_resets.rst_lc_io_div4_n[rstmgr_pkg::Domain0Sel]),
    .rst_ast_rng_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel]),
    .clk_ast_ext_i         ( ext_clk ),

    // pok test for FPGA
    .vcc_supp_i            ( 1'b1 ),
    .vcaon_supp_i          ( 1'b1 ),
    .vcmain_supp_i         ( 1'b1 ),
    .vioa_supp_i           ( 1'b1 ),
    .viob_supp_i           ( 1'b1 ),
    // pok
    .ast_pwst_o            ( ast_pwst ),
    .ast_pwst_h_o          ( ),
    // main regulator
    .main_env_iso_en_i     ( base_ast_pwr.pwr_clamp_env ),
    .main_pd_ni            ( base_ast_pwr.main_pd_n ),
    // pdm control (flash)/otp
    .flash_power_down_h_o  ( ),
    .flash_power_ready_h_o ( ),
    .otp_power_seq_i       ( otp_macro_pwr_seq ),
    .otp_power_seq_h_o     ( otp_macro_pwr_seq_h ),
    // system source clock
    .clk_src_sys_en_i      ( base_ast_pwr.core_clk_en ),
    // need to add function in clkmgr
    .clk_src_sys_jen_i     ( jen ),
    .clk_src_sys_o         ( ast_base_clks.clk_sys  ),
    .clk_src_sys_val_o     ( ast_base_pwr.core_clk_val ),
    // aon source clock
    .clk_src_aon_o         ( ast_base_clks.clk_aon ),
    .clk_src_aon_val_o     ( ast_base_pwr.slow_clk_val ),
    // io source clock
    .clk_src_io_en_i       ( base_ast_pwr.io_clk_en ),
    .clk_src_io_o          ( ast_base_clks.clk_io ),
    .clk_src_io_val_o      ( ast_base_pwr.io_clk_val ),
    .clk_src_io_48m_o      ( div_step_down_req ),
    // usb source clock
    .usb_ref_pulse_i       ( '0 ),
    .usb_ref_val_i         ( '0 ),
    .clk_src_usb_en_i      ( '0 ),
    .clk_src_usb_o         (    ),
    .clk_src_usb_val_o     (    ),
    // adc
    .adc_pd_i              ( '0 ),
    .adc_chnsel_i          ( '0 ),
    .adc_d_o               (    ),
    .adc_d_val_o           (    ),
    // rng
    .rng_en_i              ( es_rng_enable ),
    .rng_fips_i            ( es_rng_fips   ),
    .rng_val_o             ( es_rng_valid  ),
    .rng_b_o               ( es_rng_bit    ),
    // alerts
    .alert_rsp_i           ( ast_alert_rsp  ),
    .alert_req_o           ( ast_alert_req  ),
    // dft
    .lc_dft_en_i           ( lc_dft_en        ),
    .fla_obs_i             ( '0 ),
    .usb_obs_i             ( '0 ),
    .otp_obs_i             ( otp_obs ),
    .otm_obs_i             ( '0 ),
    .obs_ctrl_o            ( obs_ctrl ),
    // pinmux related
    .padmux2ast_i          ( '0         ),
    .ast2padmux_o          (            ),
    .ext_freq_is_96m_i     ( hi_speed_sel ),
    .all_clk_byp_req_i     ( all_clk_byp_req  ),
    .all_clk_byp_ack_o     ( all_clk_byp_ack  ),
    .io_clk_byp_req_i      ( io_clk_byp_req   ),
    .io_clk_byp_ack_o      ( io_clk_byp_ack   ),
    .flash_bist_en_o       ( ),
    // Memory configuration connections
    .dpram_rmf_o           ( ast_ram_2p_fcfg ),
    .dpram_rml_o           ( ast_ram_2p_lcfg ),
    .spram_rm_o            ( ast_ram_1p_cfg  ),
    .sprgf_rm_o            ( ast_rf_cfg      ),
    .sprom_rm_o            ( ast_rom_cfg     ),
    // scan
    .dft_scan_md_o         ( scanmode ),
    .scan_shift_en_o       ( scan_en ),
    .scan_reset_no         ( scan_rst_n )
  );

  //////////////////
  // TAP Instance //
  //////////////////

  tlul_pkg::tl_h2d_t dmi_h2d;
  tlul_pkg::tl_d2h_t dmi_d2h;
  jtag_pkg::jtag_req_t jtag_req;
  jtag_pkg::jtag_rsp_t jtag_rsp;

  assign jtag_req.tck    = manual_in_jtag_tck;
  assign jtag_req.tms    = manual_in_jtag_tms;
  assign jtag_req.trst_n = manual_in_jtag_trst_n;
  assign jtag_req.tdi    = manual_in_jtag_tdi;

  assign manual_out_jtag_tck     = '0;
  assign manual_out_jtag_tms     = '0;
  assign manual_out_jtag_trst_n  = '0;
  assign manual_out_jtag_tdi     = '0;
  assign manual_oe_jtag_tck      = '0;
  assign manual_oe_jtag_tms      = '0;
  assign manual_oe_jtag_trst_n   = '0;
  assign manual_oe_jtag_tdi      = '0;
  assign manual_attr_jtag_tck    = '0;
  assign manual_attr_jtag_tms    = '0;
  assign manual_attr_jtag_trst_n = '0;
  assign manual_attr_jtag_tdi    = '0;

  assign manual_out_jtag_tdo     = jtag_rsp.tdo;
  assign manual_oe_jtag_tdo      = jtag_rsp.tdo_oe;
  assign manual_attr_jtag_tdo    = '0;

  logic unused_manual_jtag_sigs;
  assign unused_manual_jtag_sigs = ^{
    manual_in_jtag_tdo
  };

  tlul_jtag_dtm #(
    .IdcodeValue(jtag_id_pkg::LC_DM_COMBINED_JTAG_IDCODE),
    // Notes:
    // - one RV_DM instance uses 9bits
    // - our crossbar tooling expects individual IPs to be spaced apart by 12bits at the moment
    // - the DMI address shifted through jtag is a word address and hence 2bits smaller than this
    // - setting this to 18bits effectively gives us 2^6 = 64 addressable 12bit ranges
    .NumDmiByteAbits(18)
  ) u_tlul_jtag_dtm (
    .clk_i      (clkmgr_aon_clocks.clk_main_infra),
    .rst_ni     (rstmgr_aon_resets.rst_sys_n[rstmgr_pkg::Domain0Sel]),
    .jtag_i     (jtag_req),
    .jtag_o     (jtag_rsp),
    .scan_rst_ni(scan_rst_n),
    .scanmode_i (scanmode),
    .tl_h2d_o   (dmi_h2d),
    .tl_d2h_i   (dmi_d2h)
  );

  // TODO: Resolve this and wire it up.
  tlul_pkg::tl_h2d_t ctn_misc_tl_h2d_i;
  assign ctn_misc_tl_h2d_i = tlul_pkg::TL_H2D_DEFAULT;
  tlul_pkg::tl_d2h_t ctn_misc_tl_d2h_o;

  // TODO: Over/ride/ all access range checks for now.
  prim_mubi_pkg::mubi8_t ac_range_check_overwrite_i;
  assign ac_range_check_overwrite_i = prim_mubi_pkg::MuBi8True;

  // TODO: External RACL error input.
  top_racl_pkg::racl_error_log_t ext_racl_error;
  assign ext_racl_error = '0;

  ////////////////
  // CTN M-to-1 //
  ////////////////

  tlul_pkg::tl_h2d_t ctn_tl_h2d[2];
  tlul_pkg::tl_d2h_t ctn_tl_d2h[2];
  //TODO: Resolve this and wire it up.
  assign ctn_tl_h2d[1] = tlul_pkg::TL_H2D_DEFAULT;

  tlul_pkg::tl_h2d_t ctn_sm1_to_s1n_tl_h2d;
  tlul_pkg::tl_d2h_t ctn_sm1_to_s1n_tl_d2h;

  tlul_socket_m1 #(
    .M         (2),
    .HReqPass  ({2{1'b1}}),
    .HRspPass  ({2{1'b1}}),
    .HReqDepth ({2{4'd0}}),
    .HRspDepth ({2{4'd0}}),
    .DReqPass  (1'b1),
    .DRspPass  (1'b1),
    .DReqDepth (4'd0),
    .DRspDepth (4'd0)
  ) u_ctn_sm1 (
    .clk_i  (clkmgr_aon_clocks.clk_main_infra),
    .rst_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel]),
    .tl_h_i (ctn_tl_h2d),
    .tl_h_o (ctn_tl_d2h),
    .tl_d_o (ctn_sm1_to_s1n_tl_h2d),
    .tl_d_i (ctn_sm1_to_s1n_tl_d2h)
  );

  ////////////////////////////////////////////
  // CTN Address decoding and SRAM Instance //
  ////////////////////////////////////////////

  localparam int CtnSramDw = top_pkg::TL_DW + tlul_pkg::DataIntgWidth;

  tlul_pkg::tl_h2d_t ctn_s1n_tl_h2d[1];
  tlul_pkg::tl_d2h_t ctn_s1n_tl_d2h[1];

  // Steering signal for address decoding.
  logic [0:0] ctn_dev_sel_s1n;

  logic sram_req, sram_we, sram_rvalid;
  logic [top_pkg::CtnSramAw-1:0] sram_addr;
  logic [CtnSramDw-1:0] sram_wdata, sram_wmask, sram_rdata;

  // Steering of requests.
  // Addresses leaving the RoT through the CTN port are mapped to an internal 1G address space of
  // 0x4000_0000 - 0x8000_0000. However, the CTN RAM only covers a 1MB region inside that space,
  // and hence additional decoding and steering logic is needed here.
  // TODO: this should in the future be replaced by an automatically generated crossbar.
  always_comb begin
    // Default steering to generate error response if address is not within the range
    ctn_dev_sel_s1n = 1'b1;
    // Steering to CTN SRAM.
    if ((ctn_sm1_to_s1n_tl_h2d.a_address & ~(TOP_DARJEELING_RAM_CTN_SIZE_BYTES-1)) ==
        (TOP_DARJEELING_RAM_CTN_BASE_ADDR - TOP_DARJEELING_CTN_BASE_ADDR)) begin
      ctn_dev_sel_s1n = 1'd0;
    end
  end

  tlul_socket_1n #(
    .HReqDepth (4'h0),
    .HRspDepth (4'h0),
    .DReqDepth (8'h0),
    .DRspDepth (8'h0),
    .N         (1)
  ) u_ctn_s1n (
    .clk_i        (clkmgr_aon_clocks.clk_main_infra),
    .rst_ni       (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel]),
    .tl_h_i       (ctn_sm1_to_s1n_tl_h2d),
    .tl_h_o       (ctn_sm1_to_s1n_tl_d2h),
    .tl_d_o       (ctn_s1n_tl_h2d),
    .tl_d_i       (ctn_s1n_tl_d2h),
    .dev_select_i (ctn_dev_sel_s1n)
  );

  tlul_adapter_sram #(
    .SramAw(top_pkg::CtnSramAw),
    .SramDw(CtnSramDw - tlul_pkg::DataIntgWidth),
    .Outstanding(2),
    .ByteAccess(1),
    .CmdIntgCheck(1),
    .EnableRspIntgGen(1),
    .EnableDataIntgGen(0),
    .EnableDataIntgPt(1),
    .SecFifoPtr      (0)
  ) u_tlul_adapter_sram_ctn (
    .clk_i       (clkmgr_aon_clocks.clk_main_infra),
    .rst_ni      (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel]),
    .tl_i        (ctn_s1n_tl_h2d[0]),
    .tl_o        (ctn_s1n_tl_d2h[0]),
    // Ifetch is explicitly allowed
    .en_ifetch_i (prim_mubi_pkg::MuBi4True),
    .req_o       (sram_req),
    .req_type_o  (),
    // SRAM can always accept a request.
    .gnt_i       (1'b1),
    .we_o        (sram_we),
    .addr_o      (sram_addr),
    .wdata_o     (sram_wdata),
    .wmask_o     (sram_wmask),
    .intg_error_o(),
    .user_rsvd_o (),
    .rdata_i     (sram_rdata),
    .rvalid_i    (sram_rvalid),
    .rerror_i    ('0),
    .compound_txn_in_progress_o(),
    .readback_en_i(prim_mubi_pkg::MuBi4False),
    .readback_error_o(),
    .wr_collision_i(1'b0),
    .write_pending_i(1'b0)
  );

  prim_ram_1p_adv #(
    .Depth(top_pkg::CtnSramDepth),
    .Width(CtnSramDw),
    .DataBitsPerMask(CtnSramDw),
    .EnableECC(0),
    .EnableParity(0),
    .EnableInputPipeline(1),
    .EnableOutputPipeline(1)
  ) u_prim_ram_1p_adv_ctn (
    .clk_i    (clkmgr_aon_clocks.clk_main_infra),
    .rst_ni   (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel]),
    .req_i    (sram_req),
    .write_i  (sram_we),
    .addr_i   (sram_addr),
    .wdata_i  (sram_wdata),
    .wmask_i  (sram_wmask),
    .rdata_o  (sram_rdata),
    .rvalid_o (sram_rvalid),
    // No error detection is enabled inside SRAM.
    // Bus ECC is checked at the consumer side.
    .rerror_o (),
    .cfg_i    (ram_1p_cfg),
    .cfg_rsp_o(),
    .alert_o()
  );



logic unused_signals;
assign unused_signals = ^{pwrmgr_boot_status.clk_status,
                          pwrmgr_boot_status.cpu_fetch_en,
                          pwrmgr_boot_status.lc_done,
                          pwrmgr_boot_status.otp_done,
                          pwrmgr_boot_status.rom_ctrl_status,
                          pwrmgr_boot_status.strap_sampled};

  //////////////////
  // PLL for FPGA //
  //////////////////

  assign manual_attr_io_clk = '0;
  assign manual_out_io_clk = 1'b0;
  assign manual_oe_io_clk = 1'b0;
  assign manual_attr_por_n = '0;
  assign manual_out_por_n = 1'b0;
  assign manual_oe_por_n = 1'b0;
  assign manual_attr_por_button_n = '0;
  assign manual_out_por_button_n = 1'b0;
  assign manual_oe_por_button_n = 1'b0;

  assign srst_n = manual_in_por_button_n;

  // Extend the internal reset request from the power manager.
  //
  // TODO: To model the SoC within FPGA this logic is insufficient; its presence here
  // is to avoid a design that locks up awaiting the deassertion of the signal
  // `soc_rst_req_async_i` in response to an internal reset request.
  logic  internal_request_d, internal_request_q;
  logic  external_reset, count_up;
  logic  [3:0] count;
  always_ff @(posedge ast_base_clks.clk_aon or negedge por_n[0]) begin
    if (!por_n[0]) begin
      external_reset     <= 1'b0;
      internal_request_q <= 1'b0;
      count_up           <= '0;
      count              <= '0;
    end else begin
      internal_request_q <= internal_request_d;
      if (!internal_request_q && internal_request_d) begin
        count_up       <= 1'b1;
        external_reset <= 1;
      end else if (count == 'd8) begin
        count_up       <= 0;
        external_reset <= 0;
        count          <= '0;
      end else if (count_up) begin
        count <= count + 1;
      end
    end
  end

  //////////////////////
  // Top-level design //
  //////////////////////

  // the rst_ni pin only goes to AST
  // the rest of the logic generates reset based on the 'pok' signal.
  // for verilator purposes, make these two the same.
  prim_mubi_pkg::mubi4_t lc_clk_bypass;   // TODO Tim

// TODO: align this with ASIC version to minimize the duplication.
// Also need to add AST simulation and FPGA emulation models for things like entropy source -
// otherwise Verilator / FPGA will hang.
  top_darjeeling #(
    .SecAesMasking(1'b1),
    .SecAesSBoxImpl(aes_pkg::SBoxImplDom),
    .SecAesStartTriggerDelay(320),
    .SecAesAllowForcingMasks(1'b1),
    .KmacEnMasking(0),
    .KmacSwKeyMasked(1),
    .SecKmacCmdDelay(320),
    .SecKmacIdleAcceptSwMsg(1'b1),
    .KeymgrDpeKmacEnMasking(0),
    .CsrngSBoxImpl(aes_pkg::SBoxImplLut),
    .OtbnRegFile(otbn_pkg::RegFileFPGA),
    .SecOtbnMuteUrnd(1'b1),
    .SecOtbnSkipUrndReseedAtStart(1'b1),
    .OtpMacroMemInitFile(OtpMacroMemInitFile),
    .RvCoreIbexPipeLine(1),
    .SramCtrlRetAonInstrExec(0),
    // TODO(opentitan-integrated/issues/251):
    // Enable hashing below once the build infrastructure can
    // load scrambled images on FPGA platforms. The DV can
    // already partially handle it by initializing the 2nd ROM
    // with random data via the backdoor loading interface - it
    // can't load "real" SW images yet since that requires
    // additional build infrastructure.
    .SecRomCtrl1DisableScrambling(1),
    .RomCtrl0BootRomInitFile(BootRomInitFile),
    .RvCoreIbexRegFile(ibex_pkg::RegFileFPGA),
    .RvCoreIbexSecureIbex(0),
    .SramCtrlMainInstrExec(1),
    .PinmuxAonTargetCfg(PinmuxTargetCfg)
  ) top_darjeeling (
    .por_n_i                      ( por_n                 ),
    .clk_main_i                   ( ast_base_clks.clk_sys ),
    .clk_io_i                     ( ast_base_clks.clk_io  ),
    .clk_aon_i                    ( ast_base_clks.clk_aon ),
    .clks_ast_o                   ( clkmgr_aon_clocks     ),
    .clk_main_jitter_en_o         ( jen                   ),
    .rsts_ast_o                   ( rstmgr_aon_resets     ),
    .integrator_id_i              ( '0                    ),
    .sck_monitor_o                ( sck_monitor           ),
    .pwrmgr_ast_req_o             ( base_ast_pwr          ),
    .pwrmgr_ast_rsp_i             ( ast_base_pwr          ),
    .obs_ctrl_i                   ( obs_ctrl              ),
    .io_clk_byp_req_o             ( io_clk_byp_req        ),
    .io_clk_byp_ack_i             ( io_clk_byp_ack        ),
    .all_clk_byp_req_o            ( all_clk_byp_req       ),
    .all_clk_byp_ack_i            ( all_clk_byp_ack       ),
    .hi_speed_sel_o               ( hi_speed_sel          ),
    .div_step_down_req_i          ( div_step_down_req     ),
    .fpga_info_i                  ( fpga_info             ),
    .ast_tl_req_o                 ( base_ast_bus               ),
    .ast_tl_rsp_i                 ( ast_base_bus               ),
    .otp_ctrl_otp_ast_pwr_seq_o   ( otp_ctrl_otp_ast_pwr_seq   ),
    .otp_ctrl_otp_ast_pwr_seq_h_i ( otp_ctrl_otp_ast_pwr_seq_h ),
    .otp_obs_o                    ( otp_obs                    ),
    .otp_cfg_i                    ( otp_cfg                    ),
    .otp_cfg_rsp_o                ( otp_cfg_rsp                ),
    .ctn_tl_h2d_o                 ( ctn_tl_h2d[0]              ),
    .ctn_tl_d2h_i                 ( ctn_tl_d2h[0]              ),
    .ac_range_check_overwrite_i   ( ac_range_check_overwrite_i ),
    .racl_error_i                 ( ext_racl_error             ),
    .soc_gpi_async_o              (                            ),
    .soc_gpo_async_i              ( '0                         ),
    .soc_dbg_policy_bus_o         ( soc_dbg_policy_bus         ),
    .debug_halt_cpu_boot_i        ( '0                         ),
    .dma_sys_req_o                (                            ),
    .dma_sys_rsp_i                ( '0                         ),
    .soc_rst_req_async_i          ( external_reset             ),
    .soc_lsio_trigger_i           ( '0                         ),
    .es_rng_enable_o              ( es_rng_enable              ),
    .es_rng_valid_i               ( es_rng_valid               ),
    .es_rng_bit_i                 ( es_rng_bit                 ),
    .calib_rdy_i                  ( ast_init_done              ),

    // DMI TL-UL
    .dbg_tl_req_i                 ( dmi_h2d                    ),
    .dbg_tl_rsp_o                 ( dmi_d2h                    ),
    // Quasi-static word address for next_dm register value.
    .rv_dm_next_dm_addr_i         ( '0                         ),
    // Multiplexed I/O
    .mio_in_i                     ( mio_in                     ),
    .mio_out_o                    ( mio_out                    ),
    .mio_oe_o                     ( mio_oe                     ),

    // Dedicated I/O
    .dio_in_i                     ( dio_in                     ),
    .dio_out_o                    ( dio_out                    ),
    .dio_oe_o                     ( dio_oe                     ),

    // Pad attributes
    .mio_attr_o                   ( mio_attr                   ),
    .dio_attr_o                   ( dio_attr                   ),

    // Memory attributes
    .rom_ctrl0_cfg_i                           ( '0 ),
    .rom_ctrl1_cfg_i                           ( '0 ),
    .i2c_ram_1p_cfg_i                          ( '0 ),
    .i2c_ram_1p_cfg_rsp_o                      (    ),
    .sram_ctrl_ret_aon_ram_1p_cfg_i            ( '0 ),
    .sram_ctrl_ret_aon_ram_1p_cfg_rsp_o        (    ),
    .sram_ctrl_main_ram_1p_cfg_i               ( '0 ),
    .sram_ctrl_main_ram_1p_cfg_rsp_o           (    ),
    .sram_ctrl_mbox_ram_1p_cfg_i               ( '0 ),
    .sram_ctrl_mbox_ram_1p_cfg_rsp_o           (    ),
    .otbn_imem_ram_1p_cfg_i                    ( '0 ),
    .otbn_imem_ram_1p_cfg_rsp_o                (    ),
    .otbn_dmem_ram_1p_cfg_i                    ( '0 ),
    .otbn_dmem_ram_1p_cfg_rsp_o                (    ),
    .rv_core_ibex_icache_tag_ram_1p_cfg_i      ( '0 ),
    .rv_core_ibex_icache_tag_ram_1p_cfg_rsp_o  (    ),
    .rv_core_ibex_icache_data_ram_1p_cfg_i     ( '0 ),
    .rv_core_ibex_icache_data_ram_1p_cfg_rsp_o (    ),
    .spi_device_ram_2p_cfg_sys2spi_i           ( '0 ),
    .spi_device_ram_2p_cfg_spi2sys_i           ( '0 ),
    .spi_device_ram_2p_cfg_rsp_sys2spi_o       (    ),
    .spi_device_ram_2p_cfg_rsp_spi2sys_o       (    ),

     // DFT signals
    .ast_lc_dft_en_o      ( lc_dft_en                  ),
    .ast_lc_hw_debug_en_o (                            ),
    // DFT signals
    .dft_hold_tap_sel_i ( '0               ),
    .scan_rst_ni        ( 1'b1             ),
    .scan_en_i          ( 1'b0             ),
    .scanmode_i         ( prim_mubi_pkg::MuBi4False )
  );


  /////////////////////////////////////////////////////
  // ChipWhisperer CW310/305 Capture Board Interface //
  /////////////////////////////////////////////////////
  // This is used to interface OpenTitan as a target with a capture board trough the ChipWhisperer
  // 20-pin connector. This is used for SCA/FI experiments only.

  logic unused_inputs;
  assign unused_inputs = manual_in_io_clkout ^ manual_in_io_trigger;

  // Synchronous clock output to capture board.
  assign manual_out_io_clkout = manual_in_io_clk;
  assign manual_oe_io_clkout = 1'b1;

  // Capture trigger.
  // We use the clkmgr_aon_idle signal of the IP of interest to form a precise capture trigger.
  // GPIO[11:10] is used for selecting the IP of interest. The encoding is as follows (see
  // hint_names_e enum in clkmgr_pkg.sv for details).
  //
  // IP              - GPIO[11:10] - Index for clkmgr_aon_idle
  // -------------------------------------------------------------
  //  AES            -   00       -  0
  //  HMAC           -   01       -  1 - not implemented on CW305
  //  KMAC           -   10       -  2 - not implemented on CW305
  //  OTBN           -   11       -  3 - not implemented on CW305
  //
  // GPIO9 is used for gating the selected capture trigger in software. Alternatively, GPIO8
  // can be used to implement a less precise but fully software-controlled capture trigger
  // similar to what can be done on ASIC.
  //
  // Note that on the CW305, GPIO[9,8] are connected to LED[5(Green),7(Red)].

  prim_mubi_pkg::mubi4_t clk_trans_idle, manual_in_io_clk_idle;

  clkmgr_pkg::hint_names_e trigger_sel;
  always_comb begin : trigger_sel_mux
    unique case ({mio_out[MioOutGpioGpio11], mio_out[MioOutGpioGpio10]})
      2'b00:   trigger_sel = clkmgr_pkg::HintMainAes;
      2'b01:   trigger_sel = clkmgr_pkg::HintMainHmac;
      2'b10:   trigger_sel = clkmgr_pkg::HintMainKmac;
      2'b11:   trigger_sel = clkmgr_pkg::HintMainOtbn;
      default: trigger_sel = clkmgr_pkg::HintMainAes;
    endcase;
  end
  assign clk_trans_idle = top_darjeeling.clkmgr_aon_idle[trigger_sel];

  logic clk_io_div4_trigger_hw_en, manual_in_io_clk_trigger_hw_en;
  logic clk_io_div4_trigger_hw_oe, manual_in_io_clk_trigger_hw_oe;
  logic clk_io_div4_trigger_sw_en, manual_in_io_clk_trigger_sw_en;
  logic clk_io_div4_trigger_sw_oe, manual_in_io_clk_trigger_sw_oe;
  assign clk_io_div4_trigger_hw_en = mio_out[MioOutGpioGpio9];
  assign clk_io_div4_trigger_hw_oe = mio_oe[MioOutGpioGpio9];
  assign clk_io_div4_trigger_sw_en = mio_out[MioOutGpioGpio8];
  assign clk_io_div4_trigger_sw_oe = mio_oe[MioOutGpioGpio8];

  // Synchronize signals to manual_in_io_clk.
  prim_flop_2sync #(
    .Width ($bits(clk_trans_idle) + 4)
  ) u_sync_trigger (
    .clk_i (manual_in_io_clk),
    .rst_ni(manual_in_por_n),
    .d_i   ({clk_trans_idle,
             clk_io_div4_trigger_hw_en,
             clk_io_div4_trigger_hw_oe,
             clk_io_div4_trigger_sw_en,
             clk_io_div4_trigger_sw_oe}),
    .q_o   ({manual_in_io_clk_idle,
             manual_in_io_clk_trigger_hw_en,
             manual_in_io_clk_trigger_hw_oe,
             manual_in_io_clk_trigger_sw_en,
             manual_in_io_clk_trigger_sw_oe})
  );

  // Generate the actual trigger signal as trigger_sw OR trigger_hw.
  assign manual_attr_io_trigger = '0;
  assign manual_oe_io_trigger  =
      manual_in_io_clk_trigger_sw_oe | manual_in_io_clk_trigger_hw_oe;
  assign manual_out_io_trigger =
      manual_in_io_clk_trigger_sw_en | (manual_in_io_clk_trigger_hw_en &
          prim_mubi_pkg::mubi4_test_false_strict(manual_in_io_clk_idle));

endmodule : chip_darjeeling_cw310
