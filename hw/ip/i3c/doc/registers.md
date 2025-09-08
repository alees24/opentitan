## Summary

| Name                                                                        | Offset   |   Length | Description                                                                        |
|:----------------------------------------------------------------------------|:---------|---------:|:-----------------------------------------------------------------------------------|
| i3c.[`INTR_STATE`](#intr_state)                                             | 0x0      |        4 | Interrupt State Register                                                           |
| i3c.[`INTR_ENABLE`](#intr_enable)                                           | 0x4      |        4 | Interrupt Enable Register                                                          |
| i3c.[`INTR_TEST`](#intr_test)                                               | 0x8      |        4 | Interrupt Test Register                                                            |
| i3c.[`ALERT_TEST`](#alert_test)                                             | 0xc      |        4 | Alert Test Register                                                                |
| i3c.[`CTRL`](#ctrl)                                                         | 0x10     |        4 | I3C control register                                                               |
| i3c.[`STATUS`](#status)                                                     | 0x14     |        4 | I3C status register                                                                |
| i3c.[`PHY_CONFIG`](#phy_config)                                             | 0x18     |        4 | PHY configuration                                                                  |
| i3c.[`CTRL_TXBUF_CONFIG`](#ctrl_txbuf_config)                               | 0x1c     |        4 | Controller TX Buffer Configuration                                                 |
| i3c.[`CTRL_TXBUF_WPTR`](#ctrl_txbuf_wptr)                                   | 0x20     |        4 | Controller TX Buffer Write Pointer                                                 |
| i3c.[`CTRL_TXBUF_RPTR`](#ctrl_txbuf_rptr)                                   | 0x24     |        4 | Controller TX Buffer Read Pointer                                                  |
| i3c.[`CTRL_RXBUF_CONFIG`](#ctrl_rxbuf_config)                               | 0x28     |        4 | Controller RX Buffer Configuration                                                 |
| i3c.[`CTRL_RXBUF_WPTR`](#ctrl_rxbuf_wptr)                                   | 0x2c     |        4 | Controller RX Buffer Write Pointer                                                 |
| i3c.[`CTRL_RXBUF_RPTR`](#ctrl_rxbuf_rptr)                                   | 0x30     |        4 | Controller RX Buffer Read Pointer                                                  |
| i3c.[`TARG_TXBUF_CONFIG`](#targ_txbuf_config)                               | 0x34     |        4 | Target TX Buffer Configuration                                                     |
| i3c.[`TARG_TXBUF_WPTR`](#targ_txbuf_wptr)                                   | 0x38     |        4 | Target TX Buffer Write Pointer                                                     |
| i3c.[`TARG_TXBUF_RPTR`](#targ_txbuf_rptr)                                   | 0x3c     |        4 | Target TX Buffer Read Pointer                                                      |
| i3c.[`TARG_RXBUF_CONFIG`](#targ_rxbuf_config)                               | 0x40     |        4 | Target RX Buffer Configuration                                                     |
| i3c.[`TARG_RXBUF_WPTR`](#targ_rxbuf_wptr)                                   | 0x44     |        4 | Target RX Buffer Write Pointer                                                     |
| i3c.[`TARG_RXBUF_RPTR`](#targ_rxbuf_rptr)                                   | 0x48     |        4 | Target RX Buffer Read Pointer                                                      |
| i3c.[`BUFFER_CTRL`](#buffer_ctrl)                                           | 0x4c     |        4 | Buffer control                                                                     |
| i3c.[`TARG_ADDR_MATCH`](#targ_addr_match)                                   | 0x50     |        4 | I3C target addresses                                                               |
| i3c.[`TARG_ADDR_MASK`](#targ_addr_mask)                                     | 0x54     |        4 | I3C target address masks                                                           |
| i3c.[`CTRL_CLK_CONFIG`](#ctrl_clk_config)                                   | 0x58     |        4 | Controller clock configuration.                                                    |
| i3c.[`HCI_VERSION`](#hci_version)                                           | 0x100    |        4 | HCI Version.                                                                       |
| i3c.[`HC_CONTROL`](#hc_control)                                             | 0x104    |        4 | Host Controller Control.                                                           |
| i3c.[`CONTROLLER_DEVICE_ADDR`](#controller_device_addr)                     | 0x108    |        4 | Controller Device Address.                                                         |
| i3c.[`HC_CAPABILITIES`](#hc_capabilities)                                   | 0x10c    |        4 | Host Controller Capabilities.                                                      |
| i3c.[`RESET_CONTROL`](#reset_control)                                       | 0x120    |        4 | Reset Control.                                                                     |
| i3c.[`PRESENT_STATE`](#present_state)                                       | 0x124    |        4 | Present State.                                                                     |
| i3c.[`INTR_STATUS`](#intr_status)                                           | 0x128    |        4 | Interrupt Status.                                                                  |
| i3c.[`INTR_STATUS_ENABLE`](#intr_status_enable)                             | 0x12c    |        4 | Interrupt Status Enable.                                                           |
| i3c.[`INTR_SIGNAL_ENABLE`](#intr_signal_enable)                             | 0x130    |        4 | Interrupt Signal Enable.                                                           |
| i3c.[`INTR_FORCE`](#intr_force)                                             | 0x134    |        4 | Interrupt Force.                                                                   |
| i3c.[`DAT_SECTION_OFFSET`](#dat_section_offset)                             | 0x138    |        4 | Device Address Table Section Offset.                                               |
| i3c.[`DCT_SECTION_OFFSET`](#dct_section_offset)                             | 0x13c    |        4 | Device Characteristics Table Section Offset.                                       |
| i3c.[`RING_HEADERS_SECTION_OFFSET`](#ring_headers_section_offset)           | 0x140    |        4 | Ring Headers Section Offset.                                                       |
| i3c.[`PIO_SECTION_OFFSET`](#pio_section_offset)                             | 0x144    |        4 | PIO Section Offset.                                                                |
| i3c.[`EXT_CAPS_SECTION_OFFSET`](#ext_caps_section_offset)                   | 0x148    |        4 | Extended Capabilities Section Offset.                                              |
| i3c.[`INT_CTRL_CMDS_EN`](#int_ctrl_cmds_en)                                 | 0x14c    |        4 | Internal Control Command Subtype Support.                                          |
| i3c.[`IBI_NOTIFY_CTRL`](#ibi_notify_ctrl)                                   | 0x158    |        4 | IBI Notify Control.                                                                |
| i3c.[`IBI_DATA_ABORT_CTRL`](#ibi_data_abort_ctrl)                           | 0x15c    |        4 | IBI Data Abort Control.                                                            |
| i3c.[`DEV_CTX_BASE_LO`](#dev_ctx_base_lo)                                   | 0x160    |        4 | Device Context Base Address Low.                                                   |
| i3c.[`DEV_CTX_BASE_HI`](#dev_ctx_base_hi)                                   | 0x164    |        4 | Device Context Base Address High.                                                  |
| i3c.[`DEV_CTX_SG`](#dev_ctx_sg)                                             | 0x168    |        4 | Device Contxt Scatter-Gather Support.                                              |
| i3c.[`COMMAND_QUEUE_PORT`](#command_queue_port)                             | 0x200    |        4 | Write-only Command Descriptor queue.                                               |
| i3c.[`RESPONSE_QUEUE_PORT`](#response_queue_port)                           | 0x204    |        4 | Read-only Response Descriptor queue.                                               |
| i3c.[`XFER_DATA_PORT`](#xfer_data_port)                                     | 0x208    |        4 | Read/write data transfer port.                                                     |
| i3c.[`IBI_PORT`](#ibi_port)                                                 | 0x20c    |        4 | Read-only IBI queue.                                                               |
| i3c.[`QUEUE_THLD_CTRL`](#queue_thld_ctrl)                                   | 0x210    |        4 | Queue Threshold Control.                                                           |
| i3c.[`DATA_BUFFER_THLD_CTRL`](#data_buffer_thld_ctrl)                       | 0x214    |        4 | Transfer Data Buffer Threshold Control.                                            |
| i3c.[`QUEUE_SIZE`](#queue_size)                                             | 0x218    |        4 | Queue Size.                                                                        |
| i3c.[`ALT_QUEUE_SIZE`](#alt_queue_size)                                     | 0x21c    |        4 | Alternate Queue Size.                                                              |
| i3c.[`PIO_INTR_STATUS`](#pio_intr_status)                                   | 0x220    |        4 | PIO Interrupt Status.                                                              |
| i3c.[`PIO_INTR_STATUS_ENABLE`](#pio_intr_status_enable)                     | 0x224    |        4 | PIO Interrupt Status Enable.                                                       |
| i3c.[`PIO_INTR_SIGNAL_ENABLE`](#pio_intr_signal_enable)                     | 0x228    |        4 | PIO Interrupt Signal Enable.                                                       |
| i3c.[`PIO_INTR_FORCE`](#pio_intr_force)                                     | 0x22c    |        4 | PIO Interrupt Force.                                                               |
| i3c.[`STBY_EXTCAP_HEADER`](#stby_extcap_header)                             | 0x230    |        4 | Standby Controller Extended Capability Header                                      |
| i3c.[`STBY_CR_CONTROL`](#stby_cr_control)                                   | 0x234    |        4 | Standby Controller Control                                                         |
| i3c.[`STBY_CR_DEVICE_ADDR`](#stby_cr_device_addr)                           | 0x238    |        4 | Standby Controller Device Address                                                  |
| i3c.[`STBY_CR_CAPABILITIES`](#stby_cr_capabilities)                         | 0x23c    |        4 | Standby Controller Capabilities                                                    |
| i3c.[`STBY_CR_STATUS`](#stby_cr_status)                                     | 0x240    |        4 | Standby Controller Status                                                          |
| i3c.[`STBY_CR_DEVICE_CHAR`](#stby_cr_device_char)                           | 0x244    |        4 | Standby Controller Device Characteristics                                          |
| i3c.[`STBY_CR_DEVICE_PID_LO`](#stby_cr_device_pid_lo)                       | 0x248    |        4 | Standby Controller PID Low                                                         |
| i3c.[`STBY_CR_INTR_STATUS`](#stby_cr_intr_status)                           | 0x24c    |        4 | Standby Controller Interrupt Status                                                |
| i3c.[`STBY_CR_INTR_SIGNAL_ENABLE`](#stby_cr_intr_signal_enable)             | 0x250    |        4 | Standby Controller Interrupt Signal Enable                                         |
| i3c.[`STBY_CR_INTR_FORCE`](#stby_cr_intr_force)                             | 0x254    |        4 | Standby Controller Interrupt Force                                                 |
| i3c.[`STBY_CR_CCC_CONFIG_GETCAPS`](#stby_cr_ccc_config_getcaps)             | 0x258    |        4 | Standby Controller CCC Auto-Response Config Get Capabilities                       |
| i3c.[`STBY_CR_CCC_CONFIG_RSTACT_PARAMS`](#stby_cr_ccc_config_rstact_params) | 0x25c    |        4 | Standby Controller CCC Auto-Response Config Get Capabilities                       |
| i3c.[`DAT`](#dat)                                                           | 0x300    |      256 | Device Address Table.                                                              |
| i3c.[`DCT`](#dct)                                                           | 0x400    |      512 | Device Characteristics Table.                                                      |
| i3c.[`BUFFER`](#buffer)                                                     | 0x800    |     2048 | Software-managed 2KiB message buffer used for transmitting and receiving messages. |

## INTR_STATE
Interrupt State Register
- Offset: `0x0`
- Reset default: `0x0`
- Reset mask: `0x1`

### Fields

```wavejson
{"reg": [{"name": "hci", "bits": 1, "attr": ["ro"], "rotate": -90}, {"bits": 31}], "config": {"lanes": 1, "fontsize": 10, "vspace": 80}}
```

|  Bits  |  Type  |  Reset  | Name   | Description           |
|:------:|:------:|:-------:|:-------|:----------------------|
|  31:1  |        |         |        | Reserved              |
|   0    |   ro   |   0x0   | hci    | HCI master interrupt. |

## INTR_ENABLE
Interrupt Enable Register
- Offset: `0x4`
- Reset default: `0x0`
- Reset mask: `0x1`

### Fields

```wavejson
{"reg": [{"name": "hci", "bits": 1, "attr": ["rw"], "rotate": -90}, {"bits": 31}], "config": {"lanes": 1, "fontsize": 10, "vspace": 80}}
```

|  Bits  |  Type  |  Reset  | Name   | Description                                                   |
|:------:|:------:|:-------:|:-------|:--------------------------------------------------------------|
|  31:1  |        |         |        | Reserved                                                      |
|   0    |   rw   |   0x0   | hci    | Enable interrupt when [`INTR_STATE.hci`](#intr_state) is set. |

## INTR_TEST
Interrupt Test Register
- Offset: `0x8`
- Reset default: `0x0`
- Reset mask: `0x1`

### Fields

```wavejson
{"reg": [{"name": "hci", "bits": 1, "attr": ["wo"], "rotate": -90}, {"bits": 31}], "config": {"lanes": 1, "fontsize": 10, "vspace": 80}}
```

|  Bits  |  Type  |  Reset  | Name   | Description                                            |
|:------:|:------:|:-------:|:-------|:-------------------------------------------------------|
|  31:1  |        |         |        | Reserved                                               |
|   0    |   wo   |   0x0   | hci    | Write 1 to force [`INTR_STATE.hci`](#intr_state) to 1. |

## ALERT_TEST
Alert Test Register
- Offset: `0xc`
- Reset default: `0x0`
- Reset mask: `0x1`

### Fields

```wavejson
{"reg": [{"name": "fatal_fault", "bits": 1, "attr": ["wo"], "rotate": -90}, {"bits": 31}], "config": {"lanes": 1, "fontsize": 10, "vspace": 130}}
```

|  Bits  |  Type  |  Reset  | Name        | Description                                      |
|:------:|:------:|:-------:|:------------|:-------------------------------------------------|
|  31:1  |        |         |             | Reserved                                         |
|   0    |   wo   |   0x0   | fatal_fault | Write 1 to trigger one alert event of this kind. |

## CTRL
I3C control register
- Offset: `0x10`
- Reset default: `0x0`
- Reset mask: `0xc000010f`

### Fields

```wavejson
{"reg": [{"name": "CTRL_TX_EN", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "CTRL_RX_EN", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "TARG_TX_EN", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "TARG_RX_EN", "bits": 1, "attr": ["rw"], "rotate": -90}, {"bits": 4}, {"name": "HDR_DDR_EN", "bits": 1, "attr": ["rw"], "rotate": -90}, {"bits": 21}, {"name": "CTRL_RESET", "bits": 1, "attr": ["wo"], "rotate": -90}, {"name": "TARG_RESET", "bits": 1, "attr": ["wo"], "rotate": -90}], "config": {"lanes": 1, "fontsize": 10, "vspace": 120}}
```

|  Bits  |  Type  |  Reset  | Name       | Description                         |
|:------:|:------:|:-------:|:-----------|:------------------------------------|
|   31   |   wo   |   0x0   | TARG_RESET | Software reset of Target logic.     |
|   30   |   wo   |   0x0   | CTRL_RESET | Software reset of Controller logic. |
|  29:9  |        |         |            | Reserved                            |
|   8    |   rw   |   0x0   | HDR_DDR_EN | HDR-DDR mode enable                 |
|  7:4   |        |         |            | Reserved                            |
|   3    |   rw   |   0x0   | TARG_RX_EN | Target RX enable                    |
|   2    |   rw   |   0x0   | TARG_TX_EN | Target TX enable                    |
|   1    |   rw   |   0x0   | CTRL_RX_EN | Controller RX enable                |
|   0    |   rw   |   0x0   | CTRL_TX_EN | Controller TX enable                |

## STATUS
I3C status register
- Offset: `0x14`
- Reset default: `0x0`
- Reset mask: `0x1`

### Fields

```wavejson
{"reg": [{"name": "DUMMY", "bits": 1, "attr": ["ro"], "rotate": -90}, {"bits": 31}], "config": {"lanes": 1, "fontsize": 10, "vspace": 80}}
```

|  Bits  |  Type  |  Reset  | Name   | Description                             |
|:------:|:------:|:-------:|:-------|:----------------------------------------|
|  31:1  |        |         |        | Reserved                                |
|   0    |   ro   |    x    | DUMMY  | Dummy field; no status information yet. |

## PHY_CONFIG
PHY configuration
- Offset: `0x18`
- Reset default: `0x3`
- Reset mask: `0x3`

### Fields

```wavejson
{"reg": [{"name": "SCL_HK_EN", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "SDA_HK_EN", "bits": 1, "attr": ["rw"], "rotate": -90}, {"bits": 30}], "config": {"lanes": 1, "fontsize": 10, "vspace": 110}}
```

|  Bits  |  Type  |  Reset  | Name      | Description                 |
|:------:|:------:|:-------:|:----------|:----------------------------|
|  31:2  |        |         |           | Reserved                    |
|   1    |   rw   |   0x1   | SDA_HK_EN | High-keeper enable for SDA. |
|   0    |   rw   |   0x1   | SCL_HK_EN | High-keeper enable for SCL. |

## CTRL_TXBUF_CONFIG
Controller TX Buffer Configuration
- Offset: `0x1c`
- Reset default: `0x607f0000`
- Reset mask: `0x73ff03ff`

### Fields

```wavejson
{"reg": [{"name": "MIN_ADDR", "bits": 10, "attr": ["rw"], "rotate": 0}, {"bits": 6}, {"name": "MAX_ADDR", "bits": 10, "attr": ["rw"], "rotate": 0}, {"bits": 2}, {"name": "SIZE_VAL", "bits": 3, "attr": ["rw"], "rotate": -90}, {"bits": 1}], "config": {"lanes": 1, "fontsize": 10, "vspace": 100}}
```

|  Bits  |  Type  |  Reset  | Name     | Description                                                                                       |
|:------:|:------:|:-------:|:---------|:--------------------------------------------------------------------------------------------------|
|   31   |        |         |          | Reserved                                                                                          |
| 30:28  |   rw   |   0x6   | SIZE_VAL | Size value presented as `QUEUE_SIZE.TX_DATA_BUFFER_SIZE`. This value is log2(size in DWORDs) - 1. |
| 27:26  |        |         |          | Reserved                                                                                          |
| 25:16  |   rw   |  0x7f   | MAX_ADDR | Maximum address for Controller TX use (inclusive).                                                |
| 15:10  |        |         |          | Reserved                                                                                          |
|  9:0   |   rw   |   0x0   | MIN_ADDR | Minimum address for Controller TX use.                                                            |

## CTRL_TXBUF_WPTR
Controller TX Buffer Write Pointer
- Offset: `0x20`
- Reset default: `0x0`
- Reset mask: `0xffff`

### Fields

```wavejson
{"reg": [{"name": "ADDR", "bits": 16, "attr": ["rw"], "rotate": 0}, {"bits": 16}], "config": {"lanes": 1, "fontsize": 10, "vspace": 80}}
```

|  Bits  |  Type  |  Reset  | Name   | Description            |
|:------:|:------:|:-------:|:-------|:-----------------------|
| 31:16  |        |         |        | Reserved               |
|  15:0  |   rw   |   0x0   | ADDR   | Current write address. |

## CTRL_TXBUF_RPTR
Controller TX Buffer Read Pointer
- Offset: `0x24`
- Reset default: `0x0`
- Reset mask: `0xffff`

### Fields

```wavejson
{"reg": [{"name": "ADDR", "bits": 16, "attr": ["rw"], "rotate": 0}, {"bits": 16}], "config": {"lanes": 1, "fontsize": 10, "vspace": 80}}
```

|  Bits  |  Type  |  Reset  | Name   | Description           |
|:------:|:------:|:-------:|:-------|:----------------------|
| 31:16  |        |         |        | Reserved              |
|  15:0  |   rw   |   0x0   | ADDR   | Current read address. |

## CTRL_RXBUF_CONFIG
Controller RX Buffer Configuration
- Offset: `0x28`
- Reset default: `0x60ff0080`
- Reset mask: `0x73ff03ff`

### Fields

```wavejson
{"reg": [{"name": "MIN_ADDR", "bits": 10, "attr": ["rw"], "rotate": 0}, {"bits": 6}, {"name": "MAX_ADDR", "bits": 10, "attr": ["rw"], "rotate": 0}, {"bits": 2}, {"name": "SIZE_VAL", "bits": 3, "attr": ["rw"], "rotate": -90}, {"bits": 1}], "config": {"lanes": 1, "fontsize": 10, "vspace": 100}}
```

|  Bits  |  Type  |  Reset  | Name     | Description                                                                                       |
|:------:|:------:|:-------:|:---------|:--------------------------------------------------------------------------------------------------|
|   31   |        |         |          | Reserved                                                                                          |
| 30:28  |   rw   |   0x6   | SIZE_VAL | Size value presented as `QUEUE_SIZE.TX_DATA_BUFFER_SIZE`. This value is log2(size in DWORDs) - 1. |
| 27:26  |        |         |          | Reserved                                                                                          |
| 25:16  |   rw   |  0xff   | MAX_ADDR | Maximum address for Controller RX use (inclusive).                                                |
| 15:10  |        |         |          | Reserved                                                                                          |
|  9:0   |   rw   |  0x80   | MIN_ADDR | Minimum address for Controller RX use.                                                            |

## CTRL_RXBUF_WPTR
Controller RX Buffer Write Pointer
- Offset: `0x2c`
- Reset default: `0x80`
- Reset mask: `0xffff`

### Fields

```wavejson
{"reg": [{"name": "ADDR", "bits": 16, "attr": ["rw"], "rotate": 0}, {"bits": 16}], "config": {"lanes": 1, "fontsize": 10, "vspace": 80}}
```

|  Bits  |  Type  |  Reset  | Name   | Description            |
|:------:|:------:|:-------:|:-------|:-----------------------|
| 31:16  |        |         |        | Reserved               |
|  15:0  |   rw   |  0x80   | ADDR   | Current write address. |

## CTRL_RXBUF_RPTR
Controller RX Buffer Read Pointer
- Offset: `0x30`
- Reset default: `0x80`
- Reset mask: `0xffff`

### Fields

```wavejson
{"reg": [{"name": "ADDR", "bits": 16, "attr": ["rw"], "rotate": 0}, {"bits": 16}], "config": {"lanes": 1, "fontsize": 10, "vspace": 80}}
```

|  Bits  |  Type  |  Reset  | Name   | Description           |
|:------:|:------:|:-------:|:-------|:----------------------|
| 31:16  |        |         |        | Reserved              |
|  15:0  |   rw   |  0x80   | ADDR   | Current read address. |

## TARG_TXBUF_CONFIG
Target TX Buffer Configuration
- Offset: `0x34`
- Reset default: `0x17f0100`
- Reset mask: `0x3ff03ff`

### Fields

```wavejson
{"reg": [{"name": "MIN_ADDR", "bits": 10, "attr": ["rw"], "rotate": 0}, {"bits": 6}, {"name": "MAX_ADDR", "bits": 10, "attr": ["rw"], "rotate": 0}, {"bits": 6}], "config": {"lanes": 1, "fontsize": 10, "vspace": 80}}
```

|  Bits  |  Type  |  Reset  | Name     | Description                                    |
|:------:|:------:|:-------:|:---------|:-----------------------------------------------|
| 31:26  |        |         |          | Reserved                                       |
| 25:16  |   rw   |  0x17f  | MAX_ADDR | Maximum address for Target TX use (inclusive). |
| 15:10  |        |         |          | Reserved                                       |
|  9:0   |   rw   |  0x100  | MIN_ADDR | Minimum address for Target TX use.             |

## TARG_TXBUF_WPTR
Target TX Buffer Write Pointer
- Offset: `0x38`
- Reset default: `0x100`
- Reset mask: `0xffff`

### Fields

```wavejson
{"reg": [{"name": "ADDR", "bits": 16, "attr": ["rw"], "rotate": 0}, {"bits": 16}], "config": {"lanes": 1, "fontsize": 10, "vspace": 80}}
```

|  Bits  |  Type  |  Reset  | Name   | Description            |
|:------:|:------:|:-------:|:-------|:-----------------------|
| 31:16  |        |         |        | Reserved               |
|  15:0  |   rw   |  0x100  | ADDR   | Current write address. |

## TARG_TXBUF_RPTR
Target TX Buffer Read Pointer
- Offset: `0x3c`
- Reset default: `0x100`
- Reset mask: `0xffff`

### Fields

```wavejson
{"reg": [{"name": "ADDR", "bits": 16, "attr": ["rw"], "rotate": 0}, {"bits": 16}], "config": {"lanes": 1, "fontsize": 10, "vspace": 80}}
```

|  Bits  |  Type  |  Reset  | Name   | Description           |
|:------:|:------:|:-------:|:-------|:----------------------|
| 31:16  |        |         |        | Reserved              |
|  15:0  |   rw   |  0x100  | ADDR   | Current read address. |

## TARG_RXBUF_CONFIG
Target RX Buffer Configuration
- Offset: `0x40`
- Reset default: `0x1ff0180`
- Reset mask: `0x3ff03ff`

### Fields

```wavejson
{"reg": [{"name": "MIN_ADDR", "bits": 10, "attr": ["rw"], "rotate": 0}, {"bits": 6}, {"name": "MAX_ADDR", "bits": 10, "attr": ["rw"], "rotate": 0}, {"bits": 6}], "config": {"lanes": 1, "fontsize": 10, "vspace": 80}}
```

|  Bits  |  Type  |  Reset  | Name     | Description                                    |
|:------:|:------:|:-------:|:---------|:-----------------------------------------------|
| 31:26  |        |         |          | Reserved                                       |
| 25:16  |   rw   |  0x1ff  | MAX_ADDR | Maximum address for Target RX use (inclusive). |
| 15:10  |        |         |          | Reserved                                       |
|  9:0   |   rw   |  0x180  | MIN_ADDR | Minimum address for Target RX use.             |

## TARG_RXBUF_WPTR
Target RX Buffer Write Pointer
- Offset: `0x44`
- Reset default: `0x180`
- Reset mask: `0xffff`

### Fields

```wavejson
{"reg": [{"name": "ADDR", "bits": 16, "attr": ["rw"], "rotate": 0}, {"bits": 16}], "config": {"lanes": 1, "fontsize": 10, "vspace": 80}}
```

|  Bits  |  Type  |  Reset  | Name   | Description            |
|:------:|:------:|:-------:|:-------|:-----------------------|
| 31:16  |        |         |        | Reserved               |
|  15:0  |   rw   |  0x180  | ADDR   | Current write address. |

## TARG_RXBUF_RPTR
Target RX Buffer Read Pointer
- Offset: `0x48`
- Reset default: `0x180`
- Reset mask: `0xffff`

### Fields

```wavejson
{"reg": [{"name": "ADDR", "bits": 16, "attr": ["rw"], "rotate": 0}, {"bits": 16}], "config": {"lanes": 1, "fontsize": 10, "vspace": 80}}
```

|  Bits  |  Type  |  Reset  | Name   | Description           |
|:------:|:------:|:-------:|:-------|:----------------------|
| 31:16  |        |         |        | Reserved              |
|  15:0  |   rw   |  0x180  | ADDR   | Current read address. |

## BUFFER_CTRL
Buffer control
- Offset: `0x4c`
- Reset default: `0x0`
- Reset mask: `0x80000000`

### Fields

```wavejson
{"reg": [{"bits": 31}, {"name": "CLEAR", "bits": 1, "attr": ["wo"], "rotate": -90}], "config": {"lanes": 1, "fontsize": 10, "vspace": 80}}
```

|  Bits  |  Type  |  Reset  | Name   | Description                                                                                                                                                          |
|:------:|:------:|:-------:|:-------|:---------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|   31   |   wo   |   0x0   | CLEAR  | Software clear of buffer state including the following: - All `WPTR` and `RPTR` values are set to the corresponding the `MIN` values, emptying all circular buffers. |
|  30:0  |        |         |        | Reserved                                                                                                                                                             |

## TARG_ADDR_MATCH
I3C target addresses
- Offset: `0x50`
- Reset default: `0x7f7f`
- Reset mask: `0x7f7f`

### Fields

```wavejson
{"reg": [{"name": "ADDR0", "bits": 7, "attr": ["rw"], "rotate": 0}, {"bits": 1}, {"name": "ADDR1", "bits": 7, "attr": ["rw"], "rotate": 0}, {"bits": 17}], "config": {"lanes": 1, "fontsize": 10, "vspace": 80}}
```

|  Bits  |  Type  |  Reset  | Name   | Description           |
|:------:|:------:|:-------:|:-------|:----------------------|
| 31:15  |        |         |        | Reserved              |
|  14:8  |   rw   |  0x7f   | ADDR1  | Second target address |
|   7    |        |         |        | Reserved              |
|  6:0   |   rw   |  0x7f   | ADDR0  | First target address  |

## TARG_ADDR_MASK
I3C target address masks
- Offset: `0x54`
- Reset default: `0x0`
- Reset mask: `0x7f7f`

### Fields

```wavejson
{"reg": [{"name": "MASK0", "bits": 7, "attr": ["rw"], "rotate": 0}, {"bits": 1}, {"name": "MASK1", "bits": 7, "attr": ["rw"], "rotate": 0}, {"bits": 17}], "config": {"lanes": 1, "fontsize": 10, "vspace": 80}}
```

|  Bits  |  Type  |  Reset  | Name   | Description                |
|:------:|:------:|:-------:|:-------|:---------------------------|
| 31:15  |        |         |        | Reserved                   |
|  14:8  |   rw   |   0x0   | MASK1  | Second target address mask |
|   7    |        |         |        | Reserved                   |
|  6:0   |   rw   |   0x0   | MASK0  | First target address mask  |

## CTRL_CLK_CONFIG
Controller clock configuration.
- Offset: `0x58`
- Reset default: `0x200`
- Reset mask: `0xffffffff`

### Fields

```wavejson
{"reg": [{"name": "TCBP", "bits": 8, "attr": ["rw"], "rotate": 0}, {"name": "TCAS", "bits": 8, "attr": ["rw"], "rotate": 0}, {"name": "CLKDIV", "bits": 16, "attr": ["rw"], "rotate": 0}], "config": {"lanes": 1, "fontsize": 10, "vspace": 80}}
```

|  Bits  |  Type  |  Reset  | Name   | Description                                        |
|:------:|:------:|:-------:|:-------|:---------------------------------------------------|
| 31:16  |   rw   |   0x0   | CLKDIV | Clock divisor for Controller-driven SCL signaling. |
|  15:8  |   rw   |   0x2   | TCAS   | Address Setup cycles.                              |
|  7:0   |   rw   |   0x0   | TCBP   | STOP cycles.                                       |

## HCI_VERSION
HCI Version.
- Offset: `0x100`
- Reset default: `0x120`
- Reset mask: `0xffffffff`

### Fields

```wavejson
{"reg": [{"name": "VERSION", "bits": 32, "attr": ["ro"], "rotate": 0}], "config": {"lanes": 1, "fontsize": 10, "vspace": 80}}
```

|  Bits  |  Type  |  Reset  | Name    | Description   |
|:------:|:------:|:-------:|:--------|:--------------|
|  31:0  |   ro   |  0x120  | VERSION | HCI Version.  |

## HC_CONTROL
Host Controller Control.
- Offset: `0x104`
- Reset default: `0x48`
- Reset mask: `0xe00011d9`

### Fields

```wavejson
{"reg": [{"name": "IBA_INCLUDE", "bits": 1, "attr": ["rw"], "rotate": -90}, {"bits": 2}, {"name": "AUTOCMD_DATA_RPT", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "DATA_BYTE_ORDER_MODE", "bits": 1, "attr": ["ro"], "rotate": -90}, {"bits": 1}, {"name": "MODE_SELECTOR", "bits": 1, "attr": ["ro"], "rotate": -90}, {"name": "I2C_DEV_PRESENT", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "HOT_JOIN_CTRL", "bits": 1, "attr": ["rw"], "rotate": -90}, {"bits": 3}, {"name": "HALT_ON_CMD_SEQ_TIMEOUT", "bits": 1, "attr": ["rw"], "rotate": -90}, {"bits": 16}, {"name": "ABORT", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "RESUME", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "BUS_ENABLE", "bits": 1, "attr": ["rw"], "rotate": -90}], "config": {"lanes": 1, "fontsize": 10, "vspace": 250}}
```

|  Bits  |  Type  |  Reset  | Name                    | Description                       |
|:------:|:------:|:-------:|:------------------------|:----------------------------------|
|   31   |   rw   |   0x0   | BUS_ENABLE              | Host Controller Bus Enable.       |
|   30   |   rw   |   0x0   | RESUME                  | Host Controller Resume.           |
|   29   |   rw   |   0x0   | ABORT                   | Host Controller Abort.            |
| 28:13  |        |         |                         | Reserved                          |
|   12   |   rw   |   0x0   | HALT_ON_CMD_SEQ_TIMEOUT | Halt on Comamnd Sequence Timeout. |
|  11:9  |        |         |                         | Reserved                          |
|   8    |   rw   |   0x0   | HOT_JOIN_CTRL           | Hot-Join ACK/NACK Control.        |
|   7    |   rw   |   0x0   | I2C_DEV_PRESENT         | I2C Device Present on Bus.        |
|   6    |   ro   |   0x1   | MODE_SELECTOR           | DMA/PIO Mode Selector.            |
|   5    |        |         |                         | Reserved                          |
|   4    |   ro   |   0x0   | DATA_BYTE_ORDER_MODE    | Data Byte Ordering Mode.          |
|   3    |   rw   |   0x1   | AUTOCMD_DATA_RPT        | Auto-Command Data Report.         |
|  2:1   |        |         |                         | Reserved                          |
|   0    |   rw   |   0x0   | IBA_INCLUDE             | Include I3C Broadcast Address.    |

## CONTROLLER_DEVICE_ADDR
Controller Device Address.
- Offset: `0x108`
- Reset default: `0x0`
- Reset mask: `0x807f0000`

### Fields

```wavejson
{"reg": [{"bits": 16}, {"name": "DYNAMIC_ADDR", "bits": 7, "attr": ["rw"], "rotate": 0}, {"bits": 8}, {"name": "DYNAMIC_ADDR_VALID", "bits": 1, "attr": ["rw"], "rotate": -90}], "config": {"lanes": 1, "fontsize": 10, "vspace": 200}}
```

|  Bits  |  Type  |  Reset  | Name               | Description               |
|:------:|:------:|:-------:|:-------------------|:--------------------------|
|   31   |   rw   |   0x0   | DYNAMIC_ADDR_VALID | Dynamic Address is Valid. |
| 30:23  |        |         |                    | Reserved                  |
| 22:16  |   rw   |   0x0   | DYNAMIC_ADDR       | Device Dynamic Address.   |
|  15:0  |        |         |                    | Reserved                  |

## HC_CAPABILITIES
Host Controller Capabilities.
- Offset: `0x10c`
- Reset default: `0x3c6c`
- Reset mask: `0x70303cec`

### Fields

```wavejson
{"reg": [{"bits": 2}, {"name": "COMBO_COMMAND", "bits": 1, "attr": ["ro"], "rotate": -90}, {"name": "AUTO_COMMAND", "bits": 1, "attr": ["ro"], "rotate": -90}, {"bits": 1}, {"name": "STANDBY_CR_CAP", "bits": 1, "attr": ["ro"], "rotate": -90}, {"name": "HDR_DDR_EN", "bits": 1, "attr": ["ro"], "rotate": -90}, {"name": "HDR_TS_EN", "bits": 1, "attr": ["ro"], "rotate": -90}, {"bits": 2}, {"name": "CMD_CCC_DEFBYTE", "bits": 1, "attr": ["ro"], "rotate": -90}, {"name": "IBI_DATA_ABORT_EN", "bits": 1, "attr": ["ro"], "rotate": -90}, {"name": "IBI_CREDIT_COUNT_EN", "bits": 1, "attr": ["ro"], "rotate": -90}, {"name": "SCHEDULE_COMMANDS_EN", "bits": 1, "attr": ["ro"], "rotate": -90}, {"bits": 6}, {"name": "CMD_SIZE", "bits": 2, "attr": ["ro"], "rotate": -90}, {"bits": 6}, {"name": "SG_CAPABILITY_CR_EN", "bits": 1, "attr": ["ro"], "rotate": -90}, {"name": "SG_CAPABILITY_IBI_EN", "bits": 1, "attr": ["ro"], "rotate": -90}, {"name": "SG_CAPABILITY_DC_EN", "bits": 1, "attr": ["ro"], "rotate": -90}, {"bits": 1}], "config": {"lanes": 1, "fontsize": 10, "vspace": 220}}
```

|  Bits  |  Type  |  Reset  | Name                 | Description                                                                                            |
|:------:|:------:|:-------:|:---------------------|:-------------------------------------------------------------------------------------------------------|
|   31   |        |         |                      | Reserved                                                                                               |
|   30   |   ro   |   0x0   | SG_CAPABILITY_DC_EN  | Defines whether the Host Controller supports Scatter-Gather for DC.                                    |
|   29   |   ro   |   0x0   | SG_CAPABILITY_IBI_EN | Defines whether the Host Controller supports Scatter-Gather for IBI.                                   |
|   28   |   ro   |   0x0   | SG_CAPABILITY_CR_EN  | Defines whether the Host Controller supports Scatter-Gather for CR.                                    |
| 27:22  |        |         |                      | Reserved                                                                                               |
| 21:20  |   ro   |   0x0   | CMD_SIZE             | Defines the size and structure of the Command Descriptor supported by the Host Controller.             |
| 19:14  |        |         |                      | Reserved                                                                                               |
|   13   |   ro   |   0x1   | SCHEDULE_COMMANDS_EN | Defines whether the Host Controller supports Scheduled Commands capabilities.                          |
|   12   |   ro   |   0x1   | IBI_CREDIT_COUNT_EN  | Defines whether the Host Controller supports Target IBI Credit Counting.                               |
|   11   |   ro   |   0x1   | IBI_DATA_ABORT_EN    | Defines whether the Host Controller supports the IBI Data Abort operation.                             |
|   10   |   ro   |   0x1   | CMD_CCC_DEFBYTE      | Defines whether the Host Controller supports Transfer Commands that indicate CCCs when Defining Bytes. |
|  9:8   |        |         |                      | Reserved                                                                                               |
|   7    |   ro   |   0x0   | HDR_TS_EN            | Defines whether the Host Controller supports HDR-Ternary transfers.                                    |
|   6    |   ro   |   0x1   | HDR_DDR_EN           | Defines whether the Host Controller supports HDR-DDR transfers.                                        |
|   5    |   ro   |   0x1   | STANDBY_CR_CAP       | Defines whether the Host Controller supports handoff of the Active Controller role.                    |
|   4    |        |         |                      | Reserved                                                                                               |
|   3    |   ro   |   0x1   | AUTO_COMMAND         | Defines whether the Host Controller supports Auto-Command functionality.                               |
|   2    |   ro   |   0x1   | COMBO_COMMAND        | Defines whether the Host Controller supports Combo Transfer Command transfers.                         |
|  1:0   |        |         |                      | Reserved                                                                                               |

## RESET_CONTROL
Reset Control.
- Offset: `0x120`
- Reset default: `0x0`
- Reset mask: `0x3f`

### Fields

```wavejson
{"reg": [{"name": "SOFT_RST", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "CMD_QUEUE_RST", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "RESP_QUEUE_RST", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "TX_FIFO_RST", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "RX_FIFO_RST", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "IBI_QUEUE_RST", "bits": 1, "attr": ["rw"], "rotate": -90}, {"bits": 26}], "config": {"lanes": 1, "fontsize": 10, "vspace": 160}}
```

|  Bits  |  Type  |  Reset  | Name           | Description                           |
|:------:|:------:|:-------:|:---------------|:--------------------------------------|
|  31:6  |        |         |                | Reserved                              |
|   5    |   rw   |   0x0   | IBI_QUEUE_RST  | IBI Queue Buffer Software Reset.      |
|   4    |   rw   |   0x0   | RX_FIFO_RST    | Receive Queue Buffer Software Reset.  |
|   3    |   rw   |   0x0   | TX_FIFO_RST    | Transmit Queue buffer Software Reset. |
|   2    |   rw   |   0x0   | RESP_QUEUE_RST | Response Queue Software Reset.        |
|   1    |   rw   |   0x0   | CMD_QUEUE_RST  | Command Queue Software Reset.         |
|   0    |   rw   |   0x0   | SOFT_RST       | Core Software Reset.                  |

## PRESENT_STATE
Present State.
- Offset: `0x124`
- Reset default: `0x0`
- Reset mask: `0x4`

### Fields

```wavejson
{"reg": [{"bits": 2}, {"name": "AC_CURRENT_OWN", "bits": 1, "attr": ["ro"], "rotate": -90}, {"bits": 29}], "config": {"lanes": 1, "fontsize": 10, "vspace": 160}}
```

|  Bits  |  Type  |  Reset  | Name           | Description        |
|:------:|:------:|:-------:|:---------------|:-------------------|
|  31:3  |        |         |                | Reserved           |
|   2    |   ro   |   0x0   | AC_CURRENT_OWN | Active Controller. |
|  1:0   |        |         |                | Reserved           |

## INTR_STATUS
Interrupt Status.
- Offset: `0x128`
- Reset default: `0x0`
- Reset mask: `0x7c00`

### Fields

```wavejson
{"reg": [{"bits": 10}, {"name": "HC_INTERNAL_ERR_STAT", "bits": 1, "attr": ["rw1c"], "rotate": -90}, {"name": "HC_SEQ_CANCEL_STAT", "bits": 1, "attr": ["rw1c"], "rotate": -90}, {"name": "HC_WARN_CMD_SEQ_STALL_STAT", "bits": 1, "attr": ["rw1c"], "rotate": -90}, {"name": "HC_ERR_CMD_SEQ_TIMEOUT_STAT", "bits": 1, "attr": ["rw1c"], "rotate": -90}, {"name": "SCHED_CMD_MISSED_TICK_STAT", "bits": 1, "attr": ["rw1c"], "rotate": -90}, {"bits": 17}], "config": {"lanes": 1, "fontsize": 10, "vspace": 290}}
```

|  Bits  |  Type  |  Reset  | Name                        | Description                                     |
|:------:|:------:|:-------:|:----------------------------|:------------------------------------------------|
| 31:15  |        |         |                             | Reserved                                        |
|   14   |  rw1c  |   0x0   | SCHED_CMD_MISSED_TICK_STAT  | Scheduled Command Missed Tick.                  |
|   13   |  rw1c  |   0x0   | HC_ERR_CMD_SEQ_TIMEOUT_STAT | Host Controller Command Sequence Timeout.       |
|   12   |  rw1c  |   0x0   | HC_WARN_CMD_SEQ_STALL_STAT  | Host Controller Command Sequence Stall.         |
|   11   |  rw1c  |   0x0   | HC_SEQ_CANCEL_STAT          | Host Controller Cancelled Transaction Sequence. |
|   10   |  rw1c  |   0x0   | HC_INTERNAL_ERR_STAT        | Host Controller Internal Error.                 |
|  9:0   |        |         |                             | Reserved                                        |

## INTR_STATUS_ENABLE
Interrupt Status Enable.
- Offset: `0x12c`
- Reset default: `0x0`
- Reset mask: `0x7c00`

### Fields

```wavejson
{"reg": [{"bits": 10}, {"name": "HC_INTERNAL_ERR_STAT_EN", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "HC_SEQ_CANCEL_STAT_EN", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "HC_WARN_CMD_SEQ_STALL_STAT_EN", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "HC_ERR_CMD_SEQ_TIMEOUT_STAT_EN", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "SCHED_CMD_MISSED_TICK_STAT_EN", "bits": 1, "attr": ["rw"], "rotate": -90}, {"bits": 17}], "config": {"lanes": 1, "fontsize": 10, "vspace": 320}}
```

|  Bits  |  Type  |  Reset  | Name                           | Description                                                   |
|:------:|:------:|:-------:|:-------------------------------|:--------------------------------------------------------------|
| 31:15  |        |         |                                | Reserved                                                      |
|   14   |   rw   |   0x0   | SCHED_CMD_MISSED_TICK_STAT_EN  | Scheduled Command Missed Tick Status Enable.                  |
|   13   |   rw   |   0x0   | HC_ERR_CMD_SEQ_TIMEOUT_STAT_EN | Host Controller Command Sequence Timeout Status Enable.       |
|   12   |   rw   |   0x0   | HC_WARN_CMD_SEQ_STALL_STAT_EN  | Host Controller Command Sequence Stall Status Enable.         |
|   11   |   rw   |   0x0   | HC_SEQ_CANCEL_STAT_EN          | Host Controller Cancelled Transaction Sequence Status Enable. |
|   10   |   rw   |   0x0   | HC_INTERNAL_ERR_STAT_EN        | Host Controller Internal Error Error Status Enable.           |
|  9:0   |        |         |                                | Reserved                                                      |

## INTR_SIGNAL_ENABLE
Interrupt Signal Enable.
- Offset: `0x130`
- Reset default: `0x0`
- Reset mask: `0x7c00`

### Fields

```wavejson
{"reg": [{"bits": 10}, {"name": "HC_INTERNAL_ERR_SIGNAL_EN", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "HC_SEQ_CANCEL_SIGNAL_EN", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "HC_WARN_CMD_SEQ_STALL_SIGNAL_EN", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "HC_ERR_CMD_SEQ_TIMEOUT_SIGNAL_EN", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "SCHED_CMD_MISSED_TICK_SIGNAL_EN", "bits": 1, "attr": ["rw"], "rotate": -90}, {"bits": 17}], "config": {"lanes": 1, "fontsize": 10, "vspace": 340}}
```

|  Bits  |  Type  |  Reset  | Name                             | Description                                                   |
|:------:|:------:|:-------:|:---------------------------------|:--------------------------------------------------------------|
| 31:15  |        |         |                                  | Reserved                                                      |
|   14   |   rw   |   0x0   | SCHED_CMD_MISSED_TICK_SIGNAL_EN  | Scheduled Command Missed Tick Signal Enable.                  |
|   13   |   rw   |   0x0   | HC_ERR_CMD_SEQ_TIMEOUT_SIGNAL_EN | Host Controller Command Sequence Timeout Signal Enable.       |
|   12   |   rw   |   0x0   | HC_WARN_CMD_SEQ_STALL_SIGNAL_EN  | Host Controller Command Sequence Stall Signal Enable.         |
|   11   |   rw   |   0x0   | HC_SEQ_CANCEL_SIGNAL_EN          | Host Controller Cancelled Transaction Sequence Signal Enable. |
|   10   |   rw   |   0x0   | HC_INTERNAL_ERR_SIGNAL_EN        | Host Controller Internal Error Signal Enable.                 |
|  9:0   |        |         |                                  | Reserved                                                      |

## INTR_FORCE
Interrupt Force.
- Offset: `0x134`
- Reset default: `0x0`
- Reset mask: `0x7c00`

### Fields

```wavejson
{"reg": [{"bits": 10}, {"name": "HC_INTERNAL_ERR_FORCE", "bits": 1, "attr": ["wo"], "rotate": -90}, {"name": "HC_SEQ_CANCEL_FORCE", "bits": 1, "attr": ["wo"], "rotate": -90}, {"name": "HC_WARN_CMD_SEQ_STALL_FORCE", "bits": 1, "attr": ["wo"], "rotate": -90}, {"name": "HC_ERR_CMD_SEQ_TIMEOUT_FORCE", "bits": 1, "attr": ["wo"], "rotate": -90}, {"name": "SCHED_CMD_MISSED_TICK_FORCE", "bits": 1, "attr": ["wo"], "rotate": -90}, {"bits": 17}], "config": {"lanes": 1, "fontsize": 10, "vspace": 300}}
```

|  Bits  |  Type  |  Reset  | Name                         | Description                                           |
|:------:|:------:|:-------:|:-----------------------------|:------------------------------------------------------|
| 31:15  |        |         |                              | Reserved                                              |
|   14   |   wo   |   0x0   | SCHED_CMD_MISSED_TICK_FORCE  | Force Scheduled Command Missed Tick.                  |
|   13   |   wo   |   0x0   | HC_ERR_CMD_SEQ_TIMEOUT_FORCE | Force Host Controller Command Sequence Timeout.       |
|   12   |   wo   |   0x0   | HC_WARN_CMD_SEQ_STALL_FORCE  | Force Host Controller Command Sequence Stall.         |
|   11   |   wo   |   0x0   | HC_SEQ_CANCEL_FORCE          | Force Host Controller Cancelled Transaction Sequence. |
|   10   |   wo   |   0x0   | HC_INTERNAL_ERR_FORCE        | Force Host Controller Internal Error.                 |
|  9:0   |        |         |                              | Reserved                                              |

## DAT_SECTION_OFFSET
Device Address Table Section Offset.
- Offset: `0x138`
- Reset default: `0x20000`
- Reset mask: `0xf007ffff`

### Fields

```wavejson
{"reg": [{"name": "TABLE_OFFSET", "bits": 12, "attr": ["ro"], "rotate": 0}, {"name": "TABLE_SIZE", "bits": 7, "attr": ["ro"], "rotate": 0}, {"bits": 9}, {"name": "ENTRY_SIZE", "bits": 4, "attr": ["ro"], "rotate": -90}], "config": {"lanes": 1, "fontsize": 10, "vspace": 120}}
```

|  Bits  |  Type  |  Reset  | Name         | Description       |
|:------:|:------:|:-------:|:-------------|:------------------|
| 31:28  |   ro   |   0x0   | ENTRY_SIZE   | DAT Entry size.   |
| 27:19  |        |         |              | Reserved          |
| 18:12  |   ro   |  0x20   | TABLE_SIZE   | DAT Table Size.   |
|  11:0  |   ro   |    x    | TABLE_OFFSET | DAT Table Offset. |

## DCT_SECTION_OFFSET
Device Characteristics Table Section Offset.
- Offset: `0x13c`
- Reset default: `0x20000`
- Reset mask: `0xf007ffff`

### Fields

```wavejson
{"reg": [{"name": "TABLE_OFFSET", "bits": 12, "attr": ["ro"], "rotate": 0}, {"name": "TABLE_SIZE", "bits": 7, "attr": ["ro"], "rotate": 0}, {"bits": 9}, {"name": "ENTRY_SIZE", "bits": 4, "attr": ["ro"], "rotate": -90}], "config": {"lanes": 1, "fontsize": 10, "vspace": 120}}
```

|  Bits  |  Type  |  Reset  | Name         | Description       |
|:------:|:------:|:-------:|:-------------|:------------------|
| 31:28  |   ro   |   0x0   | ENTRY_SIZE   | DCT Entry size.   |
| 27:19  |        |         |              | Reserved          |
| 18:12  |   ro   |  0x20   | TABLE_SIZE   | DCT Table Size.   |
|  11:0  |   ro   |    x    | TABLE_OFFSET | DCT Table Offset. |

## RING_HEADERS_SECTION_OFFSET
Ring Headers Section Offset.
- Offset: `0x140`
- Reset default: `0x0`
- Reset mask: `0xffff`

### Fields

```wavejson
{"reg": [{"name": "SECTION_OFFSET", "bits": 16, "attr": ["ro"], "rotate": 0}, {"bits": 16}], "config": {"lanes": 1, "fontsize": 10, "vspace": 80}}
```

|  Bits  |  Type  |  Reset  | Name           | Description                  |
|:------:|:------:|:-------:|:---------------|:-----------------------------|
| 31:16  |        |         |                | Reserved                     |
|  15:0  |   ro   |   0x0   | SECTION_OFFSET | Ring Headers Section Offset. |

## PIO_SECTION_OFFSET
PIO Section Offset.
- Offset: `0x144`
- Reset default: `0x100`
- Reset mask: `0xffff`

### Fields

```wavejson
{"reg": [{"name": "SECTION_OFFSET", "bits": 16, "attr": ["ro"], "rotate": 0}, {"bits": 16}], "config": {"lanes": 1, "fontsize": 10, "vspace": 80}}
```

|  Bits  |  Type  |  Reset  | Name           | Description         |
|:------:|:------:|:-------:|:---------------|:--------------------|
| 31:16  |        |         |                | Reserved            |
|  15:0  |   ro   |  0x100  | SECTION_OFFSET | PIO Section Offset. |

## EXT_CAPS_SECTION_OFFSET
Extended Capabilities Section Offset.
- Offset: `0x148`
- Reset default: `0x0`
- Reset mask: `0xffff`

### Fields

```wavejson
{"reg": [{"name": "SECTION_OFFSET", "bits": 16, "attr": ["ro"], "rotate": 0}, {"bits": 16}], "config": {"lanes": 1, "fontsize": 10, "vspace": 80}}
```

|  Bits  |  Type  |  Reset  | Name           | Description                           |
|:------:|:------:|:-------:|:---------------|:--------------------------------------|
| 31:16  |        |         |                | Reserved                              |
|  15:0  |   ro   |   0x0   | SECTION_OFFSET | Extended Capabilities Section Offset. |

## INT_CTRL_CMDS_EN
Internal Control Command Subtype Support.
- Offset: `0x14c`
- Reset default: `0x31`
- Reset mask: `0xffff`

### Fields

```wavejson
{"reg": [{"name": "ICC_SUPPORT", "bits": 1, "attr": ["ro"], "rotate": -90}, {"name": "MIPI_CMDS_SUPPORTED", "bits": 15, "attr": ["ro"], "rotate": 0}, {"bits": 16}], "config": {"lanes": 1, "fontsize": 10, "vspace": 130}}
```

|  Bits  |  Type  |  Reset  | Name                | Description                          |
|:------:|:------:|:-------:|:--------------------|:-------------------------------------|
| 31:16  |        |         |                     | Reserved                             |
|  15:1  |   ro   |  0x18   | MIPI_CMDS_SUPPORTED | MIPI Alliance Commands Supported.    |
|   0    |   ro   |   0x1   | ICC_SUPPORT         | Internal Control Commands Supported. |

## IBI_NOTIFY_CTRL
IBI Notify Control.
- Offset: `0x158`
- Reset default: `0x0`
- Reset mask: `0xb`

### Fields

```wavejson
{"reg": [{"name": "NOTIFY_HJ_REJECTED", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "NOTIFY_CRR_REJECTED", "bits": 1, "attr": ["rw"], "rotate": -90}, {"bits": 1}, {"name": "NOTIFY_IBI_REJECTED", "bits": 1, "attr": ["rw"], "rotate": -90}, {"bits": 28}], "config": {"lanes": 1, "fontsize": 10, "vspace": 210}}
```

|  Bits  |  Type  |  Reset  | Name                | Description                                        |
|:------:|:------:|:-------:|:--------------------|:---------------------------------------------------|
|  31:4  |        |         |                     | Reserved                                           |
|   3    |   rw   |   0x0   | NOTIFY_IBI_REJECTED | Notify Rejected In-Band Interrupt Request Control. |
|   2    |        |         |                     | Reserved                                           |
|   1    |   rw   |   0x0   | NOTIFY_CRR_REJECTED | Notify Rejected Controller Role Request Control.   |
|   0    |   rw   |   0x0   | NOTIFY_HJ_REJECTED  | Notify Rejected Hot-Join Control.                  |

## IBI_DATA_ABORT_CTRL
IBI Data Abort Control.
- Offset: `0x15c`
- Reset default: `0x0`
- Reset mask: `0x801fff00`

### Fields

```wavejson
{"reg": [{"bits": 8}, {"name": "MATCH_IBI_ID", "bits": 8, "attr": ["rw"], "rotate": 0}, {"name": "AFTER_N_CHUNKS", "bits": 2, "attr": ["rw"], "rotate": -90}, {"name": "MATCH_STATUS_TYPE", "bits": 3, "attr": ["rw"], "rotate": -90}, {"bits": 10}, {"name": "IBI_DATA_ABORT_MON", "bits": 1, "attr": ["rw"], "rotate": -90}], "config": {"lanes": 1, "fontsize": 10, "vspace": 200}}
```

|  Bits  |  Type  |  Reset  | Name               | Description               |
|:------:|:------:|:-------:|:-------------------|:--------------------------|
|   31   |   rw   |   0x0   | IBI_DATA_ABORT_MON | IBI Data Abort Monitor.   |
| 30:21  |        |         |                    | Reserved                  |
| 20:18  |   rw   |   0x0   | MATCH_STATUS_TYPE  | Match IBI Status Type.    |
| 17:16  |   rw   |   0x0   | AFTER_N_CHUNKS     | Abort After N Chunks.     |
|  15:8  |   rw   |   0x0   | MATCH_IBI_ID       | Match IBI Target Address. |
|  7:0   |        |         |                    | Reserved                  |

## DEV_CTX_BASE_LO
Device Context Base Address Low.
- Offset: `0x160`
- Reset default: `0x0`
- Reset mask: `0xffffffff`

### Fields

```wavejson
{"reg": [{"name": "BASE_LO", "bits": 32, "attr": ["ro"], "rotate": 0}], "config": {"lanes": 1, "fontsize": 10, "vspace": 80}}
```

|  Bits  |  Type  |  Reset  | Name    | Description              |
|:------:|:------:|:-------:|:--------|:-------------------------|
|  31:0  |   ro   |   0x0   | BASE_LO | Device Context Base Low. |

## DEV_CTX_BASE_HI
Device Context Base Address High.
- Offset: `0x164`
- Reset default: `0x0`
- Reset mask: `0xffffffff`

### Fields

```wavejson
{"reg": [{"name": "BASE_HI", "bits": 32, "attr": ["ro"], "rotate": 0}], "config": {"lanes": 1, "fontsize": 10, "vspace": 80}}
```

|  Bits  |  Type  |  Reset  | Name    | Description               |
|:------:|:------:|:-------:|:--------|:--------------------------|
|  31:0  |   ro   |   0x0   | BASE_HI | Device Context Base High. |

## DEV_CTX_SG
Device Contxt Scatter-Gather Support.
- Offset: `0x168`
- Reset default: `0x0`
- Reset mask: `0x8000ffff`

### Fields

```wavejson
{"reg": [{"name": "LIST_SIZE", "bits": 16, "attr": ["ro"], "rotate": 0}, {"bits": 15}, {"name": "BLP", "bits": 1, "attr": ["ro"], "rotate": -90}], "config": {"lanes": 1, "fontsize": 10, "vspace": 80}}
```

|  Bits  |  Type  |  Reset  | Name      | Description              |
|:------:|:------:|:-------:|:----------|:-------------------------|
|   31   |   ro   |   0x0   | BLP       | Buffer Vs. List Pointer. |
| 30:16  |        |         |           | Reserved                 |
|  15:0  |   ro   |   0x0   | LIST_SIZE | List size.               |

## COMMAND_QUEUE_PORT
Write-only Command Descriptor queue.
- Offset: `0x200`
- Reset default: `0x0`
- Reset mask: `0xffffffff`

### Fields

```wavejson
{"reg": [{"name": "CMD_DATA", "bits": 32, "attr": ["wo"], "rotate": 0}], "config": {"lanes": 1, "fontsize": 10, "vspace": 80}}
```

|  Bits  |  Type  |  Reset  | Name     | Description                   |
|:------:|:------:|:-------:|:---------|:------------------------------|
|  31:0  |   wo   |    x    | CMD_DATA | Command Descriptor structure. |

## RESPONSE_QUEUE_PORT
Read-only Response Descriptor queue.
- Offset: `0x204`
- Reset default: `0x0`
- Reset mask: `0xffffffff`

### Fields

```wavejson
{"reg": [{"name": "RSP_DATA", "bits": 32, "attr": ["ro"], "rotate": 0}], "config": {"lanes": 1, "fontsize": 10, "vspace": 80}}
```

|  Bits  |  Type  |  Reset  | Name     | Description                    |
|:------:|:------:|:-------:|:---------|:-------------------------------|
|  31:0  |   ro   |    x    | RSP_DATA | Response Descriptor structure. |

## XFER_DATA_PORT
Read/write data transfer port.

- Word Aligned Offset Range: `0x208`to`0x208`
- Size (words): `1`
- Access: `rw`
- Byte writes are  supported.

## IBI_PORT
Read-only IBI queue.
- Offset: `0x20c`
- Reset default: `0x0`
- Reset mask: `0xffffffff`

### Fields

```wavejson
{"reg": [{"name": "IBI_DATA", "bits": 32, "attr": ["ro"], "rotate": 0}], "config": {"lanes": 1, "fontsize": 10, "vspace": 80}}
```

|  Bits  |  Type  |  Reset  | Name     | Description                               |
|:------:|:------:|:-------:|:---------|:------------------------------------------|
|  31:0  |   ro   |    x    | IBI_DATA | IBI Status Descriptor and IBI Data words. |

## QUEUE_THLD_CTRL
Queue Threshold Control.
- Offset: `0x210`
- Reset default: `0x1010101`
- Reset mask: `0xffffffff`

### Fields

```wavejson
{"reg": [{"name": "CMD_EMPTY_BUF_THLD", "bits": 8, "attr": ["rw"], "rotate": -90}, {"name": "RESP_BUF_THLD", "bits": 8, "attr": ["rw"], "rotate": 0}, {"name": "IBI_DATA_SEGMENT_SIZE", "bits": 8, "attr": ["rw"], "rotate": -90}, {"name": "IBI_STATUS_THLD", "bits": 8, "attr": ["rw"], "rotate": -90}], "config": {"lanes": 1, "fontsize": 10, "vspace": 230}}
```

|  Bits  |  Type  |  Reset  | Name                  | Description                      |
|:------:|:------:|:-------:|:----------------------|:---------------------------------|
| 31:24  |   rw   |   0x1   | IBI_STATUS_THLD       | IBI Status Threshold.            |
| 23:16  |   rw   |   0x1   | IBI_DATA_SEGMENT_SIZE | IBI Data Segment Size.           |
|  15:8  |   rw   |   0x1   | RESP_BUF_THLD         | Response Ready Buffer Threshold. |
|  7:0   |   rw   |   0x1   | CMD_EMPTY_BUF_THLD    | Command Ready Buffer Threshold.  |

## DATA_BUFFER_THLD_CTRL
Transfer Data Buffer Threshold Control.
- Offset: `0x214`
- Reset default: `0x1010101`
- Reset mask: `0x7070707`

### Fields

```wavejson
{"reg": [{"name": "TX_BUF_THLD", "bits": 3, "attr": ["rw"], "rotate": -90}, {"bits": 5}, {"name": "RX_BUF_THLD", "bits": 3, "attr": ["rw"], "rotate": -90}, {"bits": 5}, {"name": "TX_START_THLD", "bits": 3, "attr": ["rw"], "rotate": -90}, {"bits": 5}, {"name": "RX_START_THLD", "bits": 3, "attr": ["rw"], "rotate": -90}, {"bits": 5}], "config": {"lanes": 1, "fontsize": 10, "vspace": 150}}
```

|  Bits  |  Type  |  Reset  | Name          | Description                                    |
|:------:|:------:|:-------:|:--------------|:-----------------------------------------------|
| 31:27  |        |         |               | Reserved                                       |
| 26:24  |   rw   |   0x1   | RX_START_THLD | Receive Start Threshold in DWORDs.             |
| 23:19  |        |         |               | Reserved                                       |
| 18:16  |   rw   |   0x1   | TX_START_THLD | Transmit (Transfer) Start Threshold in DWORDs. |
| 15:11  |        |         |               | Reserved                                       |
|  10:8  |   rw   |   0x1   | RX_BUF_THLD   | Receive Buffer Threshold.                      |
|  7:3   |        |         |               | Reserved                                       |
|  2:0   |   rw   |   0x1   | TX_BUF_THLD   | Transmit Buffer Threshold.                     |

## QUEUE_SIZE
Queue Size.
- Offset: `0x218`
- Reset default: `0x6060808`
- Reset mask: `0xffffffff`

### Fields

```wavejson
{"reg": [{"name": "CR_QUEUE_SIZE", "bits": 8, "attr": ["ro"], "rotate": 0}, {"name": "IBI_STATUS_SIZE", "bits": 8, "attr": ["ro"], "rotate": -90}, {"name": "RX_DATA_BUFFER_SIZE", "bits": 8, "attr": ["ro"], "rotate": -90}, {"name": "TX_DATA_BUFFER_SIZE", "bits": 8, "attr": ["ro"], "rotate": -90}], "config": {"lanes": 1, "fontsize": 10, "vspace": 210}}
```

|  Bits  |  Type  |  Reset  | Name                | Description                  |
|:------:|:------:|:-------:|:--------------------|:-----------------------------|
| 31:24  |   ro   |   0x6   | TX_DATA_BUFFER_SIZE | Transmit Data Buffer Size.   |
| 23:16  |   ro   |   0x6   | RX_DATA_BUFFER_SIZE | Receive Data Buffer Size.    |
|  15:8  |   ro   |   0x8   | IBI_STATUS_SIZE     | IBI Queue Size.              |
|  7:0   |   ro   |   0x8   | CR_QUEUE_SIZE       | Command/Response Queue Size. |

## ALT_QUEUE_SIZE
Alternate Queue Size.
- Offset: `0x21c`
- Reset default: `0x0`
- Reset mask: `0x110000ff`

### Fields

```wavejson
{"reg": [{"name": "ALT_RESP_QUEUE_SIZE", "bits": 8, "attr": ["ro"], "rotate": -90}, {"bits": 16}, {"name": "ALT_RESP_QUEUE_EN", "bits": 1, "attr": ["ro"], "rotate": -90}, {"bits": 3}, {"name": "EXT_IBI_QUEUE_EN", "bits": 1, "attr": ["ro"], "rotate": -90}, {"bits": 3}], "config": {"lanes": 1, "fontsize": 10, "vspace": 210}}
```

|  Bits  |  Type  |  Reset  | Name                | Description                    |
|:------:|:------:|:-------:|:--------------------|:-------------------------------|
| 31:29  |        |         |                     | Reserved                       |
|   28   |   ro   |   0x0   | EXT_IBI_QUEUE_EN    | Extended IBI Queue Size.       |
| 27:25  |        |         |                     | Reserved                       |
|   24   |   ro   |   0x0   | ALT_RESP_QUEUE_EN   | Alternate Response Queue.      |
|  23:8  |        |         |                     | Reserved                       |
|  7:0   |   ro   |   0x0   | ALT_RESP_QUEUE_SIZE | Alternate Response Queue Size. |

## PIO_INTR_STATUS
PIO Interrupt Status.
- Offset: `0x220`
- Reset default: `0x0`
- Reset mask: `0x23f`

### Fields

```wavejson
{"reg": [{"name": "TX_THLD_STAT", "bits": 1, "attr": ["ro"], "rotate": -90}, {"name": "RX_THLD_STAT", "bits": 1, "attr": ["ro"], "rotate": -90}, {"name": "IBI_STATUS_THLD_STAT", "bits": 1, "attr": ["ro"], "rotate": -90}, {"name": "CMD_QUEUE_READY_STAT", "bits": 1, "attr": ["ro"], "rotate": -90}, {"name": "RESP_READY_STAT", "bits": 1, "attr": ["ro"], "rotate": -90}, {"name": "TRANSFER_ABORT_STAT", "bits": 1, "attr": ["rw1c"], "rotate": -90}, {"bits": 3}, {"name": "TRANSFER_ERR_STAT", "bits": 1, "attr": ["rw1c"], "rotate": -90}, {"bits": 22}], "config": {"lanes": 1, "fontsize": 10, "vspace": 220}}
```

|  Bits  |  Type  |  Reset  | Name                 | Description                      |
|:------:|:------:|:-------:|:---------------------|:---------------------------------|
| 31:10  |        |         |                      | Reserved                         |
|   9    |  rw1c  |   0x0   | TRANSFER_ERR_STAT    | Transfer Error Status.           |
|  8:6   |        |         |                      | Reserved                         |
|   5    |  rw1c  |   0x0   | TRANSFER_ABORT_STAT  | Transfer Abort Status.           |
|   4    |   ro   |   0x0   | RESP_READY_STAT      | Response Ready Status.           |
|   3    |   ro   |   0x0   | CMD_QUEUE_READY_STAT | Command Queue Ready Status.      |
|   2    |   ro   |   0x0   | IBI_STATUS_THLD_STAT | IBI Status Threshold Status.     |
|   1    |   ro   |   0x0   | RX_THLD_STAT         | Rx Data Buffer Threshold Status. |
|   0    |   ro   |   0x0   | TX_THLD_STAT         | Tx Data Buffer Threshold Status. |

## PIO_INTR_STATUS_ENABLE
PIO Interrupt Status Enable.
- Offset: `0x224`
- Reset default: `0x0`
- Reset mask: `0x23f`

### Fields

```wavejson
{"reg": [{"name": "TX_THLD_STAT_EN", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "RX_THLD_STAT_EN", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "IBI_STATUS_THLD_STAT_EN", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "CMD_QUEUE_READY_STAT_EN", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "RESP_READY_STAT_EN", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "TRANSFER_ABORT_STAT_EN", "bits": 1, "attr": ["rw"], "rotate": -90}, {"bits": 3}, {"name": "TRANSFER_ERR_STAT_EN", "bits": 1, "attr": ["rw"], "rotate": -90}, {"bits": 22}], "config": {"lanes": 1, "fontsize": 10, "vspace": 250}}
```

|  Bits  |  Type  |  Reset  | Name                    | Description                             |
|:------:|:------:|:-------:|:------------------------|:----------------------------------------|
| 31:10  |        |         |                         | Reserved                                |
|   9    |   rw   |   0x0   | TRANSFER_ERR_STAT_EN    | Transfer Error Status Enable.           |
|  8:6   |        |         |                         | Reserved                                |
|   5    |   rw   |   0x0   | TRANSFER_ABORT_STAT_EN  | Transfer Abort Status Enable.           |
|   4    |   rw   |   0x0   | RESP_READY_STAT_EN      | Response Ready Status Enable.           |
|   3    |   rw   |   0x0   | CMD_QUEUE_READY_STAT_EN | Command Queue Ready Status Enable.      |
|   2    |   rw   |   0x0   | IBI_STATUS_THLD_STAT_EN | IBI Status Threshold Status Enable.     |
|   1    |   rw   |   0x0   | RX_THLD_STAT_EN         | Rx Data Buffer Threshold Status Enable. |
|   0    |   rw   |   0x0   | TX_THLD_STAT_EN         | Tx Data Buffer Threshold Status Enable. |

## PIO_INTR_SIGNAL_ENABLE
PIO Interrupt Signal Enable.
- Offset: `0x228`
- Reset default: `0x0`
- Reset mask: `0x23f`

### Fields

```wavejson
{"reg": [{"name": "TX_THLD_SIGNAL_EN", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "RX_THLD_SIGNAL_EN", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "IBI_STATUS_THLD_SIGNAL_EN", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "CMD_QUEUE_READY_SIGNAL_EN", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "RESP_READY_SIGNAL_EN", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "TRANSFER_ABORT_SIGNAL_EN", "bits": 1, "attr": ["rw"], "rotate": -90}, {"bits": 3}, {"name": "TRANSFER_ERR_SIGNAL_EN", "bits": 1, "attr": ["rw"], "rotate": -90}, {"bits": 22}], "config": {"lanes": 1, "fontsize": 10, "vspace": 270}}
```

|  Bits  |  Type  |  Reset  | Name                      | Description                             |
|:------:|:------:|:-------:|:--------------------------|:----------------------------------------|
| 31:10  |        |         |                           | Reserved                                |
|   9    |   rw   |   0x0   | TRANSFER_ERR_SIGNAL_EN    | Transfer Error Signal Enable.           |
|  8:6   |        |         |                           | Reserved                                |
|   5    |   rw   |   0x0   | TRANSFER_ABORT_SIGNAL_EN  | Transfer Abort Signal Enable.           |
|   4    |   rw   |   0x0   | RESP_READY_SIGNAL_EN      | Response Ready Signal Enable.           |
|   3    |   rw   |   0x0   | CMD_QUEUE_READY_SIGNAL_EN | Command Queue Ready Signal Enable.      |
|   2    |   rw   |   0x0   | IBI_STATUS_THLD_SIGNAL_EN | IBI Status Threshold Signal Enable.     |
|   1    |   rw   |   0x0   | RX_THLD_SIGNAL_EN         | Rx Data Buffer Threshold Signal Enable. |
|   0    |   rw   |   0x0   | TX_THLD_SIGNAL_EN         | Tx Data Buffer Threshold Signal Enable. |

## PIO_INTR_FORCE
PIO Interrupt Force.
- Offset: `0x22c`
- Reset default: `0x0`
- Reset mask: `0x23f`

### Fields

```wavejson
{"reg": [{"name": "TX_THLD_FORCE", "bits": 1, "attr": ["wo"], "rotate": -90}, {"name": "RX_THLD_FORCE", "bits": 1, "attr": ["wo"], "rotate": -90}, {"name": "IBI_THLD_FORCE", "bits": 1, "attr": ["wo"], "rotate": -90}, {"name": "CMD_QUEUE_READY_FORCE", "bits": 1, "attr": ["wo"], "rotate": -90}, {"name": "RESP_READY_FORCE", "bits": 1, "attr": ["wo"], "rotate": -90}, {"name": "TRANSFER_ABORT_FORCE", "bits": 1, "attr": ["wo"], "rotate": -90}, {"bits": 3}, {"name": "TRANSFER_ERR_FORCE", "bits": 1, "attr": ["wo"], "rotate": -90}, {"bits": 22}], "config": {"lanes": 1, "fontsize": 10, "vspace": 230}}
```

|  Bits  |  Type  |  Reset  | Name                  | Description                     |
|:------:|:------:|:-------:|:----------------------|:--------------------------------|
| 31:10  |        |         |                       | Reserved                        |
|   9    |   wo   |   0x0   | TRANSFER_ERR_FORCE    | Force Transfer Error.           |
|  8:6   |        |         |                       | Reserved                        |
|   5    |   wo   |   0x0   | TRANSFER_ABORT_FORCE  | Force Transfer Abort.           |
|   4    |   wo   |   0x0   | RESP_READY_FORCE      | Force Response Ready.           |
|   3    |   wo   |   0x0   | CMD_QUEUE_READY_FORCE | Force Command Queue Ready.      |
|   2    |   wo   |   0x0   | IBI_THLD_FORCE        | Force IBI Status Threshold.     |
|   1    |   wo   |   0x0   | RX_THLD_FORCE         | Force Rx Data Buffer Threshold. |
|   0    |   wo   |   0x0   | TX_THLD_FORCE         | Force Tx Data Buffer Threshold. |

## STBY_EXTCAP_HEADER
Standby Controller Extended Capability Header
- Offset: `0x230`
- Reset default: `0x12`
- Reset mask: `0xffffff`

### Fields

```wavejson
{"reg": [{"name": "CAP_ID", "bits": 8, "attr": ["ro"], "rotate": 0}, {"name": "CAP_LENGTH", "bits": 16, "attr": ["ro"], "rotate": 0}, {"bits": 8}], "config": {"lanes": 1, "fontsize": 10, "vspace": 80}}
```

|  Bits  |  Type  |  Reset  | Name       | Description                            |
|:------:|:------:|:-------:|:-----------|:---------------------------------------|
| 31:24  |        |         |            | Reserved                               |
|  23:8  |   ro   |   0x0   | CAP_LENGTH | Capability Structure Length in DWORDs. |
|  7:0   |   ro   |  0x12   | CAP_ID     | Extended Capability ID                 |

## STBY_CR_CONTROL
Standby Controller Control
- Offset: `0x234`
- Reset default: `0x100c`
- Reset mask: `0xc010f73f`

### Fields

```wavejson
{"reg": [{"name": "PENDING_RX_NACK", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "HANDOFF_DELAY_NACK", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "ACR_FSM_OP_SELECT", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "PRIME_ACCEPT_GETACCR", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "HANDOFF_DEEP_SLEEP", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "CR_REQUEST_SEND", "bits": 1, "attr": ["rw"], "rotate": -90}, {"bits": 2}, {"name": "BCAST_CCC_IBI_RING", "bits": 3, "attr": ["rw"], "rotate": -90}, {"bits": 1}, {"name": "TARGET_XACT_ENABLE", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "DAA_SETAASA_ENABLE", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "DAA_SETDASA_ENABLE", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "DAA_ENTDAA_ENABLE", "bits": 1, "attr": ["rw"], "rotate": -90}, {"bits": 4}, {"name": "RSTACT_DEFBYTE_02", "bits": 1, "attr": ["rw"], "rotate": -90}, {"bits": 9}, {"name": "STBY_CR_ENABLE_INIT", "bits": 2, "attr": ["rw"], "rotate": -90}], "config": {"lanes": 1, "fontsize": 10, "vspace": 220}}
```

|  Bits  |  Type  |  Reset  | Name                 | Description                                         |
|:------:|:------:|:-------:|:---------------------|:----------------------------------------------------|
| 31:30  |   rw   |   0x0   | STBY_CR_ENABLE_INIT  | Host Controller Secondary Controller Enable.        |
| 29:21  |        |         |                      | Reserved                                            |
|   20   |   rw   |   0x0   | RSTACT_DEFBYTE_02    | RSTACT Support DefByte 0x02                         |
| 19:16  |        |         |                      | Reserved                                            |
|   15   |   rw   |   0x0   | DAA_ENTDAA_ENABLE    | Dynamic Address Method Enable.                      |
|   14   |   rw   |   0x0   | DAA_SETDASA_ENABLE   | Dynamic Address Method Enable.                      |
|   13   |   rw   |   0x0   | DAA_SETAASA_ENABLE   | Dynamic Address Method Enable.                      |
|   12   |   rw   |   0x1   | TARGET_XACT_ENABLE   | Target Transaction Interface Servicing Enable.      |
|   11   |        |         |                      | Reserved                                            |
|  10:8  |   rw   |   0x0   | BCAST_CCC_IBI_RING   | Ring Bundle IBI Selector for Broadcast CCC Capture. |
|  7:6   |        |         |                      | Reserved                                            |
|   5    |   rw   |   0x0   | CR_REQUEST_SEND      | Send Controller Role Request.                       |
|   4    |   rw   |   0x0   | HANDOFF_DEEP_SLEEP   | Handoff Deep Sleep.                                 |
|   3    |   rw   |   0x1   | PRIME_ACCEPT_GETACCR | Prime to Accept Controller Role.                    |
|   2    |   rw   |   0x1   | ACR_FSM_OP_SELECT    | Active Controller Select.                           |
|   1    |   rw   |   0x0   | HANDOFF_DELAY_NACK   | Handoff Delay NACK.                                 |
|   0    |   rw   |   0x0   | PENDING_RX_NACK      | Pending RX NACK.                                    |

## STBY_CR_DEVICE_ADDR
Standby Controller Device Address
- Offset: `0x238`
- Reset default: `0x0`
- Reset mask: `0x807f807f`

### Fields

```wavejson
{"reg": [{"name": "STATIC_ADDR", "bits": 7, "attr": ["rw"], "rotate": 0}, {"bits": 8}, {"name": "STATIC_ADDR_VALID", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "DYNAMIC_ADDR", "bits": 7, "attr": ["rw"], "rotate": 0}, {"bits": 8}, {"name": "DYNAMIC_ADDR_VALID", "bits": 1, "attr": ["rw"], "rotate": -90}], "config": {"lanes": 1, "fontsize": 10, "vspace": 200}}
```

|  Bits  |  Type  |  Reset  | Name               | Description               |
|:------:|:------:|:-------:|:-------------------|:--------------------------|
|   31   |   rw   |   0x0   | DYNAMIC_ADDR_VALID | Dynamic Address is Valid. |
| 30:23  |        |         |                    | Reserved                  |
| 22:16  |   rw   |   0x0   | DYNAMIC_ADDR       | Device Dynamic Address.   |
|   15   |   rw   |   0x0   | STATIC_ADDR_VALID  | Static Addres is Valid.   |
|  14:7  |        |         |                    | Reserved                  |
|  6:0   |   rw   |   0x0   | STATIC_ADDR        | Device Static Address.    |

## STBY_CR_CAPABILITIES
Standby Controller Capabilities
- Offset: `0x23c`
- Reset default: `0xf020`
- Reset mask: `0xf020`

### Fields

```wavejson
{"reg": [{"bits": 5}, {"name": "SIMPLE_CRR_SUPPORT", "bits": 1, "attr": ["ro"], "rotate": -90}, {"bits": 6}, {"name": "TARGET_XACT_SUPPORT", "bits": 1, "attr": ["ro"], "rotate": -90}, {"name": "DAA_SETAASA_SUPPORT", "bits": 1, "attr": ["ro"], "rotate": -90}, {"name": "DAA_SETDASA_SUPPORT", "bits": 1, "attr": ["ro"], "rotate": -90}, {"name": "DAA_ENTDAA_SUPPORT", "bits": 1, "attr": ["ro"], "rotate": -90}, {"bits": 16}], "config": {"lanes": 1, "fontsize": 10, "vspace": 210}}
```

|  Bits  |  Type  |  Reset  | Name                | Description                                           |
|:------:|:------:|:-------:|:--------------------|:------------------------------------------------------|
| 31:16  |        |         |                     | Reserved                                              |
|   15   |   ro   |   0x1   | DAA_ENTDAA_SUPPORT  | Dynamic Address Assignment with ENTDAA is supported.  |
|   14   |   ro   |   0x1   | DAA_SETDASA_SUPPORT | Dynamic Address Assignment with SETDASA is supported. |
|   13   |   ro   |   0x1   | DAA_SETAASA_SUPPORT | Dynamic Address Assignment with SETAASA is supported. |
|   12   |   ro   |   0x1   | TARGET_XACT_SUPPORT | I3C Target Transaction Interface is supported.        |
|  11:6  |        |         |                     | Reserved                                              |
|   5    |   ro   |   0x1   | SIMPLE_CRR_SUPPORT  | Simple Controller Role Request is supported.          |
|  4:0   |        |         |                     | Reserved                                              |

## STBY_CR_STATUS
Standby Controller Status
- Offset: `0x240`
- Reset default: `0x0`
- Reset mask: `0x1e2`

### Fields

```wavejson
{"reg": [{"bits": 1}, {"name": "AC_CURRENT_OWN", "bits": 1, "attr": ["rw"], "rotate": -90}, {"bits": 3}, {"name": "SIMPLE_CRR_STATUS", "bits": 3, "attr": ["rw"], "rotate": -90}, {"name": "HJ_REQ_STATUS", "bits": 1, "attr": ["rw"], "rotate": -90}, {"bits": 23}], "config": {"lanes": 1, "fontsize": 10, "vspace": 190}}
```

|  Bits  |  Type  |  Reset  | Name              | Description                            |
|:------:|:------:|:-------:|:------------------|:---------------------------------------|
|  31:9  |        |         |                   | Reserved                               |
|   8    |   rw   |   0x0   | HJ_REQ_STATUS     | Hot-Join Request Status.               |
|  7:5   |   rw   |   0x0   | SIMPLE_CRR_STATUS | Simple Controller Role Request Status. |
|  4:2   |        |         |                   | Reserved                               |
|   1    |   rw   |   0x0   | AC_CURRENT_OWN    | Active Controller.                     |

## STBY_CR_DEVICE_CHAR
Standby Controller Device Characteristics
- Offset: `0x244`
- Reset default: `0x60000000`
- Reset mask: `0xfffffffe`

### Fields

```wavejson
{"reg": [{"bits": 1}, {"name": "PID_HI", "bits": 15, "attr": ["rw"], "rotate": 0}, {"name": "DCR", "bits": 8, "attr": ["rw"], "rotate": 0}, {"name": "BCR_VAR", "bits": 5, "attr": ["rw"], "rotate": 0}, {"name": "BCR_FIXED", "bits": 3, "attr": ["ro"], "rotate": -90}], "config": {"lanes": 1, "fontsize": 10, "vspace": 110}}
```

|  Bits  |  Type  |  Reset  | Name      | Description                            |
|:------:|:------:|:-------:|:----------|:---------------------------------------|
| 31:29  |   ro   |   0x3   | BCR_FIXED | Bus Characteristics Register Fixed.    |
| 28:24  |   rw   |   0x0   | BCR_VAR   | Bus Characteristics Register Variable. |
| 23:16  |   rw   |   0x0   | DCR       | Device Characteristics Register.       |
|  15:1  |   rw   |   0x0   | PID_HI    | Device Provisional ID High.            |

## STBY_CR_DEVICE_PID_LO
Standby Controller PID Low
- Offset: `0x248`
- Reset default: `0x0`
- Reset mask: `0xffffffff`

### Fields

```wavejson
{"reg": [{"name": "PID_LO", "bits": 32, "attr": ["rw"], "rotate": 0}], "config": {"lanes": 1, "fontsize": 10, "vspace": 80}}
```

|  Bits  |  Type  |  Reset  | Name   | Description                |
|:------:|:------:|:-------:|:-------|:---------------------------|
|  31:0  |   rw   |   0x0   | PID_LO | Device Provisional ID Low. |

## STBY_CR_INTR_STATUS
Standby Controller Interrupt Status
- Offset: `0x24c`
- Reset default: `0x0`
- Reset mask: `0xf7c0f`

### Fields

```wavejson
{"reg": [{"name": "ACR_HANDOFF_OK_REMAIN_STAT", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "ACR_HANDOFF_OK_PRIMED_STAT", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "ACR_HANDOFF_ERR_FAIL_STAT", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "ACR_HANDOFF_ERR_M3_STAT", "bits": 1, "attr": ["rw"], "rotate": -90}, {"bits": 6}, {"name": "CRR_RESPONSE_STAT", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "STBY_CR_DYN_ADDR_STAT", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "STBY_CR_ACCEPT_NACKED_STAT", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "STBY_CR_ACCEPT_OK_STAT", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "STBY_CR_ACCEPT_ERR_STAT", "bits": 1, "attr": ["rw"], "rotate": -90}, {"bits": 1}, {"name": "STBY_CR_OP_RSTACT_STAT", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "CCC_PARAM_MODIFIED_STAT", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "CCC_UNHANDLED_NACK_STAT", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "CCC_FATAL_RSTDAA_ERR_STAT", "bits": 1, "attr": ["rw"], "rotate": -90}, {"bits": 12}], "config": {"lanes": 1, "fontsize": 10, "vspace": 280}}
```

|  Bits  |  Type  |  Reset  | Name                       | Description                                                      |
|:------:|:------:|:-------:|:---------------------------|:-----------------------------------------------------------------|
| 31:20  |        |         |                            | Reserved                                                         |
|   19   |   rw   |   0x0   | CCC_FATAL_RSTDAA_ERR_STAT  | CCC Fatal RSTDAA Error Status.                                   |
|   18   |   rw   |   0x0   | CCC_UNHANDLED_NACK_STAT    | CCC Unhandled NACK Status.                                       |
|   17   |   rw   |   0x0   | CCC_PARAM_MODIFIED_STAT    | CCC Parameter Modified Status.                                   |
|   16   |   rw   |   0x0   | STBY_CR_OP_RSTACT_STAT     | Secondary Controller Operation Reset Action.                     |
|   15   |        |         |                            | Reserved                                                         |
|   14   |   rw   |   0x0   | STBY_CR_ACCEPT_ERR_STAT    | Secondary Controller Transition Error Status.                    |
|   13   |   rw   |   0x0   | STBY_CR_ACCEPT_OK_STAT     | Secondary Controller Transition OK Status.                       |
|   12   |   rw   |   0x0   | STBY_CR_ACCEPT_NACKED_STAT | Secondary Controller Transition NACKed.                          |
|   11   |   rw   |   0x0   | STBY_CR_DYN_ADDR_STAT      | Secondary Controller Dynamic Address Status.                     |
|   10   |   rw   |   0x0   | CRR_RESPONSE_STAT          | Controller Role Request Response Status.                         |
|  9:4   |        |         |                            | Reserved                                                         |
|   3    |   rw   |   0x0   | ACR_HANDOFF_ERR_M3_STAT    | Controller Role Handoff Error Type CE3 Recovery.                 |
|   2    |   rw   |   0x0   | ACR_HANDOFF_ERR_FAIL_STAT  | Controller Role Handoff Error Due To Failure.                    |
|   1    |   rw   |   0x0   | ACR_HANDOFF_OK_PRIMED_STAT | Controller Role Handoff OK and Primed to Accept.                 |
|   0    |   rw   |   0x0   | ACR_HANDOFF_OK_REMAIN_STAT | Controller Role Handoff OK and Will Remain Secondary Controller. |

## STBY_CR_INTR_SIGNAL_ENABLE
Standby Controller Interrupt Signal Enable
- Offset: `0x250`
- Reset default: `0xc0000`
- Reset mask: `0xf7c0f`

### Fields

```wavejson
{"reg": [{"name": "ACR_HANDOFF_OK_REMAIN_SIGNAL_EN", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "ACR_HANDOFF_OK_PRIMED_SIGNAL_EN", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "ACR_HANDOFF_ERR_FAIL_SIGNAL_EN", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "ACR_HANDOFF_ERR_M3_SIGNAL_EN", "bits": 1, "attr": ["rw"], "rotate": -90}, {"bits": 6}, {"name": "CRR_RESPONSE_SIGNAL_EN", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "STBY_CR_DYN_ADDR_SIGNAL_EN", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "STBY_CR_ACCEPT_NACKED_SIGNAL_EN", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "STBY_CR_ACCEPT_OK_SIGNAL_EN", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "STBY_CR_ACCEPT_ERR_SIGNAL_EN", "bits": 1, "attr": ["rw"], "rotate": -90}, {"bits": 1}, {"name": "STBY_CR_OP_RSTACT_SIGNAL_EN", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "CCC_PARAM_MODIFIED_SIGNAL_EN", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "CCC_UNHANDLED_NACK_SIGNAL_EN", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "CCC_FATAL_RSTDAA_ERR_SIGNAL_EN", "bits": 1, "attr": ["rw"], "rotate": -90}, {"bits": 12}], "config": {"lanes": 1, "fontsize": 10, "vspace": 330}}
```

|  Bits  |  Type  |  Reset  | Name                            | Description                                 |
|:------:|:------:|:-------:|:--------------------------------|:--------------------------------------------|
| 31:20  |        |         |                                 | Reserved                                    |
|   19   |   rw   |   0x1   | CCC_FATAL_RSTDAA_ERR_SIGNAL_EN  | Standby Controller Interrupt Signal Enable. |
|   18   |   rw   |   0x1   | CCC_UNHANDLED_NACK_SIGNAL_EN    | Standby Controller Interrupt Signal Enable. |
|   17   |   rw   |   0x0   | CCC_PARAM_MODIFIED_SIGNAL_EN    | Standby Controller Interrupt Signal Enable. |
|   16   |   rw   |   0x0   | STBY_CR_OP_RSTACT_SIGNAL_EN     | Standby Controller Interrupt Signal Enable. |
|   15   |        |         |                                 | Reserved                                    |
|   14   |   rw   |   0x0   | STBY_CR_ACCEPT_ERR_SIGNAL_EN    | Standby Controller Interrupt Signal Enable. |
|   13   |   rw   |   0x0   | STBY_CR_ACCEPT_OK_SIGNAL_EN     | Standby Controller Interrupt Signal Enable. |
|   12   |   rw   |   0x0   | STBY_CR_ACCEPT_NACKED_SIGNAL_EN | Standby Controller Interrupt Signal Enable. |
|   11   |   rw   |   0x0   | STBY_CR_DYN_ADDR_SIGNAL_EN      | Standby Controller Interrupt Signal Enable. |
|   10   |   rw   |   0x0   | CRR_RESPONSE_SIGNAL_EN          | Standby Controller Interrupt Signal Enable. |
|  9:4   |        |         |                                 | Reserved                                    |
|   3    |   rw   |   0x0   | ACR_HANDOFF_ERR_M3_SIGNAL_EN    | Standby Controller Interrupt Signal Enable. |
|   2    |   rw   |   0x0   | ACR_HANDOFF_ERR_FAIL_SIGNAL_EN  | Standby Controller Interrupt Signal Enable. |
|   1    |   rw   |   0x0   | ACR_HANDOFF_OK_PRIMED_SIGNAL_EN | Standby Controller Interrupt Signal Enable. |
|   0    |   rw   |   0x0   | ACR_HANDOFF_OK_REMAIN_SIGNAL_EN | Standby Controller Interrupt Signal Enable. |

## STBY_CR_INTR_FORCE
Standby Controller Interrupt Force
- Offset: `0x254`
- Reset default: `0x0`
- Reset mask: `0xf7c00`

### Fields

```wavejson
{"reg": [{"bits": 10}, {"name": "CRR_RESPONSE_FORCE", "bits": 1, "attr": ["wo"], "rotate": -90}, {"name": "STBY_CR_DYN_ADDR_FORCE", "bits": 1, "attr": ["wo"], "rotate": -90}, {"name": "STBY_CR_ACCEPT_NACKED_FORCE", "bits": 1, "attr": ["wo"], "rotate": -90}, {"name": "STBY_CR_ACCEPT_OK_FORCE", "bits": 1, "attr": ["wo"], "rotate": -90}, {"name": "STBY_CR_ACCEPT_ERR_FORCE", "bits": 1, "attr": ["wo"], "rotate": -90}, {"bits": 1}, {"name": "STBY_CR_OP_RSTACT_FORCE", "bits": 1, "attr": ["wo"], "rotate": -90}, {"name": "CCC_PARAM_MODIFIED_FORCE", "bits": 1, "attr": ["wo"], "rotate": -90}, {"name": "CCC_UNHANDLED_NACK_FORCE", "bits": 1, "attr": ["wo"], "rotate": -90}, {"name": "CCC_FATAL_RSTDAA_ERR_FORCE", "bits": 1, "attr": ["wo"], "rotate": -90}, {"bits": 12}], "config": {"lanes": 1, "fontsize": 10, "vspace": 290}}
```

|  Bits  |  Type  |  Reset  | Name                        | Description                         |
|:------:|:------:|:-------:|:----------------------------|:------------------------------------|
| 31:20  |        |         |                             | Reserved                            |
|   19   |   wo   |   0x0   | CCC_FATAL_RSTDAA_ERR_FORCE  | Standby Controller Interrupt Force. |
|   18   |   wo   |   0x0   | CCC_UNHANDLED_NACK_FORCE    | Standby Controller Interrupt Force. |
|   17   |   wo   |   0x0   | CCC_PARAM_MODIFIED_FORCE    | Standby Controller Interrupt Force. |
|   16   |   wo   |   0x0   | STBY_CR_OP_RSTACT_FORCE     | Standby Controller Interrupt Force. |
|   15   |        |         |                             | Reserved                            |
|   14   |   wo   |   0x0   | STBY_CR_ACCEPT_ERR_FORCE    | Standby Controller Interrupt Force. |
|   13   |   wo   |   0x0   | STBY_CR_ACCEPT_OK_FORCE     | Standby Controller Interrupt Force. |
|   12   |   wo   |   0x0   | STBY_CR_ACCEPT_NACKED_FORCE | Standby Controller Interrupt Force. |
|   11   |   wo   |   0x0   | STBY_CR_DYN_ADDR_FORCE      | Standby Controller Interrupt Force. |
|   10   |   wo   |   0x0   | CRR_RESPONSE_FORCE          | Standby Controller Interrupt Force. |
|  9:0   |        |         |                             | Reserved                            |

## STBY_CR_CCC_CONFIG_GETCAPS
Standby Controller CCC Auto-Response Config Get Capabilities
- Offset: `0x258`
- Reset default: `0x0`
- Reset mask: `0x17`

### Fields

```wavejson
{"reg": [{"name": "F2_CRCAP1_BUS_CONFIG", "bits": 3, "attr": ["rw"], "rotate": -90}, {"bits": 1}, {"name": "F2_CRCAP2_DEV_INTERACT", "bits": 1, "attr": ["rw"], "rotate": -90}, {"bits": 27}], "config": {"lanes": 1, "fontsize": 10, "vspace": 240}}
```

|  Bits  |  Type  |  Reset  | Name                   | Description                |
|:------:|:------:|:-------:|:-----------------------|:---------------------------|
|  31:5  |        |         |                        | Reserved                   |
|   4    |   rw   |   0x0   | F2_CRCAP2_DEV_INTERACT | GETCAPS CCC CRCAPS Byte 2. |
|   3    |        |         |                        | Reserved                   |
|  2:0   |   rw   |   0x0   | F2_CRCAP1_BUS_CONFIG   | GETCAPS CCC CRCAPS Byte 1. |

## STBY_CR_CCC_CONFIG_RSTACT_PARAMS
Standby Controller CCC Auto-Response Config Get Capabilities
- Offset: `0x25c`
- Reset default: `0x80000000`
- Reset mask: `0x80ffffff`

### Fields

```wavejson
{"reg": [{"name": "RST_ACTION", "bits": 8, "attr": ["rw"], "rotate": 0}, {"name": "RESET_TIME_PERIPHERAL", "bits": 8, "attr": ["rw"], "rotate": -90}, {"name": "RESET_TIME_TARGET", "bits": 8, "attr": ["rw"], "rotate": -90}, {"bits": 7}, {"name": "RESET_DYNAMIC_ADDR", "bits": 1, "attr": ["rw"], "rotate": -90}], "config": {"lanes": 1, "fontsize": 10, "vspace": 230}}
```

|  Bits  |  Type  |  Reset  | Name                  | Description                               |
|:------:|:------:|:-------:|:----------------------|:------------------------------------------|
|   31   |   rw   |   0x1   | RESET_DYNAMIC_ADDR    | Reset Dynamic Address After Target Reset. |
| 30:24  |        |         |                       | Reserved                                  |
| 23:16  |   rw   |   0x0   | RESET_TIME_TARGET     | Time to Reset Target.                     |
|  15:8  |   rw   |   0x0   | RESET_TIME_PERIPHERAL | Time to Reset Peripheral.                 |
|  7:0   |   rw   |   0x0   | RST_ACTION            | Defining Byte of the RSTACT CCC.          |

## DAT
Device Address Table.

- Word Aligned Offset Range: `0x300`to`0x3fc`
- Size (words): `64`
- Access: `rw`
- Byte writes are *not* supported.

## DCT
Device Characteristics Table.

- Word Aligned Offset Range: `0x400`to`0x5fc`
- Size (words): `128`
- Access: `rw`
- Byte writes are *not* supported.

## BUFFER
Software-managed 2KiB message buffer used for transmitting and receiving messages.

- Word Aligned Offset Range: `0x800`to`0xffc`
- Size (words): `512`
- Access: `rw`
- Byte writes are  supported.

