# I3C Controller/Target

# Overview

This document specifies the I3C Controller/Target IP functionality.

## Features

The IP block implements the following features:

- I3C Controller (Primary and/or Secondary).
- I3C Target.
- Host Controller Interface (HCI) in PIO mode.
- Signaling modes:
  - HDR-DDR (I3C Double Data Rate, 25Mbps).
  - SDR (I3C Single Data Rate, 12.5Mbps).
  - I2C Fast Mode and Fast Mode Plus for backwards compatibility with some I2C devices
- 2KiB data transfer buffer.
- 

## Description

## Compatibility

The IP block implements the following specifications:

- MIPI Alliance Specification for I3C Host Controller Interface (I3C HCI) Version 1.2.
- MIPI Alliance Specification for I3C Transfer Command Response (I3C TCRI) Version 1.0.
- MIPI Alliance Specification for I3C Basic (I3C Basic) Version 1.2.

