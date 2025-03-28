// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include "sw/device/lib/dif/dif_rstmgr.h"

#include <assert.h>
#include <stdint.h>

#include "sw/device/lib/base/bitfield.h"
#include "sw/device/lib/base/macros.h"
#include "sw/device/lib/base/mmio.h"
#include "sw/device/lib/base/multibits.h"
#include "sw/device/lib/dif/dif_base.h"

#include "rstmgr_regs.h"  // Generated.

// These assertions are only defined for the Earl Grey chip.
#if defined(OPENTITAN_IS_EARLGREY)
// This macro simplifies the `static_assert` check to make sure that the
// public reset info register bitfield matches register bits.
#define RSTMGR_RESET_INFO_CHECK(pub_name, priv_name)         \
  static_assert(kDifRstmgrResetInfo##pub_name ==             \
                    (0x1 << RSTMGR_RESET_##priv_name##_BIT), \
                "kDifRstmgrResetInfo" #pub_name              \
                " must match the register definition!")

RSTMGR_RESET_INFO_CHECK(Por, INFO_POR);
RSTMGR_RESET_INFO_CHECK(LowPowerExit, INFO_LOW_POWER_EXIT);

static_assert(kDifRstmgrResetInfoHwReq == (RSTMGR_RESET_INFO_HW_REQ_MASK
                                           << RSTMGR_RESET_INFO_HW_REQ_OFFSET),
              "kDifRstmgrResetInfoHwReq must match the register definition!");

static_assert(
    RSTMGR_PARAM_NUM_SW_RESETS == 8,
    "Number of software resets has changed, please update this file!");

// The Reset Manager implementation will have to be updated if the number
// of software resets grows, as it would span across multiple registers, so
// there will be multiple of Reset Enable and Reset Control registers. The
// appropriate offset from the peripheral base would then have to be
// calculated.
static_assert(
    RSTMGR_PARAM_NUM_SW_RESETS <= 32,
    "Reset Enable and Control registers span across multiple registers!");

// Make sure that the public alert info crash dump size matches the HW.
// Note that `RSTMGR_ALERT_INFO_CTRL_INDEX_MASK` implies 16 indexes ( 0 - 15
// inclusive). However, in reality it only supports 15, as
// `RSTMGR_ALERT_INFO_ATTR_CNT_AVAIL_MASK` is of the same size, but value of
// 0 indicates that there is no alert info crash dump.
static_assert(
    DIF_RSTMGR_ALERT_INFO_MAX_SIZE == RSTMGR_ALERT_INFO_CTRL_INDEX_MASK,
    "Alert info dump max size has grown, please update the public define!");
#elif defined(OPENTITAN_IS_DARJEELING)
// TODO: equivalent assertations are not yet defined for Darjeeling
#else
#error "dif_rstmgr does not support this top"
#endif

/**
 * Checks whether alert_info capture is disabled.
 */
static bool alert_capture_is_locked(mmio_region_t base_addr) {
  uint32_t bitfield =
      mmio_region_read32(base_addr, RSTMGR_ALERT_REGWEN_REG_OFFSET);

  // When bit is cleared, alert capture is disabled.
  return !bitfield_bit32_read(bitfield, RSTMGR_ALERT_REGWEN_EN_BIT);
}

/**
 * Checks whether CPU info capture is disabled.
 */
static bool cpu_capture_is_locked(mmio_region_t base_addr) {
  uint32_t bitfield =
      mmio_region_read32(base_addr, RSTMGR_CPU_REGWEN_REG_OFFSET);

  // When bit is cleared, APU capture is disabled.
  return !bitfield_bit32_read(bitfield, RSTMGR_CPU_REGWEN_EN_BIT);
}

/**
 * Checks whether the software reset is disabled for a `peripheral`.
 */
static bool rstmgr_software_reset_is_locked(
    mmio_region_t base_addr, dif_rstmgr_peripheral_t peripheral) {
  return !mmio_region_read32(
      base_addr, RSTMGR_SW_RST_REGWEN_0_REG_OFFSET + 4 * (ptrdiff_t)peripheral);
}

/**
 * Holds or releases a `peripheral` in/from the reset state.
 */
static void rstmgr_software_reset_hold(mmio_region_t base_addr,
                                       dif_rstmgr_peripheral_t peripheral,
                                       bool hold) {
  bool value = hold ? false : true;
  mmio_region_write32(
      base_addr, RSTMGR_SW_RST_CTRL_N_0_REG_OFFSET + 4 * (ptrdiff_t)peripheral,
      value);
}

/**
 * Clears entire reset info register.
 *
 * Normal "Power On Reset" cause is also cleared. Set bit to clear.
 */
static void rstmgr_reset_info_clear(mmio_region_t base_addr) {
  mmio_region_write32(base_addr, RSTMGR_RESET_INFO_REG_OFFSET, UINT32_MAX);
}

dif_result_t dif_rstmgr_reset(const dif_rstmgr_t *handle) {
  if (handle == NULL) {
    return kDifBadArg;
  }

  mmio_region_t base_addr = handle->base_addr;

  rstmgr_reset_info_clear(base_addr);

  // Set bits to stop holding all peripherals in the reset state.
  for (uint32_t i = 0; i < RSTMGR_PARAM_NUM_SW_RESETS; i++) {
    mmio_region_write32(base_addr,
                        RSTMGR_SW_RST_CTRL_N_0_REG_OFFSET + (ptrdiff_t)i * 4,
                        UINT32_MAX);
  }

  return kDifOk;
}

dif_result_t dif_rstmgr_reset_lock(const dif_rstmgr_t *handle,
                                   dif_rstmgr_peripheral_t peripheral) {
  if (handle == NULL || peripheral >= RSTMGR_PARAM_NUM_SW_RESETS) {
    return kDifBadArg;
  }

  mmio_region_t base_addr = handle->base_addr;

  mmio_region_write32(
      base_addr, RSTMGR_SW_RST_REGWEN_0_REG_OFFSET + 4 * (ptrdiff_t)peripheral,
      0);

  return kDifOk;
}

dif_result_t dif_rstmgr_reset_is_locked(const dif_rstmgr_t *handle,
                                        dif_rstmgr_peripheral_t peripheral,
                                        bool *is_locked) {
  if (handle == NULL || is_locked == NULL ||
      peripheral >= RSTMGR_PARAM_NUM_SW_RESETS) {
    return kDifBadArg;
  }

  mmio_region_t base_addr = handle->base_addr;
  *is_locked = rstmgr_software_reset_is_locked(base_addr, peripheral);

  return kDifOk;
}

dif_result_t dif_rstmgr_reset_info_get(const dif_rstmgr_t *handle,
                                       dif_rstmgr_reset_info_bitfield_t *info) {
  if (handle == NULL || info == NULL) {
    return kDifBadArg;
  }

  mmio_region_t base_addr = handle->base_addr;
  *info = mmio_region_read32(base_addr, RSTMGR_RESET_INFO_REG_OFFSET);

  return kDifOk;
}

dif_result_t dif_rstmgr_reset_info_clear(const dif_rstmgr_t *handle) {
  if (handle == NULL) {
    return kDifBadArg;
  }

  mmio_region_t base_addr = handle->base_addr;

  rstmgr_reset_info_clear(base_addr);

  return kDifOk;
}

dif_result_t dif_rstmgr_alert_info_set_enabled(const dif_rstmgr_t *handle,
                                               dif_toggle_t state) {
  if (handle == NULL) {
    return kDifBadArg;
  }

  mmio_region_t base_addr = handle->base_addr;

  if (alert_capture_is_locked(base_addr)) {
    return kDifLocked;
  }

  uint32_t enabled = (state == kDifToggleEnabled) ? 0x1 : 0x0;

  // This will clobber the `ALERT_INFO_CTRL.INDEX` field. However, the index
  // field is only relevant during the crash dump read operation, and is
  // set by the caller and not the hardware, so it is safe to clobber it.
  mmio_region_write32(base_addr, RSTMGR_ALERT_INFO_CTRL_REG_OFFSET, enabled);

  return kDifOk;
}

dif_result_t dif_rstmgr_alert_info_get_enabled(const dif_rstmgr_t *handle,
                                               dif_toggle_t *state) {
  if (handle == NULL || state == NULL) {
    return kDifBadArg;
  }

  mmio_region_t base_addr = handle->base_addr;

  uint32_t reg =
      mmio_region_read32(base_addr, RSTMGR_ALERT_INFO_CTRL_REG_OFFSET);
  bool enabled = bitfield_bit32_read(reg, RSTMGR_ALERT_INFO_CTRL_EN_BIT);

  *state = enabled ? kDifToggleEnabled : kDifToggleDisabled;

  return kDifOk;
}

dif_result_t dif_rstmgr_alert_info_get_size(const dif_rstmgr_t *handle,
                                            size_t *size) {
  if (handle == NULL || size == NULL) {
    return kDifBadArg;
  }

  mmio_region_t base_addr = handle->base_addr;
  *size = mmio_region_read32(base_addr, RSTMGR_ALERT_INFO_ATTR_REG_OFFSET);
  return kDifOk;
}

dif_result_t dif_rstmgr_alert_info_dump_read(
    const dif_rstmgr_t *handle, dif_rstmgr_alert_info_dump_segment_t *dump,
    size_t dump_size, size_t *segments_read) {
  if (handle == NULL || dump == NULL || segments_read == NULL) {
    return kDifBadArg;
  }

  mmio_region_t base_addr = handle->base_addr;

  // The actual crash dump size (can be smaller than `dump_size`).
  size_t dump_size_actual =
      mmio_region_read32(base_addr, RSTMGR_ALERT_INFO_ATTR_REG_OFFSET);

  // Partial crash dump read is not allowed.
  if (dump_size < dump_size_actual) {
    return kDifError;
  }

  uint32_t control_reg =
      mmio_region_read32(base_addr, RSTMGR_ALERT_INFO_CTRL_REG_OFFSET);

  // Read the entire alert info crash dump, one 32bit data segment at the time.
  for (uint32_t i = 0; i < dump_size_actual; ++i) {
    control_reg = bitfield_field32_write(control_reg,
                                         RSTMGR_ALERT_INFO_CTRL_INDEX_FIELD, i);

    // Set the index of the 32bit data segment to be read at `i`.
    mmio_region_write32(base_addr, RSTMGR_ALERT_INFO_CTRL_REG_OFFSET,
                        control_reg);

    // Read the alert info crash dump 32bit data segment.
    dump[i] = mmio_region_read32(base_addr, RSTMGR_ALERT_INFO_REG_OFFSET);
  }

  *segments_read = dump_size_actual;

  return kDifOk;
}

dif_result_t dif_rstmgr_cpu_info_set_enabled(const dif_rstmgr_t *handle,
                                             dif_toggle_t state) {
  if (handle == NULL) {
    return kDifBadArg;
  }

  mmio_region_t base_addr = handle->base_addr;

  if (cpu_capture_is_locked(base_addr)) {
    return kDifLocked;
  }

  uint32_t enabled = (state == kDifToggleEnabled) ? 0x1 : 0x0;

  // This will clobber the `CPU_INFO_CTRL.INDEX` field. However, the index
  // field is only relevant during the crash dump read operation, and is
  // set by the caller and not the hardware, so it is safe to clobber it.
  mmio_region_write32(base_addr, RSTMGR_CPU_INFO_CTRL_REG_OFFSET, enabled);

  return kDifOk;
}

dif_result_t dif_rstmgr_cpu_info_get_enabled(const dif_rstmgr_t *handle,
                                             dif_toggle_t *state) {
  if (handle == NULL || state == NULL) {
    return kDifBadArg;
  }

  mmio_region_t base_addr = handle->base_addr;

  uint32_t reg = mmio_region_read32(base_addr, RSTMGR_CPU_INFO_CTRL_REG_OFFSET);
  bool enabled = bitfield_bit32_read(reg, RSTMGR_CPU_INFO_CTRL_EN_BIT);

  *state = enabled ? kDifToggleEnabled : kDifToggleDisabled;

  return kDifOk;
}

dif_result_t dif_rstmgr_cpu_info_get_size(const dif_rstmgr_t *handle,
                                          size_t *size) {
  if (handle == NULL || size == NULL) {
    return kDifBadArg;
  }

  mmio_region_t base_addr = handle->base_addr;
  *size = mmio_region_read32(base_addr, RSTMGR_CPU_INFO_ATTR_REG_OFFSET);
  return kDifOk;
}

dif_result_t dif_rstmgr_cpu_info_dump_read(
    const dif_rstmgr_t *handle, dif_rstmgr_cpu_info_dump_segment_t *dump,
    size_t dump_size, size_t *segments_read) {
  if (handle == NULL || dump == NULL || segments_read == NULL) {
    return kDifBadArg;
  }

  mmio_region_t base_addr = handle->base_addr;

  // The actual crash dump size (can be smaller than `dump_size`).
  size_t dump_size_actual =
      mmio_region_read32(base_addr, RSTMGR_CPU_INFO_ATTR_REG_OFFSET);

  // Partial crash dump read is not allowed.
  if (dump_size < dump_size_actual) {
    return kDifError;
  }

  uint32_t control_reg =
      mmio_region_read32(base_addr, RSTMGR_CPU_INFO_CTRL_REG_OFFSET);

  // Read the entire cpu info crash dump, one 32bit data segment at the time.
  for (uint32_t i = 0; i < dump_size_actual; ++i) {
    control_reg = bitfield_field32_write(control_reg,
                                         RSTMGR_CPU_INFO_CTRL_INDEX_FIELD, i);

    // Set the index of the 32bit data segment to be read at `i`.
    mmio_region_write32(base_addr, RSTMGR_CPU_INFO_CTRL_REG_OFFSET,
                        control_reg);

    // Read the cpu info crash dump 32bit data segment.
    dump[i] = mmio_region_read32(base_addr, RSTMGR_CPU_INFO_REG_OFFSET);
  }

  *segments_read = dump_size_actual;

  return kDifOk;
}

dif_result_t dif_rstmgr_software_reset(const dif_rstmgr_t *handle,
                                       dif_rstmgr_peripheral_t peripheral,
                                       dif_rstmgr_software_reset_t reset) {
  if (handle == NULL || peripheral >= RSTMGR_PARAM_NUM_SW_RESETS) {
    return kDifBadArg;
  }

  mmio_region_t base_addr = handle->base_addr;
  if (rstmgr_software_reset_is_locked(base_addr, peripheral)) {
    return kDifLocked;
  }

  switch (reset) {
    case kDifRstmgrSoftwareReset:
      rstmgr_software_reset_hold(base_addr, peripheral, true);
      rstmgr_software_reset_hold(base_addr, peripheral, false);
      break;
    case kDifRstmgrSoftwareResetHold:
      rstmgr_software_reset_hold(base_addr, peripheral, true);
      break;
    case kDifRstmgrSoftwareResetRelease:
      rstmgr_software_reset_hold(base_addr, peripheral, false);
      break;
    default:
      return kDifError;
  }

  return kDifOk;
}

dif_result_t dif_rstmgr_software_reset_is_held(
    const dif_rstmgr_t *handle, dif_rstmgr_peripheral_t peripheral,
    bool *asserted) {
  if (handle == NULL || asserted == NULL ||
      peripheral >= RSTMGR_PARAM_NUM_SW_RESETS) {
    return kDifBadArg;
  }

  // When the bit is cleared - peripheral is held in reset.
  *asserted =
      !mmio_region_read32(handle->base_addr, RSTMGR_SW_RST_CTRL_N_0_REG_OFFSET +
                                                 4 * (ptrdiff_t)peripheral);

  return kDifOk;
}

dif_result_t dif_rstmgr_software_device_reset(const dif_rstmgr_t *handle) {
  if (handle == NULL) {
    return kDifBadArg;
  }

  mmio_region_write32(handle->base_addr, RSTMGR_RESET_REQ_REG_OFFSET,
                      kMultiBitBool4True);

  return kDifOk;
}

dif_result_t dif_rstmgr_get_sw_reset_index(dt_rstmgr_t dt, dt_reset_t reset,
                                           size_t *sw_rst_idx) {
  size_t sw_reset_count = dt_rstmgr_sw_reset_count(dt);
  for (*sw_rst_idx = 0; *sw_rst_idx < sw_reset_count; ++(*sw_rst_idx)) {
    if (dt_rstmgr_sw_reset(dt, *sw_rst_idx) == reset) {
      return kDifOk;
    }
  }
  return kDifBadArg;
}

dif_result_t dif_rstmgr_fatal_err_code_get_codes(
    const dif_rstmgr_t *rstmgr, dif_rstmgr_fatal_err_codes_t *codes) {
  if (rstmgr == NULL || codes == NULL) {
    return kDifBadArg;
  }
  *codes = mmio_region_read32(rstmgr->base_addr, RSTMGR_ERR_CODE_REG_OFFSET);
  return kDifOk;
}
