// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include "sw/device/lib/base/abs_mmio.h"
#include "sw/device/lib/base/memory.h"
#include "sw/device/lib/dif/dif_aon_timer.h"
#include "sw/device/lib/dif/dif_base.h"
#include "sw/device/lib/dif/dif_clkmgr.h"
#include "sw/device/lib/dif/dif_pwrmgr.h"
#include "sw/device/lib/dif/dif_rstmgr.h"
#include "sw/device/lib/dif/dif_rv_core_ibex.h"
#include "sw/device/lib/dif/dif_spi_host.h"
#include "sw/device/lib/dif/dif_uart.h"
#include "sw/device/lib/dif/dif_usbdev.h"
#include "sw/device/lib/runtime/log.h"
#include "sw/device/lib/testing/aon_timer_testutils.h"
#include "sw/device/lib/testing/ret_sram_testutils.h"
#include "sw/device/lib/testing/rstmgr_testutils.h"
#include "sw/device/lib/testing/test_framework/check.h"
#include "sw/device/lib/testing/test_framework/ottf_main.h"

#include "hw/top_earlgrey/sw/autogen/top_earlgrey.h"
#include "spi_host_regs.h"
#include "uart_regs.h"
#include "usbdev_regs.h"

static const dt_pwrmgr_t kPwrmgrDt = 0;
static_assert(kDtPwrmgrCount == 1, "this test expects a pwrmgr");

/**
 * The peripherals used to test when the peri clocks are disabled are
 * bit 0: clk_io_div4_peri: uart0
 * bit 1: clk_io_div2_peri: spi_host1
 * bit 2: clk_io_peri: spi_host0
 * bit 3: clk_usb_peri: usbdev
 */

OTTF_DEFINE_TEST_CONFIG();

typedef struct peri_context {
  const char *peripheral_name;  // The name of the peripheral tested.
  void (*csr_access)(void);     // The function causing a timeout.
  uint32_t address;             // The address causing a timeout.
} peri_context_t;

/**
 * The maximum offset past the entry point of the function that causes
 * the cpu to hang.
 */
enum { kPcSpread = 8 * 4 };

static dif_aon_timer_t aon_timer;
static dif_spi_host_t spi_host0;
static dif_spi_host_t spi_host1;
static dif_usbdev_t usbdev;
static dif_uart_t uart0;

OT_NOINLINE static void uart0_csr_access(void) {
  dif_uart_irq_state_snapshot_t snapshot;
  CHECK_DIF_OK(dif_uart_irq_get_state(&uart0, &snapshot));
}

OT_NOINLINE static void spi_host0_csr_access(void) {
  dif_spi_host_irq_state_snapshot_t snapshot;
  CHECK_DIF_OK(dif_spi_host_irq_get_state(&spi_host0, &snapshot));
}

OT_NOINLINE static void spi_host1_csr_access(void) {
  CHECK_DIF_OK(dif_spi_host_output_set_enabled(&spi_host1, true));
}

OT_NOINLINE static void usbdev_csr_access(void) {
  dif_usbdev_irq_state_snapshot_t snapshot;
  CHECK_DIF_OK(dif_usbdev_irq_get_state(&usbdev, &snapshot));
}

peri_context_t peri_context[kTopEarlgreyGateableClocksLast + 1] = {
    {"uart0", uart0_csr_access,
     TOP_EARLGREY_UART0_BASE_ADDR + UART_INTR_STATE_REG_OFFSET},
    {"spi_host1", spi_host1_csr_access,
     TOP_EARLGREY_SPI_HOST1_BASE_ADDR + SPI_HOST_CONTROL_REG_OFFSET},
    {"spi_host0", spi_host0_csr_access,
     TOP_EARLGREY_SPI_HOST0_BASE_ADDR + SPI_HOST_INTR_STATE_REG_OFFSET},
    {"usbdev", usbdev_csr_access,
     TOP_EARLGREY_USBDEV_BASE_ADDR + USBDEV_INTR_STATE_REG_OFFSET}};

/**
 * Test that disabling a 'gateable' unit's clock causes the unit to become
 * unresponsive to CSR accesses. Configure a watchdog reset, and if it triggers
 * the test is successful.
 */
static void test_gateable_clocks_off(const dif_clkmgr_t *clkmgr,
                                     const dif_pwrmgr_t *pwrmgr,
                                     dif_clkmgr_gateable_clock_t clock) {
  // Make sure the clock for the unit is on.
  CHECK_DIF_OK(
      dif_clkmgr_gateable_clock_set_enabled(clkmgr, clock, kDifToggleEnabled));
  // Enable watchdog bite reset.
  dif_pwrmgr_request_sources_t reset_sources;
  CHECK_DIF_OK(dif_pwrmgr_find_request_source(
      pwrmgr, kDifPwrmgrReqTypeReset, dt_aon_timer_instance_id(kDtAonTimerAon),
      kDtAonTimerResetReqAonTimer, &reset_sources));
  CHECK_DIF_OK(dif_pwrmgr_set_request_sources(
      pwrmgr, kDifPwrmgrReqTypeReset, reset_sources, kDifToggleEnabled));
  LOG_INFO("Testing peripheral clock %d, with unit %s", clock,
           peri_context[clock].peripheral_name);

  // Bite after enough time has elapsed past the hung csr access.
  uint32_t bite_us = (kDeviceType == kDeviceSimDV) ? 400 : 800;
  uint32_t bite_cycles = 0;
  CHECK_STATUS_OK(
      aon_timer_testutils_get_aon_cycles_32_from_us(bite_us, &bite_cycles));
  LOG_INFO("Setting bite reset for %u us (%u cycles)", bite_us, bite_cycles);

  // Make sure the CSR is accessible before turning the clock off.
  (*peri_context[clock].csr_access)();
  LOG_INFO("CSR access was okay before disabling the clock");

  // Set bite timer.
  CHECK_STATUS_OK(aon_timer_testutils_watchdog_config(&aon_timer, UINT32_MAX,
                                                      bite_cycles, false));
  // Disable the peripheral's clock.
  CHECK_DIF_OK(
      dif_clkmgr_gateable_clock_set_enabled(clkmgr, clock, kDifToggleDisabled));
  // Wait for the clock to really turn off.
  busy_spin_micros(100);
  // And issue the CSR access that will freeze and cause a reset.
  (*peri_context[clock].csr_access)();
}

bool test_main(void) {
  dif_clkmgr_t clkmgr;
  dif_pwrmgr_t pwrmgr;
  dif_rstmgr_t rstmgr;

  CHECK_DIF_OK(dif_rstmgr_init(
      mmio_region_from_addr(TOP_EARLGREY_RSTMGR_AON_BASE_ADDR), &rstmgr));

  CHECK_DIF_OK(dif_clkmgr_init(
      mmio_region_from_addr(TOP_EARLGREY_CLKMGR_AON_BASE_ADDR), &clkmgr));

  CHECK_DIF_OK(dif_pwrmgr_init_from_dt(kPwrmgrDt, &pwrmgr));

  // Initialize aon timer.
  CHECK_DIF_OK(dif_aon_timer_init(
      mmio_region_from_addr(TOP_EARLGREY_AON_TIMER_AON_BASE_ADDR), &aon_timer));

  // Initialize peripherals.
  CHECK_DIF_OK(dif_uart_init(
      mmio_region_from_addr(TOP_EARLGREY_UART0_BASE_ADDR), &uart0));
  CHECK_DIF_OK(dif_spi_host_init(
      mmio_region_from_addr(TOP_EARLGREY_SPI_HOST0_BASE_ADDR), &spi_host0));
  CHECK_DIF_OK(dif_spi_host_init(
      mmio_region_from_addr(TOP_EARLGREY_SPI_HOST1_BASE_ADDR), &spi_host1));
  CHECK_DIF_OK(dif_usbdev_init(
      mmio_region_from_addr(TOP_EARLGREY_USBDEV_BASE_ADDR), &usbdev));

  // Initialize the retention sram utils.
  ret_sram_testutils_init();

  // Enable cpu dump capture.
  CHECK_DIF_OK(dif_rstmgr_cpu_info_set_enabled(&rstmgr, kDifToggleEnabled));
  if (UNWRAP(rstmgr_testutils_is_reset_info(&rstmgr, kDifRstmgrResetInfoPor))) {
    CHECK_STATUS_OK(rstmgr_testutils_pre_reset(&rstmgr));

    // Starting clock.
    dif_clkmgr_gateable_clock_t clock = kTopEarlgreyGateableClocksIoDiv4Peri;
    uint32_t prev_value = 0;
    uint32_t value = 0;
    CHECK_STATUS_OK(ret_sram_testutils_counter_clear(0));
    for (int i = 0; i < 20; ++i) {
      CHECK_STATUS_OK(ret_sram_testutils_counter_increment(0));
      CHECK_STATUS_OK(ret_sram_testutils_counter_get(0, &value));
      CHECK(value == prev_value + 1);
      prev_value = value;
    }
    CHECK_STATUS_OK(ret_sram_testutils_counter_clear(0));
    CHECK_STATUS_OK(ret_sram_testutils_counter_get(0, &value));

    test_gateable_clocks_off(&clkmgr, &pwrmgr, clock);

    // This should never be reached.
    LOG_ERROR("This is unreachable since a reset should have been triggered");
    return false;
  } else if (UNWRAP(rstmgr_testutils_is_reset_info(
                 &rstmgr, kDifRstmgrResetInfoWatchdog))) {
    dif_clkmgr_gateable_clock_t clock = {0};
    CHECK_STATUS_OK(ret_sram_testutils_counter_get(0, &clock));
    LOG_INFO("Got an expected watchdog reset when reading for clock %d", clock);

    size_t actual_size;
    CHECK_DIF_OK(dif_rstmgr_cpu_info_get_size(&rstmgr, &actual_size));
    dif_rv_core_ibex_crash_dump_info_t crash_dump;
    // The sizes for dif_rstmgr_cpu_info_dump_read are measured in
    // units of dif_rstmgr_cpu_info_dump_segment_t.
    size_t size_read;
    CHECK_DIF_OK(dif_rstmgr_cpu_info_dump_read(
        &rstmgr, (dif_rstmgr_cpu_info_dump_segment_t *)&crash_dump,
        sizeof(crash_dump) / sizeof(dif_rstmgr_cpu_info_dump_segment_t),
        &size_read));
    CHECK(size_read == actual_size);
    uint32_t expected_hung_address = peri_context[clock].address;
    CHECK(crash_dump.fault_state.mdaa == expected_hung_address,
          "Unexpected hung address for clock %d, via peripheral %s", clock,
          peri_context[clock].peripheral_name);
    // Check the fault PC is close enough to the address of the function
    // that causes the hung access.
    uint32_t crash_function = (uint32_t)peri_context[clock].csr_access;
    CHECK(crash_dump.fault_state.mcpc >= crash_function &&
              crash_dump.fault_state.mcpc <= crash_function + kPcSpread,
          "The crash PC 0x%x is too far from the expected 0x%x",
          crash_dump.fault_state.mcpc, crash_function);
    // Mark this clock as tested.
    LOG_INFO("Expectations are okay for clock %d, with peripheral %s", clock,
             peri_context[clock].peripheral_name);
    CHECK_STATUS_OK(ret_sram_testutils_counter_increment(0));

    if (clock < kTopEarlgreyGateableClocksLast) {
      CHECK_STATUS_OK(ret_sram_testutils_counter_get(0, &clock));
      LOG_INFO("Next clock to test %d", clock);

      CHECK_STATUS_OK(rstmgr_testutils_pre_reset(&rstmgr));

      test_gateable_clocks_off(&clkmgr, &pwrmgr, clock);

      // This should never be reached.
      LOG_ERROR("This is unreachable since a reset should have been triggered");
      return false;
    } else {
      return true;
    }
  } else {
    dif_rstmgr_reset_info_bitfield_t reset_info;
    reset_info = rstmgr_testutils_reason_get();
    LOG_ERROR("Unexpected reset_info 0x%x", reset_info);
  }
  return false;
}
