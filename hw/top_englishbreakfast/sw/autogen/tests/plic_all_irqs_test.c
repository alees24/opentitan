// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
// clang-format off
//
// ------------------- W A R N I N G: A U T O - G E N E R A T E D   C O D E !! -------------------//
// PLEASE DO NOT HAND-EDIT THIS FILE. IT HAS BEEN AUTO-GENERATED WITH THE FOLLOWING COMMAND:
// util/topgen.py -t hw/top_englishbreakfast/data/top_englishbreakfast.hjson
// -o hw/top_englishbreakfast
#include <limits.h>

// This test should avoid otp_ctrl interrupts in rom_ext, since the rom
// extension configures CSR accesses to OTP and AST to become illegal.
//
// This test is getting too big so we need to split it up. To do so,
// each peripheral is given an ID (according to their alphabetical order)
// and we define TEST_MIN_IRQ_PERIPHERAL and TEST_MAX_IRQ_PERIPHERAL to
// choose which ones are being tested.

#ifndef TEST_MIN_IRQ_PERIPHERAL
#define TEST_MIN_IRQ_PERIPHERAL 0
#endif

#ifndef TEST_MAX_IRQ_PERIPHERAL
#define TEST_MAX_IRQ_PERIPHERAL 8
#endif

#include "sw/device/lib/arch/boot_stage.h"
#include "sw/device/lib/base/csr.h"
#include "sw/device/lib/base/mmio.h"
#include "sw/device/lib/dif/autogen/dif_aon_timer_autogen.h"
#include "sw/device/lib/dif/autogen/dif_flash_ctrl_autogen.h"
#include "sw/device/lib/dif/autogen/dif_gpio_autogen.h"
#include "sw/device/lib/dif/autogen/dif_pwrmgr_autogen.h"
#include "sw/device/lib/dif/autogen/dif_rv_plic_autogen.h"
#include "sw/device/lib/dif/autogen/dif_spi_device_autogen.h"
#include "sw/device/lib/dif/autogen/dif_spi_host_autogen.h"
#include "sw/device/lib/dif/autogen/dif_uart_autogen.h"
#include "sw/device/lib/dif/autogen/dif_usbdev_autogen.h"
#include "sw/device/lib/runtime/ibex.h"
#include "sw/device/lib/runtime/irq.h"
#include "sw/device/lib/runtime/log.h"
#include "sw/device/lib/testing/rv_plic_testutils.h"
#include "sw/device/lib/testing/test_framework/check.h"
#include "sw/device/lib/testing/test_framework/ottf_main.h"
#include "sw/device/lib/testing/test_framework/status.h"

#include "hw/top_englishbreakfast/sw/autogen/top_englishbreakfast.h"

#if TEST_MIN_IRQ_PERIPHERAL <= 0 && 0 < TEST_MAX_IRQ_PERIPHERAL
static dif_aon_timer_t aon_timer_aon;
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 1 && 1 < TEST_MAX_IRQ_PERIPHERAL
static dif_flash_ctrl_t flash_ctrl;
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 2 && 2 < TEST_MAX_IRQ_PERIPHERAL
static dif_gpio_t gpio;
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 3 && 3 < TEST_MAX_IRQ_PERIPHERAL
static dif_pwrmgr_t pwrmgr_aon;
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 4 && 4 < TEST_MAX_IRQ_PERIPHERAL
static dif_spi_device_t spi_device;
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 5 && 5 < TEST_MAX_IRQ_PERIPHERAL
static dif_spi_host_t spi_host0;
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 6 && 6 < TEST_MAX_IRQ_PERIPHERAL
static dif_uart_t uart0;
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 6 && 6 < TEST_MAX_IRQ_PERIPHERAL
static dif_uart_t uart1;
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 7 && 7 < TEST_MAX_IRQ_PERIPHERAL
static dif_usbdev_t usbdev;
#endif

static dif_rv_plic_t plic;
static const top_englishbreakfast_plic_target_t kHart = kTopEnglishbreakfastPlicTargetIbex0;

/**
 * Flag indicating which peripheral is under test.
 *
 * Declared volatile because it is referenced in the main program flow as well
 * as the ISR.
 */
static volatile top_englishbreakfast_plic_peripheral_t peripheral_expected;

/**
 * Flags indicating the IRQ expected to have triggered and serviced within the
 * peripheral.
 *
 * Declared volatile because it is referenced in the main program flow as well
 * as the ISR.
 */

#if TEST_MIN_IRQ_PERIPHERAL <= 0 && 0 < TEST_MAX_IRQ_PERIPHERAL
static volatile dif_aon_timer_irq_t aon_timer_irq_expected;
static volatile dif_aon_timer_irq_t aon_timer_irq_serviced;
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 1 && 1 < TEST_MAX_IRQ_PERIPHERAL
static volatile dif_flash_ctrl_irq_t flash_ctrl_irq_expected;
static volatile dif_flash_ctrl_irq_t flash_ctrl_irq_serviced;
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 2 && 2 < TEST_MAX_IRQ_PERIPHERAL
static volatile dif_gpio_irq_t gpio_irq_expected;
static volatile dif_gpio_irq_t gpio_irq_serviced;
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 3 && 3 < TEST_MAX_IRQ_PERIPHERAL
static volatile dif_pwrmgr_irq_t pwrmgr_irq_expected;
static volatile dif_pwrmgr_irq_t pwrmgr_irq_serviced;
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 4 && 4 < TEST_MAX_IRQ_PERIPHERAL
static volatile dif_spi_device_irq_t spi_device_irq_expected;
static volatile dif_spi_device_irq_t spi_device_irq_serviced;
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 5 && 5 < TEST_MAX_IRQ_PERIPHERAL
static volatile dif_spi_host_irq_t spi_host_irq_expected;
static volatile dif_spi_host_irq_t spi_host_irq_serviced;
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 6 && 6 < TEST_MAX_IRQ_PERIPHERAL
static volatile dif_uart_irq_t uart_irq_expected;
static volatile dif_uart_irq_t uart_irq_serviced;
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 7 && 7 < TEST_MAX_IRQ_PERIPHERAL
static volatile dif_usbdev_irq_t usbdev_irq_expected;
static volatile dif_usbdev_irq_t usbdev_irq_serviced;
#endif

/**
 * Provides external IRQ handling for this test.
 *
 * This function overrides the default OTTF external ISR.
 *
 * For each IRQ, it performs the following:
 * 1. Claims the IRQ fired (finds PLIC IRQ index).
 * 2. Checks that the index belongs to the expected peripheral.
 * 3. Checks that the correct and the only IRQ from the expected peripheral
 *    triggered.
 * 4. Clears the IRQ at the peripheral.
 * 5. Completes the IRQ service at PLIC.
 */
void ottf_external_isr(uint32_t *exc_info) {
  dif_rv_plic_irq_id_t plic_irq_id;
  CHECK_DIF_OK(dif_rv_plic_irq_claim(&plic, kHart, &plic_irq_id));

  top_englishbreakfast_plic_peripheral_t peripheral = (top_englishbreakfast_plic_peripheral_t)
      top_englishbreakfast_plic_interrupt_for_peripheral[plic_irq_id];
  CHECK(peripheral == peripheral_expected,
        "Interrupt from incorrect peripheral: exp = %d, obs = %d",
        peripheral_expected, peripheral);

  switch (peripheral) {
#if TEST_MIN_IRQ_PERIPHERAL <= 0 && 0 < TEST_MAX_IRQ_PERIPHERAL
    case kTopEnglishbreakfastPlicPeripheralAonTimerAon: {
      dif_aon_timer_irq_t irq =
          (dif_aon_timer_irq_t)(plic_irq_id -
                                (dif_rv_plic_irq_id_t)
                                    kTopEnglishbreakfastPlicIrqIdAonTimerAonWkupTimerExpired);
      CHECK(irq == aon_timer_irq_expected,
            "Incorrect aon_timer_aon IRQ triggered: exp = %d, obs = %d",
            aon_timer_irq_expected, irq);
      aon_timer_irq_serviced = irq;

      dif_aon_timer_irq_state_snapshot_t snapshot;
      CHECK_DIF_OK(dif_aon_timer_irq_get_state(&aon_timer_aon, &snapshot));
      CHECK(snapshot == (dif_aon_timer_irq_state_snapshot_t)(1 << irq),
            "Only aon_timer_aon IRQ %d expected to fire. Actual interrupt "
            "status = %x",
            irq, snapshot);

      CHECK_DIF_OK(dif_aon_timer_irq_acknowledge(&aon_timer_aon, irq));
      break;
    }
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 1 && 1 < TEST_MAX_IRQ_PERIPHERAL
    case kTopEnglishbreakfastPlicPeripheralFlashCtrl: {
      dif_flash_ctrl_irq_t irq =
          (dif_flash_ctrl_irq_t)(plic_irq_id -
                                 (dif_rv_plic_irq_id_t)
                                     kTopEnglishbreakfastPlicIrqIdFlashCtrlProgEmpty);
      CHECK(irq == flash_ctrl_irq_expected,
            "Incorrect flash_ctrl IRQ triggered: exp = %d, obs = %d",
            flash_ctrl_irq_expected, irq);
      flash_ctrl_irq_serviced = irq;

      dif_flash_ctrl_irq_state_snapshot_t snapshot;
      CHECK_DIF_OK(dif_flash_ctrl_irq_get_state(&flash_ctrl, &snapshot));
      CHECK(snapshot == (dif_flash_ctrl_irq_state_snapshot_t)((1 << irq) | 0x3),
            "Expected flash_ctrl interrupt status %x. Actual interrupt "
            "status = %x",
            (1 << irq) | 0x3, snapshot);

      if (0xf & (1 << irq)) {
        // We do not acknowledge status type interrupt at the IP side, but we
        // need to clear the test force register.
        CHECK_DIF_OK(dif_flash_ctrl_irq_force(&flash_ctrl, irq, false));
        // In case this status interrupt is asserted by default, we also
        // disable it at this point so that it does not interfere with the
        // rest of the test.
        if ((0x3 & (1 << irq))) {
          CHECK_DIF_OK(dif_flash_ctrl_irq_set_enabled(&flash_ctrl, irq, false));
        }
      } else {
        // We acknowledge event type interrupt.
        CHECK_DIF_OK(dif_flash_ctrl_irq_acknowledge(&flash_ctrl, irq));
      }
      break;
    }
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 2 && 2 < TEST_MAX_IRQ_PERIPHERAL
    case kTopEnglishbreakfastPlicPeripheralGpio: {
      dif_gpio_irq_t irq =
          (dif_gpio_irq_t)(plic_irq_id -
                           (dif_rv_plic_irq_id_t)
                               kTopEnglishbreakfastPlicIrqIdGpioGpio0);
      CHECK(irq == gpio_irq_expected,
            "Incorrect gpio IRQ triggered: exp = %d, obs = %d",
            gpio_irq_expected, irq);
      gpio_irq_serviced = irq;

      dif_gpio_irq_state_snapshot_t snapshot;
      CHECK_DIF_OK(dif_gpio_irq_get_state(&gpio, &snapshot));
      CHECK(snapshot == (dif_gpio_irq_state_snapshot_t)(1 << irq),
            "Only gpio IRQ %d expected to fire. Actual interrupt "
            "status = %x",
            irq, snapshot);

      CHECK_DIF_OK(dif_gpio_irq_acknowledge(&gpio, irq));
      break;
    }
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 3 && 3 < TEST_MAX_IRQ_PERIPHERAL
    case kTopEnglishbreakfastPlicPeripheralPwrmgrAon: {
      dif_pwrmgr_irq_t irq =
          (dif_pwrmgr_irq_t)(plic_irq_id -
                             (dif_rv_plic_irq_id_t)
                                 kTopEnglishbreakfastPlicIrqIdPwrmgrAonWakeup);
      CHECK(irq == pwrmgr_irq_expected,
            "Incorrect pwrmgr_aon IRQ triggered: exp = %d, obs = %d",
            pwrmgr_irq_expected, irq);
      pwrmgr_irq_serviced = irq;

      dif_pwrmgr_irq_state_snapshot_t snapshot;
      CHECK_DIF_OK(dif_pwrmgr_irq_get_state(&pwrmgr_aon, &snapshot));
      CHECK(snapshot == (dif_pwrmgr_irq_state_snapshot_t)(1 << irq),
            "Only pwrmgr_aon IRQ %d expected to fire. Actual interrupt "
            "status = %x",
            irq, snapshot);

      CHECK_DIF_OK(dif_pwrmgr_irq_acknowledge(&pwrmgr_aon, irq));
      break;
    }
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 4 && 4 < TEST_MAX_IRQ_PERIPHERAL
    case kTopEnglishbreakfastPlicPeripheralSpiDevice: {
      dif_spi_device_irq_t irq =
          (dif_spi_device_irq_t)(plic_irq_id -
                                 (dif_rv_plic_irq_id_t)
                                     kTopEnglishbreakfastPlicIrqIdSpiDeviceUploadCmdfifoNotEmpty);
      CHECK(irq == spi_device_irq_expected,
            "Incorrect spi_device IRQ triggered: exp = %d, obs = %d",
            spi_device_irq_expected, irq);
      spi_device_irq_serviced = irq;

      dif_spi_device_irq_state_snapshot_t snapshot;
      CHECK_DIF_OK(dif_spi_device_irq_get_state(&spi_device, &snapshot));
      CHECK(snapshot == (dif_spi_device_irq_state_snapshot_t)(1 << irq),
            "Only spi_device IRQ %d expected to fire. Actual interrupt "
            "status = %x",
            irq, snapshot);

      if (0x20 & (1 << irq)) {
        // We do not acknowledge status type interrupt at the IP side, but we
        // need to clear the test force register.
        CHECK_DIF_OK(dif_spi_device_irq_force(&spi_device, irq, false));
        // In case this status interrupt is asserted by default, we also
        // disable it at this point so that it does not interfere with the
        // rest of the test.
        if ((0x0 & (1 << irq))) {
          CHECK_DIF_OK(dif_spi_device_irq_set_enabled(&spi_device, irq, false));
        }
      } else {
        // We acknowledge event type interrupt.
        CHECK_DIF_OK(dif_spi_device_irq_acknowledge(&spi_device, irq));
      }
      break;
    }
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 5 && 5 < TEST_MAX_IRQ_PERIPHERAL
    case kTopEnglishbreakfastPlicPeripheralSpiHost0: {
      dif_spi_host_irq_t irq =
          (dif_spi_host_irq_t)(plic_irq_id -
                               (dif_rv_plic_irq_id_t)
                                   kTopEnglishbreakfastPlicIrqIdSpiHost0Error);
      CHECK(irq == spi_host_irq_expected,
            "Incorrect spi_host0 IRQ triggered: exp = %d, obs = %d",
            spi_host_irq_expected, irq);
      spi_host_irq_serviced = irq;

      dif_spi_host_irq_state_snapshot_t snapshot;
      CHECK_DIF_OK(dif_spi_host_irq_get_state(&spi_host0, &snapshot));
      CHECK(snapshot == (dif_spi_host_irq_state_snapshot_t)(1 << irq),
            "Only spi_host0 IRQ %d expected to fire. Actual interrupt "
            "status = %x",
            irq, snapshot);

      if (0x2 & (1 << irq)) {
        // We do not acknowledge status type interrupt at the IP side, but we
        // need to clear the test force register.
        CHECK_DIF_OK(dif_spi_host_irq_force(&spi_host0, irq, false));
        // In case this status interrupt is asserted by default, we also
        // disable it at this point so that it does not interfere with the
        // rest of the test.
        if ((0x0 & (1 << irq))) {
          CHECK_DIF_OK(dif_spi_host_irq_set_enabled(&spi_host0, irq, false));
        }
      } else {
        // We acknowledge event type interrupt.
        CHECK_DIF_OK(dif_spi_host_irq_acknowledge(&spi_host0, irq));
      }
      break;
    }
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 6 && 6 < TEST_MAX_IRQ_PERIPHERAL
    case kTopEnglishbreakfastPlicPeripheralUart0: {
      dif_uart_irq_t irq =
          (dif_uart_irq_t)(plic_irq_id -
                           (dif_rv_plic_irq_id_t)
                               kTopEnglishbreakfastPlicIrqIdUart0TxWatermark);
      CHECK(irq == uart_irq_expected,
            "Incorrect uart0 IRQ triggered: exp = %d, obs = %d",
            uart_irq_expected, irq);
      uart_irq_serviced = irq;

      dif_uart_irq_state_snapshot_t snapshot;
      CHECK_DIF_OK(dif_uart_irq_get_state(&uart0, &snapshot));
      CHECK(snapshot == (dif_uart_irq_state_snapshot_t)((1 << irq) | 0x101),
            "Expected uart0 interrupt status %x. Actual interrupt "
            "status = %x",
            (1 << irq) | 0x101, snapshot);

      if (0x103 & (1 << irq)) {
        // We do not acknowledge status type interrupt at the IP side, but we
        // need to clear the test force register.
        CHECK_DIF_OK(dif_uart_irq_force(&uart0, irq, false));
        // In case this status interrupt is asserted by default, we also
        // disable it at this point so that it does not interfere with the
        // rest of the test.
        if ((0x101 & (1 << irq))) {
          CHECK_DIF_OK(dif_uart_irq_set_enabled(&uart0, irq, false));
        }
      } else {
        // We acknowledge event type interrupt.
        CHECK_DIF_OK(dif_uart_irq_acknowledge(&uart0, irq));
      }
      break;
    }
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 6 && 6 < TEST_MAX_IRQ_PERIPHERAL
    case kTopEnglishbreakfastPlicPeripheralUart1: {
      dif_uart_irq_t irq =
          (dif_uart_irq_t)(plic_irq_id -
                           (dif_rv_plic_irq_id_t)
                               kTopEnglishbreakfastPlicIrqIdUart1TxWatermark);
      CHECK(irq == uart_irq_expected,
            "Incorrect uart1 IRQ triggered: exp = %d, obs = %d",
            uart_irq_expected, irq);
      uart_irq_serviced = irq;

      dif_uart_irq_state_snapshot_t snapshot;
      CHECK_DIF_OK(dif_uart_irq_get_state(&uart1, &snapshot));
      CHECK(snapshot == (dif_uart_irq_state_snapshot_t)((1 << irq) | 0x101),
            "Expected uart1 interrupt status %x. Actual interrupt "
            "status = %x",
            (1 << irq) | 0x101, snapshot);

      if (0x103 & (1 << irq)) {
        // We do not acknowledge status type interrupt at the IP side, but we
        // need to clear the test force register.
        CHECK_DIF_OK(dif_uart_irq_force(&uart1, irq, false));
        // In case this status interrupt is asserted by default, we also
        // disable it at this point so that it does not interfere with the
        // rest of the test.
        if ((0x101 & (1 << irq))) {
          CHECK_DIF_OK(dif_uart_irq_set_enabled(&uart1, irq, false));
        }
      } else {
        // We acknowledge event type interrupt.
        CHECK_DIF_OK(dif_uart_irq_acknowledge(&uart1, irq));
      }
      break;
    }
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 7 && 7 < TEST_MAX_IRQ_PERIPHERAL
    case kTopEnglishbreakfastPlicPeripheralUsbdev: {
      dif_usbdev_irq_t irq =
          (dif_usbdev_irq_t)(plic_irq_id -
                             (dif_rv_plic_irq_id_t)
                                 kTopEnglishbreakfastPlicIrqIdUsbdevPktReceived);
      CHECK(irq == usbdev_irq_expected,
            "Incorrect usbdev IRQ triggered: exp = %d, obs = %d",
            usbdev_irq_expected, irq);
      usbdev_irq_serviced = irq;

      dif_usbdev_irq_state_snapshot_t snapshot;
      CHECK_DIF_OK(dif_usbdev_irq_get_state(&usbdev, &snapshot));
      CHECK(snapshot == (dif_usbdev_irq_state_snapshot_t)(1 << irq),
            "Only usbdev IRQ %d expected to fire. Actual interrupt "
            "status = %x",
            irq, snapshot);

      if (0x20183 & (1 << irq)) {
        // We do not acknowledge status type interrupt at the IP side, but we
        // need to clear the test force register.
        CHECK_DIF_OK(dif_usbdev_irq_force(&usbdev, irq, false));
        // In case this status interrupt is asserted by default, we also
        // disable it at this point so that it does not interfere with the
        // rest of the test.
        if ((0x0 & (1 << irq))) {
          CHECK_DIF_OK(dif_usbdev_irq_set_enabled(&usbdev, irq, false));
        }
      } else {
        // We acknowledge event type interrupt.
        CHECK_DIF_OK(dif_usbdev_irq_acknowledge(&usbdev, irq));
      }
      break;
    }
#endif

    default:
      LOG_FATAL("ISR is not implemented!");
      test_status_set(kTestStatusFailed);
  }
  // Complete the IRQ at PLIC.
  CHECK_DIF_OK(dif_rv_plic_irq_complete(&plic, kHart, plic_irq_id));
}

/**
 * Initializes the handles to all peripherals.
 */
static void peripherals_init(void) {
  mmio_region_t base_addr;

#if TEST_MIN_IRQ_PERIPHERAL <= 0 && 0 < TEST_MAX_IRQ_PERIPHERAL
  base_addr = mmio_region_from_addr(TOP_ENGLISHBREAKFAST_AON_TIMER_AON_BASE_ADDR);
  CHECK_DIF_OK(dif_aon_timer_init(base_addr, &aon_timer_aon));
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 1 && 1 < TEST_MAX_IRQ_PERIPHERAL
  base_addr = mmio_region_from_addr(TOP_ENGLISHBREAKFAST_FLASH_CTRL_CORE_BASE_ADDR);
  CHECK_DIF_OK(dif_flash_ctrl_init(base_addr, &flash_ctrl));
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 2 && 2 < TEST_MAX_IRQ_PERIPHERAL
  base_addr = mmio_region_from_addr(TOP_ENGLISHBREAKFAST_GPIO_BASE_ADDR);
  CHECK_DIF_OK(dif_gpio_init(base_addr, &gpio));
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 3 && 3 < TEST_MAX_IRQ_PERIPHERAL
  base_addr = mmio_region_from_addr(TOP_ENGLISHBREAKFAST_PWRMGR_AON_BASE_ADDR);
  CHECK_DIF_OK(dif_pwrmgr_init(base_addr, &pwrmgr_aon));
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 4 && 4 < TEST_MAX_IRQ_PERIPHERAL
  base_addr = mmio_region_from_addr(TOP_ENGLISHBREAKFAST_SPI_DEVICE_BASE_ADDR);
  CHECK_DIF_OK(dif_spi_device_init(base_addr, &spi_device));
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 5 && 5 < TEST_MAX_IRQ_PERIPHERAL
  base_addr = mmio_region_from_addr(TOP_ENGLISHBREAKFAST_SPI_HOST0_BASE_ADDR);
  CHECK_DIF_OK(dif_spi_host_init(base_addr, &spi_host0));
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 6 && 6 < TEST_MAX_IRQ_PERIPHERAL
  base_addr = mmio_region_from_addr(TOP_ENGLISHBREAKFAST_UART0_BASE_ADDR);
  CHECK_DIF_OK(dif_uart_init(base_addr, &uart0));
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 6 && 6 < TEST_MAX_IRQ_PERIPHERAL
  base_addr = mmio_region_from_addr(TOP_ENGLISHBREAKFAST_UART1_BASE_ADDR);
  CHECK_DIF_OK(dif_uart_init(base_addr, &uart1));
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 7 && 7 < TEST_MAX_IRQ_PERIPHERAL
  base_addr = mmio_region_from_addr(TOP_ENGLISHBREAKFAST_USBDEV_BASE_ADDR);
  CHECK_DIF_OK(dif_usbdev_init(base_addr, &usbdev));
#endif

  base_addr = mmio_region_from_addr(TOP_ENGLISHBREAKFAST_RV_PLIC_BASE_ADDR);
  CHECK_DIF_OK(dif_rv_plic_init(base_addr, &plic));
}

/**
 * Clears pending IRQs in all peripherals.
 */
static void peripheral_irqs_clear(void) {
#if TEST_MIN_IRQ_PERIPHERAL <= 0 && 0 < TEST_MAX_IRQ_PERIPHERAL
  CHECK_DIF_OK(dif_aon_timer_irq_acknowledge_all(&aon_timer_aon));
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 1 && 1 < TEST_MAX_IRQ_PERIPHERAL
  CHECK_DIF_OK(dif_flash_ctrl_irq_acknowledge_all(&flash_ctrl));
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 2 && 2 < TEST_MAX_IRQ_PERIPHERAL
  CHECK_DIF_OK(dif_gpio_irq_acknowledge_all(&gpio));
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 3 && 3 < TEST_MAX_IRQ_PERIPHERAL
  CHECK_DIF_OK(dif_pwrmgr_irq_acknowledge_all(&pwrmgr_aon));
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 4 && 4 < TEST_MAX_IRQ_PERIPHERAL
  CHECK_DIF_OK(dif_spi_device_irq_acknowledge_all(&spi_device));
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 5 && 5 < TEST_MAX_IRQ_PERIPHERAL
  CHECK_DIF_OK(dif_spi_host_irq_acknowledge_all(&spi_host0));
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 6 && 6 < TEST_MAX_IRQ_PERIPHERAL
  CHECK_DIF_OK(dif_uart_irq_acknowledge_all(&uart0));
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 6 && 6 < TEST_MAX_IRQ_PERIPHERAL
  CHECK_DIF_OK(dif_uart_irq_acknowledge_all(&uart1));
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 7 && 7 < TEST_MAX_IRQ_PERIPHERAL
  CHECK_DIF_OK(dif_usbdev_irq_acknowledge_all(&usbdev));
#endif
}

/**
 * Enables all IRQs in all peripherals.
 */
static void peripheral_irqs_enable(void) {
#if TEST_MIN_IRQ_PERIPHERAL <= 1 && 1 < TEST_MAX_IRQ_PERIPHERAL
  // Note: this peripheral contains status interrupts that are asserted by
  // default. Therefore, not all interrupts are enabled here, since that
  // would interfere with this test. Instead, these interrupts are enabled on
  // demand once they are being tested.
  dif_flash_ctrl_irq_state_snapshot_t flash_ctrl_irqs =
      (dif_flash_ctrl_irq_state_snapshot_t)0xfffffffc;
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 2 && 2 < TEST_MAX_IRQ_PERIPHERAL
  dif_gpio_irq_state_snapshot_t gpio_irqs =
      (dif_gpio_irq_state_snapshot_t)0xffffffff;
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 3 && 3 < TEST_MAX_IRQ_PERIPHERAL
  dif_pwrmgr_irq_state_snapshot_t pwrmgr_irqs =
      (dif_pwrmgr_irq_state_snapshot_t)0xffffffff;
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 4 && 4 < TEST_MAX_IRQ_PERIPHERAL
  dif_spi_device_irq_state_snapshot_t spi_device_irqs =
      (dif_spi_device_irq_state_snapshot_t)0xffffffff;
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 5 && 5 < TEST_MAX_IRQ_PERIPHERAL
  dif_spi_host_irq_state_snapshot_t spi_host_irqs =
      (dif_spi_host_irq_state_snapshot_t)0xffffffff;
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 6 && 6 < TEST_MAX_IRQ_PERIPHERAL
  // Note: this peripheral contains status interrupts that are asserted by
  // default. Therefore, not all interrupts are enabled here, since that
  // would interfere with this test. Instead, these interrupts are enabled on
  // demand once they are being tested.
  dif_uart_irq_state_snapshot_t uart_irqs =
      (dif_uart_irq_state_snapshot_t)0xfffffefe;
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 7 && 7 < TEST_MAX_IRQ_PERIPHERAL
  dif_usbdev_irq_state_snapshot_t usbdev_irqs =
      (dif_usbdev_irq_state_snapshot_t)0xffffffff;
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 1 && 1 < TEST_MAX_IRQ_PERIPHERAL
  CHECK_DIF_OK(dif_flash_ctrl_irq_restore_all(&flash_ctrl, &flash_ctrl_irqs));
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 2 && 2 < TEST_MAX_IRQ_PERIPHERAL
  CHECK_DIF_OK(dif_gpio_irq_restore_all(&gpio, &gpio_irqs));
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 3 && 3 < TEST_MAX_IRQ_PERIPHERAL
  CHECK_DIF_OK(dif_pwrmgr_irq_restore_all(&pwrmgr_aon, &pwrmgr_irqs));
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 4 && 4 < TEST_MAX_IRQ_PERIPHERAL
  CHECK_DIF_OK(dif_spi_device_irq_restore_all(&spi_device, &spi_device_irqs));
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 5 && 5 < TEST_MAX_IRQ_PERIPHERAL
  CHECK_DIF_OK(dif_spi_host_irq_restore_all(&spi_host0, &spi_host_irqs));
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 6 && 6 < TEST_MAX_IRQ_PERIPHERAL
  // lowrisc/opentitan#8656: Skip UART0 in non-DV setups due to interference
  // from the logging facility.
  if (kDeviceType == kDeviceSimDV) {
    CHECK_DIF_OK(dif_uart_irq_restore_all(&uart0, &uart_irqs));
  }
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 6 && 6 < TEST_MAX_IRQ_PERIPHERAL
  CHECK_DIF_OK(dif_uart_irq_restore_all(&uart1, &uart_irqs));
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 7 && 7 < TEST_MAX_IRQ_PERIPHERAL
  CHECK_DIF_OK(dif_usbdev_irq_restore_all(&usbdev, &usbdev_irqs));
#endif
}

/**
 * Triggers all IRQs in all peripherals one by one.
 *
 * Walks through all instances of all peripherals and triggers an interrupt one
 * by one, by forcing with the `intr_test` CSR. On trigger, the CPU instantly
 * jumps into the ISR. The main flow of execution thus proceeds to check that
 * the correct IRQ was serviced immediately. The ISR, in turn checks if the
 * expected IRQ from the expected peripheral triggered.
 */
static void peripheral_irqs_trigger(void) {
  unsigned int status_default_mask;
  // Depending on the build configuration, this variable may show up as unused
  // in the clang linter. This statement waives that error.
  (void)status_default_mask;

#if TEST_MIN_IRQ_PERIPHERAL <= 0 && 0 < TEST_MAX_IRQ_PERIPHERAL
  // lowrisc/opentitan#8656: Skip UART0 in non-DV setups due to interference
  // from the logging facility.
  // aon_timer may generate a NMI instead of a PLIC IRQ depending on the ROM.
  // Since there are other tests covering this already, we just skip this for
  // non-DV setups.
  if (kDeviceType == kDeviceSimDV) {
    peripheral_expected = kTopEnglishbreakfastPlicPeripheralAonTimerAon;
    for (dif_aon_timer_irq_t irq = kDifAonTimerIrqWkupTimerExpired; irq <= kDifAonTimerIrqWdogTimerBark;
         ++irq) {
      aon_timer_irq_expected = irq;
      LOG_INFO("Triggering aon_timer_aon IRQ %d.", irq);
      CHECK_DIF_OK(dif_aon_timer_irq_force(&aon_timer_aon, irq, true));

      // This avoids a race where *irq_serviced is read before
      // entering the ISR.
      IBEX_SPIN_FOR(aon_timer_irq_serviced == irq, 1);
      LOG_INFO("IRQ %d from aon_timer_aon is serviced.", irq);
    }
  }
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 1 && 1 < TEST_MAX_IRQ_PERIPHERAL
  peripheral_expected = kTopEnglishbreakfastPlicPeripheralFlashCtrl;
  status_default_mask = 0x3;
  for (dif_flash_ctrl_irq_t irq = kDifFlashCtrlIrqProgEmpty; irq <= kDifFlashCtrlIrqCorrErr;
       ++irq) {
    flash_ctrl_irq_expected = irq;
    LOG_INFO("Triggering flash_ctrl IRQ %d.", irq);
    CHECK_DIF_OK(dif_flash_ctrl_irq_force(&flash_ctrl, irq, true));

    // In this case, the interrupt has not been enabled yet because that would
    // interfere with testing other interrupts. We enable it here and let the
    // interrupt handler disable it again.
    if ((status_default_mask & 0x1)) {
      CHECK_DIF_OK(dif_flash_ctrl_irq_set_enabled(&flash_ctrl, irq, true));
    }
    status_default_mask >>= 1;

    // This avoids a race where *irq_serviced is read before
    // entering the ISR.
    IBEX_SPIN_FOR(flash_ctrl_irq_serviced == irq, 1);
    LOG_INFO("IRQ %d from flash_ctrl is serviced.", irq);
  }
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 2 && 2 < TEST_MAX_IRQ_PERIPHERAL
  peripheral_expected = kTopEnglishbreakfastPlicPeripheralGpio;
  for (dif_gpio_irq_t irq = kDifGpioIrqGpio0; irq <= kDifGpioIrqGpio31;
       ++irq) {
    gpio_irq_expected = irq;
    LOG_INFO("Triggering gpio IRQ %d.", irq);
    CHECK_DIF_OK(dif_gpio_irq_force(&gpio, irq, true));

    // This avoids a race where *irq_serviced is read before
    // entering the ISR.
    IBEX_SPIN_FOR(gpio_irq_serviced == irq, 1);
    LOG_INFO("IRQ %d from gpio is serviced.", irq);
  }
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 3 && 3 < TEST_MAX_IRQ_PERIPHERAL
  peripheral_expected = kTopEnglishbreakfastPlicPeripheralPwrmgrAon;
  for (dif_pwrmgr_irq_t irq = kDifPwrmgrIrqWakeup; irq <= kDifPwrmgrIrqWakeup;
       ++irq) {
    pwrmgr_irq_expected = irq;
    LOG_INFO("Triggering pwrmgr_aon IRQ %d.", irq);
    CHECK_DIF_OK(dif_pwrmgr_irq_force(&pwrmgr_aon, irq, true));

    // This avoids a race where *irq_serviced is read before
    // entering the ISR.
    IBEX_SPIN_FOR(pwrmgr_irq_serviced == irq, 1);
    LOG_INFO("IRQ %d from pwrmgr_aon is serviced.", irq);
  }
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 4 && 4 < TEST_MAX_IRQ_PERIPHERAL
  peripheral_expected = kTopEnglishbreakfastPlicPeripheralSpiDevice;
  status_default_mask = 0x0;
  for (dif_spi_device_irq_t irq = kDifSpiDeviceIrqUploadCmdfifoNotEmpty; irq <= kDifSpiDeviceIrqTpmRdfifoDrop;
       ++irq) {
    spi_device_irq_expected = irq;
    LOG_INFO("Triggering spi_device IRQ %d.", irq);
    CHECK_DIF_OK(dif_spi_device_irq_force(&spi_device, irq, true));

    // In this case, the interrupt has not been enabled yet because that would
    // interfere with testing other interrupts. We enable it here and let the
    // interrupt handler disable it again.
    if ((status_default_mask & 0x1)) {
      CHECK_DIF_OK(dif_spi_device_irq_set_enabled(&spi_device, irq, true));
    }
    status_default_mask >>= 1;

    // This avoids a race where *irq_serviced is read before
    // entering the ISR.
    IBEX_SPIN_FOR(spi_device_irq_serviced == irq, 1);
    LOG_INFO("IRQ %d from spi_device is serviced.", irq);
  }
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 5 && 5 < TEST_MAX_IRQ_PERIPHERAL
  peripheral_expected = kTopEnglishbreakfastPlicPeripheralSpiHost0;
  status_default_mask = 0x0;
  for (dif_spi_host_irq_t irq = kDifSpiHostIrqError; irq <= kDifSpiHostIrqSpiEvent;
       ++irq) {
    spi_host_irq_expected = irq;
    LOG_INFO("Triggering spi_host0 IRQ %d.", irq);
    CHECK_DIF_OK(dif_spi_host_irq_force(&spi_host0, irq, true));

    // In this case, the interrupt has not been enabled yet because that would
    // interfere with testing other interrupts. We enable it here and let the
    // interrupt handler disable it again.
    if ((status_default_mask & 0x1)) {
      CHECK_DIF_OK(dif_spi_host_irq_set_enabled(&spi_host0, irq, true));
    }
    status_default_mask >>= 1;

    // This avoids a race where *irq_serviced is read before
    // entering the ISR.
    IBEX_SPIN_FOR(spi_host_irq_serviced == irq, 1);
    LOG_INFO("IRQ %d from spi_host0 is serviced.", irq);
  }
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 6 && 6 < TEST_MAX_IRQ_PERIPHERAL
  // lowrisc/opentitan#8656: Skip UART0 in non-DV setups due to interference
  // from the logging facility.
  // aon_timer may generate a NMI instead of a PLIC IRQ depending on the ROM.
  // Since there are other tests covering this already, we just skip this for
  // non-DV setups.
  if (kDeviceType == kDeviceSimDV) {
    peripheral_expected = kTopEnglishbreakfastPlicPeripheralUart0;
    status_default_mask = 0x101;
    for (dif_uart_irq_t irq = kDifUartIrqTxWatermark; irq <= kDifUartIrqTxEmpty;
         ++irq) {
      uart_irq_expected = irq;
      LOG_INFO("Triggering uart0 IRQ %d.", irq);
      CHECK_DIF_OK(dif_uart_irq_force(&uart0, irq, true));

      // In this case, the interrupt has not been enabled yet because that would
      // interfere with testing other interrupts. We enable it here and let the
      // interrupt handler disable it again.
      if ((status_default_mask & 0x1)) {
        CHECK_DIF_OK(dif_uart_irq_set_enabled(&uart0, irq, true));
      }
      status_default_mask >>= 1;

      // This avoids a race where *irq_serviced is read before
      // entering the ISR.
      IBEX_SPIN_FOR(uart_irq_serviced == irq, 1);
      LOG_INFO("IRQ %d from uart0 is serviced.", irq);
    }
  }
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 6 && 6 < TEST_MAX_IRQ_PERIPHERAL
  peripheral_expected = kTopEnglishbreakfastPlicPeripheralUart1;
  status_default_mask = 0x101;
  for (dif_uart_irq_t irq = kDifUartIrqTxWatermark; irq <= kDifUartIrqTxEmpty;
       ++irq) {
    uart_irq_expected = irq;
    LOG_INFO("Triggering uart1 IRQ %d.", irq);
    CHECK_DIF_OK(dif_uart_irq_force(&uart1, irq, true));

    // In this case, the interrupt has not been enabled yet because that would
    // interfere with testing other interrupts. We enable it here and let the
    // interrupt handler disable it again.
    if ((status_default_mask & 0x1)) {
      CHECK_DIF_OK(dif_uart_irq_set_enabled(&uart1, irq, true));
    }
    status_default_mask >>= 1;

    // This avoids a race where *irq_serviced is read before
    // entering the ISR.
    IBEX_SPIN_FOR(uart_irq_serviced == irq, 1);
    LOG_INFO("IRQ %d from uart1 is serviced.", irq);
  }
#endif

#if TEST_MIN_IRQ_PERIPHERAL <= 7 && 7 < TEST_MAX_IRQ_PERIPHERAL
  peripheral_expected = kTopEnglishbreakfastPlicPeripheralUsbdev;
  status_default_mask = 0x0;
  for (dif_usbdev_irq_t irq = kDifUsbdevIrqPktReceived; irq <= kDifUsbdevIrqAvSetupEmpty;
       ++irq) {
    usbdev_irq_expected = irq;
    LOG_INFO("Triggering usbdev IRQ %d.", irq);
    CHECK_DIF_OK(dif_usbdev_irq_force(&usbdev, irq, true));

    // In this case, the interrupt has not been enabled yet because that would
    // interfere with testing other interrupts. We enable it here and let the
    // interrupt handler disable it again.
    if ((status_default_mask & 0x1)) {
      CHECK_DIF_OK(dif_usbdev_irq_set_enabled(&usbdev, irq, true));
    }
    status_default_mask >>= 1;

    // This avoids a race where *irq_serviced is read before
    // entering the ISR.
    IBEX_SPIN_FOR(usbdev_irq_serviced == irq, 1);
    LOG_INFO("IRQ %d from usbdev is serviced.", irq);
  }
#endif
}

/**
 * Checks that the target ID corresponds to the ID of the hart on which
 * this test is executed on. This check is meant to be used in a
 * single-hart system only.
 */
static void check_hart_id(uint32_t exp_hart_id) {
  uint32_t act_hart_id;
  CSR_READ(CSR_REG_MHARTID, &act_hart_id);
  CHECK(act_hart_id == exp_hart_id, "Processor has unexpected HART ID.");
}

OTTF_DEFINE_TEST_CONFIG();

bool test_main(void) {
  irq_global_ctrl(true);
  irq_external_ctrl(true);
  peripherals_init();
  check_hart_id((uint32_t)kHart);
  rv_plic_testutils_irq_range_enable(
      &plic, kHart, kTopEnglishbreakfastPlicIrqIdNone + 1, kTopEnglishbreakfastPlicIrqIdLast);
  peripheral_irqs_clear();
  peripheral_irqs_enable();
  peripheral_irqs_trigger();
  return true;
}

// clang-format on
