// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// USB suspend/resume test

#include <inttypes.h>
#include <stdint.h>
#include <string.h>

#include "sw/device/lib/runtime/ibex.h"
#include "sw/device/lib/runtime/irq.h"
#include "sw/device/lib/runtime/log.h"
#include "sw/device/lib/runtime/print.h"
#include "sw/device/lib/testing/pinmux_testutils.h"
#include "sw/device/lib/testing/test_framework/check.h"
#include "sw/device/lib/testing/test_framework/ottf_main.h"
#include "sw/device/lib/testing/usb_testutils.h"
#include "sw/device/lib/testing/usb_testutils_controlep.h"

/**
 * Test phases; named according to the event that we are expecting to occur.
 */
typedef enum {
  /**
   * First test phase just tests regular Suspend/Resume signaling; after we've
   * resumed, we expect a Bus Reset from the DPI/Host.
   */
  kSuspendPhaseSuspend = 0u,
  /**
   * This test phase instructs the DPI model to put the DUT into Suspend long
   * enough that this software will attempt to put the device into its Normal
   * Sleep state and exercise the AON/Wakeup module, stopping the clocks but not
   * powering down.
   */
  kSuspendPhaseSleepResume,
  /*
   * The AON/Wakeup module will cause us to awaken in response to a bus reset.
   */
  kSuspendPhaseSleepReset,
  /**
   * As above, but this time we're expecting a VBUS/SENSE loss.
   */
  kSuspendPhaseSleepDisconnect,
  /**
   * Mirrors Resume detection for normal sleep, but this time we enter Deep
   * Sleep and the power is removed too.
   */
  kSuspendPhaseDeepResume,
  /**
   * Mirrors Bus Reset detection for normal sleep, but this time we enter Deep
   * Sleep and the power is removed too.
   */
  kSuspendPhaseDeepReset,
  /**
   * As above, but this time we're expecting a VBUS/SENSE loss.
   */
  kSuspendPhaseDeepDisconnect,
  /**
   * Final phase; shut down.
   */
  kSuspendPhaseShutdown,
} usbdev_suspend_phase_t;


#include "hw/top_earlgrey/sw/autogen/top_earlgrey.h"  // Generated.
//#include "sw/device/lib/testing/autogen/isr_testutils.h"

// Set up the recording of function points for this module
#define USBUTILS_FUNCPT_FILE USBUTILS_FUNCPT_FILE_USBDEV_SUSP_TEST

// Are we expecting a full frame interval?
// TODO: this must match the setting in the DPI model, but we do not have the
// ability to share header files.
// TODO: also note that this shall always be 1 for physical hosts.
#define FULL_FRAME 0

#if FULL_FRAME
#define USBDPI_FRAME_INTERVAL 1000  // 1ms frame time
#else
#define USBDPI_FRAME_INTERVAL 500  // 0.5ms
#endif

/**
 * Timeout constants in microseconds;
 */
#define TIMEOUT_RESUME_MISSED 4500u
// #define TIMEOUT_

#define TIMEOUT_WAKEUP_RESUME 3000u
#define TIMEOUT_FINISH_MISSED 2000u

// How long should we wait on FPGA for configuration to occur?
#define TIMEOUT_FPGA_CONFIG 10U * 1000U * 1000U

/**
 * Durations that are specified in terms of bus frames, however long those
 * simulated bus frames happen to be (ie. these intervals are determined by
 * the DPI host behavior rather than the USB protocol specification)
 */

#define FRAMES_CONFIG_DELAY 2u

#define FRAMES_SUSPEND_MISSED 2u

#define FRAMES_INITIATE_RESUME 2u

/**
 * Number of frame delays to wait after device signals Suspend, before we
 */
#define FRAMES_WAIT_ENTER_SLEEP 1u

#define USB_EVENT_REPORT(s) USBUTILS_TRACE(__LINE__, (s))

/**
 * Test states
 */
typedef enum {
  /**
   * Power On Reset (start of first phase of test sequence).
   */
  kSuspendStatePowerOnReset = 0u,
  /**
   * Bus Reset from DPI/Host has occurred.
   */
  kSuspendStateBusReset,
  /**
   * Waiting for the DPI/Host to suspend the device, for normal Suspend/Resume
   * behavior, not involving AON/Wakeup functionality.
   */
  kSuspendStateWaitSuspend,
  kSuspendStateWaitResume,
  kSuspendStateWaitBusReset,
  /**
   * Waiting for the DPI/Host to suspend the device, expecting a longer suspend
   * period, during which we put the device into Normal/Deep Sleep using the
   * AON/Wakeup functionality.
   */
  kSuspendStateWaitLongSuspend,
  /**
   * Waiting whilst Suspended, until we decide to enter Normal/Deep Sleep.
   * The DPI model is not expected to resume communication during this time.
   */
  kSuspendStateWaitSuspendTimeout,
  /**
   * We have instructed the AON/Wakeup module to wake over control of the USB
   * signals. It does not do so immediately because it lives in a slower clock
   * domain, but the delay should be very short.
   */
  kSuspendStateActivatedAON,
  /**
   * We are expecting to fall into a Normal Sleep.
   */
  kSuspendStateNormalSleep,
  /**
   * We are expecting to fall into a Deep Sleep.
   */
  kSuspendStateDeepSleep,
  /**
   * We have just returned from a Normal Sleep.
   */
  kSuspendStateNormalWaking,
  /**
   * We have just returned from a Deep Sleep.
   */
  kSuspendStateDeepWaking,
  /**
   * We've instructed the AON/Wakeup module to relinquish its control of the
   * USB and deactivate.
   */
  kSuspendStateAONWakeup,
  kSuspendStateWaitResumeTimeout,
  /**
   * Waiting for the DPI/Host to decide that the test phase is complete.
   */
  kSuspendStateWaitFinish,
  /**
   * Transition to next test phase, with the device still connected and
   * operational, ie. Resume Signaling has occurred.
   */
  kSuspendStateNextPhase,
  /**
   * Test completed successfully.
   */
  kSuspendStateComplete,
  /**
   * Test failed.
   */
  kSuspendStateFailed,
} usbdev_suspend_state_t;

/**
 * Retained state; to be held in the Retention SRAM during Deep Sleep
 */
typedef struct {
  /**
   * Host-suppplied device address on the USB.
   */
  uint8_t dev_address;
  /**
   * Selected device configuration number.
   */
  uint8_t dev_config;
  /**
   * Test phase.
   */
  uint8_t test_phase;
  /**
   * Unused padding.
   */
  uint8_t pad0;
  uint32_t pad[3];
} usbdev_retn_state_t;

/**
 * Test context
 */
typedef struct usbdev_suspend_ctx {
  /**
   * Access to usb_testutils context
   */
  usb_testutils_ctx_t *usbdev;
  /**
   * Current test state
   */
  usbdev_suspend_state_t test_state;
  /**
   * Current test phase
   */
  usbdev_suspend_phase_t test_phase;
  /**
   * Final test phase (inclusive)
   */
  usbdev_suspend_phase_t fin_phase;
  /**
   * Streaming traffic throughout test?
   */
  bool with_traffic;
  /**
   * Timeout catching any failure of test to advance as expected
   */
  ibex_timeout_t timeout;
  /**
   * Most recent status of wakeup monitor
   */
  dif_usbdev_wake_status_t wake_status;
  /**
   * Test descriptor for current test phase
   */
  uint8_t test_dscr[USB_TESTUTILS_TEST_DSCR_LEN];
  /**
   * Our retained state; transferred to and from Retention SRAM over Sleep
   */
  usbdev_retn_state_t retn_state;
} usbdev_suspend_ctx_t;

/**
 * Retention SRAM start address
 */
//const uint32_t kRetSramBaseAddr = TOP_EARLGREY_SRAM_CTRL_RET_AON_RAM_BASE_ADDR;
/**
 * Retention SRAM address at which we may store some state.
 */
//const uint32_t kRetSramOwnerAddr =
//    kRetSramBaseAddr + offsetof(retention_sram_t, reserved_owner);

/**
 * USB device context types.
 */
static usb_testutils_ctx_t usbdev;
static usb_testutils_controlep_ctx_t usbdev_control;

/**
 * Do we expect this host to put the device into suspend?
 *
 * Note: the only WakeUp stimulus that a FPGA host can currently produce is a
 * a Disconnection event, and that requires manual involvement in the form of
 * unplugging the device.
 */
//static bool host_suspends = true;
#define host_suspends 1
/**
 * Verbose logging? Mostly useful on FPGA; be warned that it can affect
 * timing in simulation, and in particular will likely break Verilator simus.
 */
static bool verbose = false;

/**
 * Context information for suspend/resume test
 */
static usbdev_suspend_ctx_t suspend_ctx;

// Return a timeout in microseconds, scaled for the test target; longer timeout
// periods are more appropriate for FPGA tests and decidedly undesirable for
// Verilator top-level simulations
static uint32_t time_frames(unsigned n) {
  uint32_t scale = 1u;
  return scale * n * USBDPI_FRAME_INTERVAL;
}

#define phase_name(x) "Phase"
#define state_name(x) "State"

// Transition to a (new) test state
static void state_enter(usbdev_suspend_ctx_t *ctx,
                        usbdev_suspend_state_t state) {
  if (verbose) {
    LOG_INFO("entering state %s", state_name(state));
  }
  ctx->test_state = state;
}

#if 1
// Set a time out for the current test state, in microseconds
static void timeout_set(usbdev_suspend_ctx_t *ctx, uint32_t interval_us) {
  if (verbose) {
    uint64_t now = ibex_mcycle_read();
    uint64_t then = now + interval_us;
    LOG_INFO("  setting timeout to 0x%x%x (at 0x%x%x)", (uint32_t)(then >> 32),
             (uint32_t)then, (uint32_t)(now >> 32), (uint32_t)now);
  }
  ctx->timeout = ibex_timeout_init(interval_us);
}
#else
#define timeout_set(...)
#endif

#if 1
// Set a time out, in frames, for the current test state
static void timeout_frames_set(usbdev_suspend_ctx_t *ctx,
                               uint32_t interval_frames) {
  timeout_set(ctx, time_frames(interval_frames));
}
#else
#define timeout_frames_set(...)
#endif

////////////////////////////////////////////////////////////////////////////////
// Continue a phase, following a return from Deep Sleep
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Foreground event handling
//
// Some monitoring activities must occur in the foreground because the
// monitored event is neither immediate nor will it trigger a link
// event.
////////////////////////////////////////////////////////////////////////////////

static status_t state_service(usbdev_suspend_ctx_t *ctx) {
  uint32_t timeout = FRAMES_SUSPEND_MISSED;

  switch (ctx->test_state) {
    default:
      TRY_CHECK(ctx->test_state == kSuspendStateDeepSleep);
      // no break     
    case kSuspendStatePowerOnReset:
    case kSuspendStateBusReset:
      if (ctx->test_state == kSuspendStateDeepSleep) {
        // Collect the device address and configuration previously used.
        const uint8_t dev_address = ctx->retn_state.dev_address;
        const uint8_t dev_config = ctx->retn_state.dev_config;

        // NOTE: We've run through the usb_testutils/controlep_init sequence as
        // normal because this sets up our endpoints as they were before, whilst
        // requiring less information to be stored in the retention RAM, but we
        // must still reinstate the device address.
        CHECK_DIF_OK(dif_usbdev_address_set(ctx->usbdev->dev, dev_address));

        // At this point we expected the device to be in the Powered state, and
        // since it won't see a Bus Reset, we nudge it into the ActiveNoSOF state.
        dif_usbdev_link_state_t link_state;
        TRY(dif_usbdev_status_get_link_state(ctx->usbdev->dev, &link_state));
        TRY_CHECK(link_state == kDifUsbdevLinkStatePowered);

        TRY(dif_usbdev_resume_link_to_active(ctx->usbdev->dev));
      }
      else {
        // In this case, since we have no retention RAM, we must wait until the
        // host has reconfigured us; do not wait indefinitely, in case something
        // has gone wrong. It should take only a few bus frames even with the
        // older, slower DPI behavior.
        if (kDeviceType == kDeviceFpgaCw310) {
          timeout_set(ctx, TIMEOUT_FPGA_CONFIG);
        } else {
          timeout_frames_set(ctx, 8u);
        }

        while (usbdev_control.device_state != kUsbTestutilsDeviceConfigured &&
               !ibex_timeout_check(&ctx->timeout)) {
          CHECK_STATUS_OK(usb_testutils_poll(ctx->usbdev));
        }
      }

      // The DPI model still needs to complete the 'SET_DEVICE_CONFIG' bus frame
      // and then devote another bus frame to reading the test configuration.
      // (GET_TEST_CONFIG), so we must expect to wait longer before seeing the
      // Suspend signaling.
      timeout += FRAMES_CONFIG_DELAY;
      // no break
    case kSuspendStateNextPhase:
      // Enter the appropriate starting state based upon the test phase
      switch (ctx->test_phase) {
        case kSuspendPhaseShutdown:
          // Nothing more to be done
          state_enter(ctx, kSuspendStateComplete);
          break;

        case kSuspendPhaseSuspend:
          state_enter(ctx, kSuspendStateWaitSuspend);
          break;

        default:
          CHECK(ctx->test_phase == kSuspendPhaseDeepDisconnect);
          // no break
        case kSuspendPhaseDeepReset:
        case kSuspendPhaseDeepResume:
// TODO: surely this is state-dependent?
          state_enter(ctx, kSuspendStateDeepWaking);
          break;

        case kSuspendPhaseSleepDisconnect:
        case kSuspendPhaseSleepReset:
        case kSuspendPhaseSleepResume:
          state_enter(ctx, kSuspendStateWaitLongSuspend);
          break;
      }

      // Initialize timeout to catch any failure of the host to suspend the bus
      timeout_frames_set(ctx, timeout);
      break;

    case kSuspendStateActivatedAON:
      // Since the AON/Wakeup operates on a low clock frequency, it may
      // take some time for it to become active....await its signal
      TRY(dif_usbdev_get_wake_status(ctx->usbdev->dev, &ctx->wake_status));

      LOG_INFO("wake status active %u disconnected %u bus_reset %u\n",
               ctx->wake_status.active, ctx->wake_status.disconnected,
               ctx->wake_status.bus_reset);

      if (ctx->wake_status.active) {
        // Retain our state information
        TRY(dif_usbdev_address_get(ctx->usbdev->dev,
                                   &ctx->retn_state.dev_address));
//        ctx->retn_state.dev_config = usbdev_control.usb_config;
        ctx->retn_state.test_phase = (uint8_t)ctx->test_phase;
        if (verbose) {
          LOG_INFO(" - retaining address %u config %u phase %u (%s)",
                   ctx->retn_state.dev_address, ctx->retn_state.dev_config,
                   ctx->retn_state.test_phase,
                   phase_name(ctx->retn_state.test_phase));
        }

//        retention_sram_store(&ctx->retn_state);

// TODO: migrate this into a subfunction
        // Enter low power mode.
        switch (ctx->test_phase) {
          default:
            if (!host_suspends) {
              LOG_INFO("auto-skipping WFI (FPGA)");
              break;
            }
            // no break            
          case kSuspendPhaseSleepDisconnect:
          case kSuspendPhaseDeepDisconnect:
            if (ctx->test_phase == kSuspendPhaseDeepResume ||
                ctx->test_phase == kSuspendPhaseDeepReset ||
                ctx->test_phase == kSuspendPhaseDeepDisconnect) {
              LOG_INFO("Requesting Deep sleep");

              // Record that we've asked to power down; timeout should never occur.
              timeout_frames_set(ctx, 2u);
              state_enter(ctx, kSuspendStateDeepSleep);
            } else {
              LOG_INFO("Requesting Normal sleep");

              // Record that we've asked to enter lower power mode; timeout should
              // never occur.
              timeout_frames_set(ctx, 2u);
              state_enter(ctx, kSuspendStateNormalSleep);
            }

            LOG_INFO("Issuing WFI to enter sleep");
            wait_for_interrupt();

            // Check that a DeepSleep request did not somehow run past the WFI...
            TRY_CHECK(ctx->test_state == kSuspendStateNormalSleep);
            break;
        }

        //---------------------------------------------------------------
        // After a Normal sleep, we resume execution here; after a Deep
        // sleep we start again as if from a Power On Reset, but the
        // pwrmgr tells us otherwise.
        //---------------------------------------------------------------

// TODO: check the IRQ source in the event of a Normal Sleep and proceeding
// past the WFI
        // ... and we should be in one of these test phases.
        TRY_CHECK(ctx->retn_state.test_phase == kSuspendPhaseSleepResume ||
                  ctx->retn_state.test_phase == kSuspendPhaseSleepReset ||
                  ctx->retn_state.test_phase == kSuspendPhaseSleepDisconnect);

        // Retrieve it and check; just additional testing.
        usbdev_retn_state_t stored_state;
        if (verbose) {
          LOG_INFO(" - retained address %u config %u phase %u (%s)",
                   stored_state.dev_address, stored_state.dev_config, stored_state.test_phase, phase_name(stored_state.test_phase));
        }

        // Check that the Retention SRAM did its job over Normal Sleep at least;
        // the SRAM should remain powered and clocked so this should not be
        // challenging.
        TRY_CHECK(stored_state.dev_address == ctx->retn_state.dev_address &&
                  stored_state.dev_config == ctx->retn_state.dev_config &&
                  stored_state.test_phase == ctx->retn_state.test_phase);
        timeout_frames_set(ctx, 1u);
        state_enter(ctx, kSuspendStateNormalWaking);
      }
      break;

    case kSuspendStateNormalWaking:
    case kSuspendStateDeepWaking:
      // We've returned from sleeping; enquire of the USB AON Wake module
      // what happened...
      TRY(dif_usbdev_get_wake_status(ctx->usbdev->dev, &ctx->wake_status));

      // There are three ways that we may exit from Deep Sleep in which
      // the AON/Wake module has been handling the bus:
      // - Disconnecion (loss of VBUS/SENSE)
      // - Bus Reset (from host)
      // - Non-Idle state detected (Resume Signaling; this is inferred by
      //   neither of the other two conditions having occurred.)
      //   Resume signaling shall last at last 20ms, but the AON/Wake
      //   module alerts us long before that time has elapsed.

      LOG_INFO("Awaking: active %u disconnected %u bus_reset %u\n",
               ctx->wake_status.active, ctx->wake_status.disconnected,
               ctx->wake_status.bus_reset);

      // Check the report from the AON/Wakeup module
      if (ctx->wake_status.active) {
        bool got_signal = false;

        switch (ctx->test_phase) {
          case kSuspendPhaseSleepResume:
          case kSuspendPhaseDeepResume:
            got_signal =
                (!ctx->wake_status.bus_reset && !ctx->wake_status.disconnected);
            break;

          case kSuspendPhaseSleepReset:
          case kSuspendPhaseDeepReset:
            got_signal = (ctx->wake_status.bus_reset != 0);
            break;

          default:
            TRY_CHECK(ctx->test_phase == kSuspendPhaseDeepDisconnect);
            // no break
          case kSuspendPhaseSleepDisconnect:
            got_signal = (ctx->wake_status.disconnected != 0);
            break;
        }

        if (got_signal || !host_suspends) {
          // TODO: Issue #18562 VBUS Disconnection leaves pull ups asserted
          // by the USB AON Wake module, so disconnect them here before
          // potential confusion results.
          if (ctx->test_phase == kSuspendPhaseSleepDisconnect) {
            bool sense;

            TRY(dif_usbdev_status_get_sense(ctx->usbdev->dev, &sense));
            if (verbose) {
              LOG_INFO("Handling Disconnection when VBUS %sasserted", sense ? "" : "de-");
            }
            // If VBUS/SENSE is not asserted, then the pull up will be removed
            // as soon as the AON Wake module is deactivated, because usbdev
            // qualifies its own pull up assertions with VBUS/SENSE presence.
            if (sense) {
              TRY(dif_usbdev_interface_enable(ctx->usbdev->dev, false));
            }
          }

          // Signal to the AON wakeup module that it should deactivate and
          // relinquish control of the bus
          TRY(dif_usbdev_set_wake_enable(ctx->usbdev->dev, kDifToggleDisabled));

          // Although it operates at only 200kHz, it should't take long
          timeout_frames_set(ctx, 1u);
          state_enter(ctx, kSuspendStateAONWakeup);
        } else {
          LOG_INFO("Unexpected report from USB AON Wake module");
          state_enter(ctx, kSuspendStateFailed);
        }
      } else {
        LOG_INFO("AON/Wake module not active when expected");
        state_enter(ctx, kSuspendStateFailed);
      }
      break;

    case kSuspendStateAONWakeup:
      // Since the AON wakeup module operates on a much lower clock
      // frequency it may take some time for it to stop monitoring and to
      // report becoming inactive...
      TRY(dif_usbdev_get_wake_status(ctx->usbdev->dev, &ctx->wake_status));

      LOG_INFO("AON Wake module active %u disconnected %u bus_reset %u\n",
               ctx->wake_status.active, ctx->wake_status.disconnected,
               ctx->wake_status.bus_reset);

      if (!ctx->wake_status.active) {
        // If we've been awoken by a Disconnection event or by a Bus Reset
        // event rather than by Resume Signaling, then we must advance to
        // the next test phase and expect to be reconfigured.
        //
        // Note: at this point we may assume that we _did_ get the
        // expected wakeup stimulus/report, because it was checked above.
        switch (ctx->test_phase) {
          case kSuspendPhaseSleepDisconnect:
          case kSuspendPhaseDeepDisconnect:
            state_enter(ctx, kSuspendStatePowerOnReset);
            break;

          case kSuspendPhaseSleepReset:
          case kSuspendPhaseDeepReset:
            state_enter(ctx, kSuspendStateWaitBusReset);
            break;

          default:
            TRY_CHECK(ctx->test_phase == kSuspendPhaseDeepResume);
            // no break
          case kSuspendPhaseSleepResume:
            timeout_set(ctx, TIMEOUT_WAKEUP_RESUME);
            state_enter(ctx, kSuspendStateWaitResumeTimeout);
            break;
        }
      } else {
        LOG_INFO("AON Wake module not active when expected");
        state_enter(ctx, kSuspendStateFailed);
      }
      break;

      // TODO: do we still want this state?
    case kSuspendStateWaitFinish:
      break;

    // Phase-initial states
    case kSuspendStateNormalSleep:
    case kSuspendStateDeepSleep:
      LOG_INFO("Phase-initial state %u (%s) should have been handled in phase_start()", ctx->test_state, state_name(ctx->test_state));
  }

  return OK_STATUS();
}

/**
 * Run a single test phase to completion
 */
static status_t phase_run(usbdev_suspend_ctx_t *ctx) {
  bool phase_done = false;

  do {
    // The DPI model and our callback handler for USB link events do most of the
    // work of walking through the test states until completion
    if (ibex_timeout_check(&ctx->timeout)) {
    } else {
      TRY(state_service(ctx));
    }
    switch (ctx->test_state) {
      // These states terminate the phase, either advancing to the next phase
      // or terminating the test sequence.
      case kSuspendStateBusReset:
      case kSuspendStateNextPhase:
      case kSuspendStatePowerOnReset:
      case kSuspendStateComplete:  // from PhaseShutdown only
      case kSuspendStateFailed:  // from any phase
        phase_done = true;
        break;

      // Do not poll the USB device or perform traffic in these states.
      case kSuspendStateActivatedAON:
      case kSuspendStateAONWakeup:
      case kSuspendStateDeepSleep:
      case kSuspendStateNormalSleep:
        break;

      default:
        if (ctx->with_traffic) {
        } else {
          // No traffic, but we must still poll the usb_testutils layer to handle
          // hardware events and callbacks.
          CHECK_STATUS_OK(usb_testutils_poll(ctx->usbdev));
        }
        break;
    }
  } while (!phase_done);

  return OK_STATUS();
}

OTTF_DEFINE_TEST_CONFIG();

bool test_main(void) {

  usbdev_suspend_ctx_t *ctx = &suspend_ctx;

  do {
    // Run this test phase
    CHECK_STATUS_OK(phase_run(ctx));

    // Keep going if we're advancing to the next phase.
    //  (NextPhase means that we advance whilst still active and can thus skip
    //   device setup and configuratinon)
  } while (ctx->test_state == kSuspendStateNextPhase ||  // from Resume
           ctx->test_state == kSuspendStateBusReset ||  // after Bus Reset
           ctx->test_state == kSuspendStatePowerOnReset);  // after Disconnect

  return true;
}

