
#include "Vtop_pkg.h"
#define VTOP Vtop_pkg

#include "verilated.h"
#include "verilated_fst_c.h"

int main(int argc, char** argv, char** env) {
  VerilatedContext* contextp = new VerilatedContext;
  contextp->commandArgs(argc, argv);

  // Verilator must compute traced signals
  contextp->traceEverOn(true);
  VerilatedFstC *trace = new VerilatedFstC;

  VTOP* top = new VTOP{contextp};
  svSetScope(svGetScopeFromName("tb"));

  top->trace(trace, 10);
  trace->open("sim.fst");

  top->clk_i  = 0;
  top->rst_ni = 0;

  top->clk_aon_i = 0;
  top->rst_aon_ni = 0;

  // Clock half-periods in picoseconds
  uint32_t clk_aon_hperiod = 1000*1000*1000 / 200;  // 100kHz.
  uint32_t clk_hperiod = 1000*1000 / 100;  // 50MHz.

  // Start the clocks out-of-phase.
  int32_t clk_aon_phase = clk_aon_hperiod;
  int32_t clk_phase = clk_hperiod;

  uint64_t sim_time = 0u;
  while (!contextp->gotFinish()) {
    // Ascertain when the next clock edge occurs
    int32_t t_del = (clk_aon_phase < clk_phase) ? clk_aon_phase : clk_phase;

    trace->dump(sim_time);

    clk_aon_phase -= t_del;
    if (clk_aon_phase <= 0) {
      top->clk_aon_i = !top->clk_aon_i;
      clk_aon_phase += clk_aon_hperiod;

      if (!top->rst_aon_ni) {
        if (contextp->time() > 1 && contextp->time() < clk_aon_hperiod * 5) {
          top->rst_aon_ni = 0;  // Assert reset
        } else {
          top->rst_aon_ni = 1;  // Deassert reset
        }
      }
    }

    clk_phase -= t_del;
    if (clk_phase <= 0) {
      top->clk_i = !top->clk_i;
      clk_phase += clk_hperiod;

      // Toggle control signals on an edge that doesn't correspond
      // to where the controls are sampled; in this example we do
      // this only on a negedge of clk, because we know
      // reset is not sampled there.
      if (!top->clk_i) {
        if (contextp->time() > 1 && contextp->time() < clk_hperiod * 5) {
          top->rst_ni = 0;  // Assert reset
        } else {
          top->rst_ni = 1;  // Deassert reset
        }
      }
    }

    contextp->timeInc(t_del);
    sim_time += t_del;

    top->eval();
  }

  trace->close();

  delete top;
  delete contextp;
  return 0;
}

