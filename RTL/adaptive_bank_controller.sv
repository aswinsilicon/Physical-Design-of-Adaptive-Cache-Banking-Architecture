// adaptive_bank_controller.sv
// One ACTIVE/STANDBY/SLEEP power-state FSM per bank (see
// docs/02_architecture.md section 5 for the full diagram/rationale).
//
// A bank's idle counter runs while it is not the target of the current
// request; reaching STANDBY_TIMEOUT (measured from ACTIVE) drops it to
// STANDBY, and a further SLEEP_TIMEOUT idle cycles (measured fresh from the
// moment STANDBY was entered) drops it to SLEEP. A request targeting a
// non-ACTIVE bank starts a wake counter; once it reaches the state's wake
// latency (STANDBY_WAKE_CYC or SLEEP_WAKE_CYC) the bank returns to ACTIVE.
// Storage contents are never touched by these transitions (see
// tag_array.sv/data_array.sv) -- only bank_enable, which gates array
// reads/writes, changes.
//
// bank_state is exposed as a plain 2-bit-per-bank packed vector (values are
// cache_pkg::BANK_ACTIVE/STANDBY/SLEEP) rather than an enum-typed port/array
// port, for the same Icarus-build compatibility reason documented in
// cache_pkg.sv next to POLICY_LRU/POLICY_FIFO.

module adaptive_bank_controller
  import cache_pkg::*;
#(
  parameter int NUM_BANKS        = 4,
  parameter int STANDBY_TIMEOUT  = 16,
  parameter int SLEEP_TIMEOUT    = 64,
  parameter int STANDBY_WAKE_CYC = 1,
  parameter int SLEEP_WAKE_CYC   = 4
)(
  input  logic clk,
  input  logic rst_n,

  input  logic                      req_valid,    // controller wants req_bank_sel usable
  input  logic [$clog2(NUM_BANKS)-1:0] req_bank_sel,

  output logic [NUM_BANKS-1:0]      bank_enable,  // 1 = ACTIVE
  output logic                      bank_ready,   // req_bank_sel's bank is ACTIVE
  output logic [NUM_BANKS-1:0][1:0] bank_state    // debug/coverage visibility
);

  localparam int IDLE_MAX     = (STANDBY_TIMEOUT > SLEEP_TIMEOUT) ? STANDBY_TIMEOUT : SLEEP_TIMEOUT;
  localparam int WAKE_MAX     = (STANDBY_WAKE_CYC > SLEEP_WAKE_CYC) ? STANDBY_WAKE_CYC : SLEEP_WAKE_CYC;
  localparam int IDLE_CNT_W   = $clog2(IDLE_MAX + 1);
  localparam int WAKE_CNT_W   = $clog2(WAKE_MAX + 1);

  genvar b;
  generate
    for (b = 0; b < NUM_BANKS; b = b + 1) begin : g_bank

      logic [IDLE_CNT_W-1:0] idle_cnt;
      logic [WAKE_CNT_W-1:0] wake_cnt;
      logic [1:0]            state;
      logic                  targeted;

      assign targeted = req_valid && (req_bank_sel == b[$clog2(NUM_BANKS)-1:0]);

      // idle_cnt/wake_cnt are sized to the max timeout/wake parameter in use
      // (IDLE_CNT_W/WAKE_CNT_W), narrower than the 32-bit int timeout/wake
      // parameters they are compared against below -- a safe, intentional
      // width extension of the comparison RHS, not a truncation. When a
      // *_WAKE_CYC parameter is 1, "wake_cnt >= *_WAKE_CYC-1" is trivially
      // true (wake completes the same cycle it is requested); that is the
      // intended single-cycle-wake behavior, not a lint bug, so the
      // resulting UNSIGNED "constant comparison" lint is waived too.
      // verilator lint_off WIDTH
      // verilator lint_off UNSIGNED
      always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
          state    <= cache_pkg::BANK_ACTIVE;
          idle_cnt <= {IDLE_CNT_W{1'b0}};
          wake_cnt <= {WAKE_CNT_W{1'b0}};
        end else begin
          case (state)

            cache_pkg::BANK_ACTIVE: begin
              if (targeted) begin
                idle_cnt <= {IDLE_CNT_W{1'b0}};
              end else if (idle_cnt >= STANDBY_TIMEOUT - 1) begin
                state    <= cache_pkg::BANK_STANDBY;
                idle_cnt <= {IDLE_CNT_W{1'b0}};
              end else begin
                idle_cnt <= idle_cnt + 1'b1;
              end
            end

            cache_pkg::BANK_STANDBY: begin
              if (targeted) begin
                if (wake_cnt >= STANDBY_WAKE_CYC - 1) begin
                  state    <= cache_pkg::BANK_ACTIVE;
                  idle_cnt <= {IDLE_CNT_W{1'b0}};
                  wake_cnt <= {WAKE_CNT_W{1'b0}};
                end else begin
                  wake_cnt <= wake_cnt + 1'b1;
                end
              end else if (idle_cnt >= SLEEP_TIMEOUT - 1) begin
                state    <= cache_pkg::BANK_SLEEP;
                idle_cnt <= {IDLE_CNT_W{1'b0}};
              end else begin
                idle_cnt <= idle_cnt + 1'b1;
              end
            end

            cache_pkg::BANK_SLEEP: begin
              if (targeted) begin
                if (wake_cnt >= SLEEP_WAKE_CYC - 1) begin
                  state    <= cache_pkg::BANK_ACTIVE;
                  idle_cnt <= {IDLE_CNT_W{1'b0}};
                  wake_cnt <= {WAKE_CNT_W{1'b0}};
                end else begin
                  wake_cnt <= wake_cnt + 1'b1;
                end
              end
            end

            default: state <= cache_pkg::BANK_ACTIVE;

          endcase
        end
      end
      // verilator lint_on WIDTH
      // verilator lint_on UNSIGNED

      assign bank_enable[b] = (state == cache_pkg::BANK_ACTIVE);
      assign bank_state[b]  = state;

    end
  endgenerate

  assign bank_ready = bank_enable[req_bank_sel];

endmodule : adaptive_bank_controller