// replacement_unit.sv
// Victim-way selection, per (bank, set). REPLACEMENT_POLICY is an
// elaboration-time parameter: only the selected policy's storage/logic is
// generated, the other branch is fully elided.
//
// - LRU: per-way age counters (a permutation of 0..NUM_WAYS-1 per set).
//     On any touch (hit, or a fresh fill) the touched way jumps to
//     NUM_WAYS-1 (MRU) and every way that was more-recently-used than
//     the touched way's old age shifts down by one. Victim = age 0.
// - FIFO: a single per-set round-robin pointer, advanced only on fill_en
//     (i.e. only when a way is actually replaced, not on every hit).
//     Victim = current pointer value.
//
// access_en/access_way are unused in FIFO mode (FIFO never reorders on a
// hit) -- ports are kept uniform across both policies so cache_controller
// doesn't need policy-conditional wiring; the resulting UNUSED lint on
// those two ports under FIFO is expected and waived below.

// verilator lint_off UNUSED
module replacement_unit
  import cache_pkg::*;
#(
  parameter int         NUM_BANKS     = 4,
  parameter int         NUM_WAYS      = 4,
  parameter int         SETS_PER_BANK = 16,
  parameter logic [1:0] POLICY        = POLICY_LRU
)(
  input  logic clk,
  input  logic rst_n,

  input  logic [$clog2(NUM_BANKS)-1:0]     bank_sel,
  input  logic [$clog2(SETS_PER_BANK)-1:0] set_idx,

  input  logic                             access_en,   // genuine hit touch (LRU only)
  input  logic [$clog2(NUM_WAYS)-1:0]      access_way,

  input  logic                             fill_en,     // a way was just refilled
  input  logic [$clog2(NUM_WAYS)-1:0]      fill_way,

  output logic [$clog2(NUM_WAYS)-1:0]      victim_way
);

  localparam int NSETS    = NUM_BANKS * SETS_PER_BANK;
  localparam int WAY_BITS = $clog2(NUM_WAYS);

  function automatic int flat_set(input int bank, input int set);
    return bank * SETS_PER_BANK + set;
  endfunction

  // verilator lint_off WIDTH
  generate
    if (POLICY == POLICY_LRU) begin : g_lru

      logic [WAY_BITS-1:0] age [NSETS][NUM_WAYS];
      integer ss, w;
      integer touch_idx;
      logic [WAY_BITS-1:0] touch_way;
      logic [WAY_BITS-1:0] old_age;

      always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
          // verilator lint_off BLKSEQ
          for (ss = 0; ss < NSETS; ss = ss + 1) begin
            for (w = 0; w < NUM_WAYS; w = w + 1) begin
              age[ss][w] = w[WAY_BITS-1:0];
            end
          end
          // verilator lint_on BLKSEQ
        end else if (access_en || fill_en) begin
          // Scratch (not state-carrying) blocking assigns used to stage the
          // touched way/index for the non-blocking state update below --
          // waived rather than switched to non-blocking, since re-reading a
          // signal driven non-blocking earlier in the same always block
          // would still see the *old* value this same cycle.
          // verilator lint_off BLKSEQ
          touch_idx = flat_set(bank_sel, set_idx);
          touch_way = access_en ? access_way : fill_way;
          old_age   = age[touch_idx][touch_way];
          // verilator lint_on BLKSEQ
          for (w = 0; w < NUM_WAYS; w = w + 1) begin
            if (w[WAY_BITS-1:0] == touch_way) begin
              age[touch_idx][w] <= NUM_WAYS - 1;
            end else if (age[touch_idx][w] > old_age) begin
              age[touch_idx][w] <= age[touch_idx][w] - 1'b1;
            end
          end
        end
      end

      integer vw;
      always @(*) begin
        victim_way = {WAY_BITS{1'b0}};
        for (vw = 0; vw < NUM_WAYS; vw = vw + 1) begin
          if (age[flat_set(bank_sel, set_idx)][vw] == {WAY_BITS{1'b0}}) begin
            victim_way = vw[WAY_BITS-1:0];
          end
        end
      end

    end else begin : g_fifo

      logic [WAY_BITS-1:0] ptr [NSETS];
      integer ss;

      always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
          // verilator lint_off BLKSEQ
          for (ss = 0; ss < NSETS; ss = ss + 1) ptr[ss] <= {WAY_BITS{1'b0}};
          // verilator lint_on BLKSEQ
        end else if (fill_en) begin
          if (ptr[flat_set(bank_sel, set_idx)] == NUM_WAYS - 1) begin
            ptr[flat_set(bank_sel, set_idx)] <= {WAY_BITS{1'b0}};
          end else begin
            ptr[flat_set(bank_sel, set_idx)] <= ptr[flat_set(bank_sel, set_idx)] + 1'b1;
          end
        end
      end

      assign victim_way = ptr[flat_set(bank_sel, set_idx)];

    end
  endgenerate
  // verilator lint_on WIDTH
  // verilator lint_on UNUSED

endmodule : replacement_unit