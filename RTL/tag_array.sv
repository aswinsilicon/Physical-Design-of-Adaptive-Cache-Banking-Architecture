// tag_array.sv
// Tag / valid / dirty storage for all banks and ways. Architecturally this
// represents three logical arrays (Tag Memory, Valid Bit Array, Dirty Bit
// Array) bundled into one storage element per (bank, way, set) because they
// always share the same address and update timing -- standard practice, and
// avoids three separately-addressed memories that would only ever be
// accessed in lockstep.
//
// Read is combinational (flip-flop / register-file style array), matching a
// single-cycle L1 hit-detection assumption appropriate at this cache scale.
// For much larger configurations this would need to move to a registered
// (1-cycle-latency) SRAM read with an extra controller pipeline stage --
// flagged here rather than silently assumed.
//
// Only the valid bits are reset. Tag/dirty bits of an invalid line are
// don't-cares (never read as meaningful until valid=1 is set by a fill),
// which mirrors how real SRAM-mapped arrays behave -- they cannot be
// instantly reset in silicon, only small dedicated valid flops can.

module tag_array #(
  parameter int NUM_BANKS     = 4,
  parameter int NUM_WAYS      = 4,
  parameter int SETS_PER_BANK = 16,
  parameter int TAG_BITS      = 22
)(
  input  logic clk,
  input  logic rst_n,

  input  logic [$clog2(NUM_BANKS)-1:0]     bank_sel,
  input  logic [$clog2(SETS_PER_BANK)-1:0] set_idx,
  input  logic [NUM_BANKS-1:0]             bank_enable,

  // Full-line fill (miss refill): installs a new tag, sets valid, clears dirty.
  input  logic                             fill_en,
  input  logic [$clog2(NUM_WAYS)-1:0]      fill_way,
  input  logic [TAG_BITS-1:0]              fill_tag,
  input  logic                             fill_dirty,

  // Dirty-bit-only update (write hit on an already-resident line).
  input  logic                             dirty_set_en,
  input  logic [$clog2(NUM_WAYS)-1:0]      dirty_set_way,

  output logic [NUM_WAYS-1:0][TAG_BITS-1:0] tag_out,
  output logic [NUM_WAYS-1:0]               valid_out,
  output logic [NUM_WAYS-1:0]               dirty_out
);

  localparam int DEPTH = NUM_BANKS * NUM_WAYS * SETS_PER_BANK;

  logic [TAG_BITS-1:0] tag_mem   [DEPTH];
  logic                valid_mem [DEPTH];
  logic                dirty_mem [DEPTH];

  // Callers pass narrower unsigned index fields (bank_sel/way/set_idx) into
  // these wider `int` formals: a safe zero-extension, not a truncation, so
  // the lint WIDTH warning on the call sites below is waived rather than
  // "fixed" with a cast -- an explicit `int'()` self-determined cast trips
  // an internal assertion in this Icarus 10.2 build's signedness handling
  // (signed `int` target vs. unsigned source), so casting is avoided
  // entirely rather than fixing one tool's cosmetic warning by crashing
  // the other tool.
  function automatic int flat_addr(input int bank, input int way, input int set);
    return (bank * NUM_WAYS + way) * SETS_PER_BANK + set;
  endfunction
  // verilator lint_off WIDTH

  integer i;

  // NOTE: this codebase targets Icarus Verilog 10.2 and Verilator 4.024 as
  // its two simulators; this Icarus build does not accept `always_ff` /
  // `always_comb` (confirmed by direct test), so all sequential and
  // combinational logic in this design deliberately uses classic
  // `always @(posedge clk or negedge rst_n)` / `always @(*)` blocks instead.
  // These are equally synthesizable and semantically equivalent to the
  // SV-2012 forms; only the extra lint-time intent-checking is given up.
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      // Blocking assignment in the reset loop (order-independent, all set to
      // the same constant) -- Verilator 4.024 rejects a non-blocking
      // assignment to the loop-variable-indexed target inside a for loop.
      // verilator lint_off BLKSEQ
      for (i = 0; i < DEPTH; i = i + 1) begin
        valid_mem[i] = 1'b0;
        dirty_mem[i] = 1'b0;
      end
      // verilator lint_on BLKSEQ
    end else if (bank_enable[bank_sel]) begin
      if (fill_en) begin
        tag_mem[flat_addr(bank_sel, fill_way, set_idx)]   <= fill_tag;
        valid_mem[flat_addr(bank_sel, fill_way, set_idx)] <= 1'b1;
        dirty_mem[flat_addr(bank_sel, fill_way, set_idx)] <= fill_dirty;
      end else if (dirty_set_en) begin
        dirty_mem[flat_addr(bank_sel, dirty_set_way, set_idx)] <= 1'b1;
      end
    end
  end

  genvar gw;
  generate
    // NOTE: `gw = gw + 1`, not `gw++` -- this Icarus build's generate-for
    // parser rejects the increment/decrement operators inside genvar loops.
    for (gw = 0; gw < NUM_WAYS; gw = gw + 1) begin : g_read
      assign tag_out[gw]   = tag_mem[flat_addr(bank_sel, gw, set_idx)];
      assign valid_out[gw] = valid_mem[flat_addr(bank_sel, gw, set_idx)];
      assign dirty_out[gw] = dirty_mem[flat_addr(bank_sel, gw, set_idx)];
    end
  endgenerate
  // verilator lint_on WIDTH

endmodule : tag_array