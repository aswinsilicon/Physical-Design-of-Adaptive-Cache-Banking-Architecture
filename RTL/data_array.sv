// data_array.sv
// Line data storage for all banks and ways. Combinational read (see
// tag_array.sv header for the single-cycle-hit rationale/scalability note).
// No reset on the storage contents themselves: a real SRAM macro cannot be
// reset in one cycle, and correctness never depends on it -- a line only
// becomes readable once tag_array's valid bit for that (bank, way, set) is
// set by a fill, so pre-reset garbage is never observably read.

module data_array #(
  parameter int NUM_BANKS     = 4,
  parameter int NUM_WAYS      = 4,
  parameter int SETS_PER_BANK = 16,
  parameter int LINE_SIZE     = 16, // bytes
  parameter int DATA_WIDTH    = 32  // bits
)(
  input  logic clk,

  input  logic [$clog2(NUM_BANKS)-1:0]     bank_sel,
  input  logic [$clog2(SETS_PER_BANK)-1:0] set_idx,
  input  logic [NUM_BANKS-1:0]             bank_enable,

  // Full-line fill (miss refill).
  input  logic                             fill_en,
  input  logic [$clog2(NUM_WAYS)-1:0]      fill_way,
  input  logic [LINE_SIZE*8-1:0]           fill_line,

  // Single-word write with byte-lane merge (write hit / post-refill store).
  input  logic                             word_write_en,
  input  logic [$clog2(NUM_WAYS)-1:0]      word_write_way,
  input  logic [$clog2((LINE_SIZE*8)/DATA_WIDTH)-1:0] word_idx,
  input  logic [DATA_WIDTH-1:0]            word_wdata,
  input  logic [(DATA_WIDTH/8)-1:0]        word_wstrb,

  // Combinational line read (CPU load data slice / victim writeback source).
  input  logic [$clog2(NUM_WAYS)-1:0]      read_way,
  output logic [LINE_SIZE*8-1:0]           line_out
);

  localparam int DEPTH         = NUM_BANKS * NUM_WAYS * SETS_PER_BANK;
  localparam int BYTES_PER_WORD = DATA_WIDTH / 8;

  logic [LINE_SIZE*8-1:0] mem [DEPTH];

  function automatic int flat_addr(input int bank, input int way, input int set);
    return (bank * NUM_WAYS + way) * SETS_PER_BANK + set;
  endfunction
  // verilator lint_off WIDTH

  integer bi;

  always @(posedge clk) begin
    if (bank_enable[bank_sel]) begin
      if (fill_en) begin
        mem[flat_addr(bank_sel, fill_way, set_idx)] <= fill_line;
      end else if (word_write_en) begin
        for (bi = 0; bi < BYTES_PER_WORD; bi = bi + 1) begin
          if (word_wstrb[bi]) begin
            mem[flat_addr(bank_sel, word_write_way, set_idx)]
               [(word_idx*DATA_WIDTH) + (bi*8) +: 8] <= word_wdata[bi*8 +: 8];
          end
        end
      end
    end
  end

  assign line_out = mem[flat_addr(bank_sel, read_way, set_idx)];
  // verilator lint_on WIDTH

endmodule : data_array