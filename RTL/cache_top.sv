// cache_top.sv
// Top-level parameterized cache: wires cache_controller to tag_array,
// data_array, hit_miss_unit, replacement_unit and adaptive_bank_controller.
// See docs/01_specification.md for parameters/interfaces and
// docs/02_architecture.md for the block diagram this instantiates.

module cache_top
  import cache_pkg::*;
#(
  parameter int ADDR_WIDTH   = 32,
  parameter int DATA_WIDTH   = 32,
  parameter int CACHE_SIZE   = 4096, // bytes
  parameter int LINE_SIZE    = 16,   // bytes
  parameter int NUM_WAYS     = 4,
  parameter int NUM_BANKS    = 4,
  parameter logic [1:0] REPLACEMENT_POLICY = cache_pkg::POLICY_LRU,
  parameter int STANDBY_TIMEOUT  = 16,
  parameter int SLEEP_TIMEOUT    = 64,
  parameter int STANDBY_WAKE_CYC = 1,
  parameter int SLEEP_WAKE_CYC   = 4
)(
  input  logic clk,
  input  logic rst_n,

  // ---- CPU interface ----
  input  logic                          req_valid,
  input  logic [ADDR_WIDTH-1:0]         req_addr,
  input  logic                          req_we,
  input  logic [DATA_WIDTH-1:0]         req_wdata,
  input  logic [(DATA_WIDTH/8)-1:0]     req_wstrb,
  output logic                          req_ready,
  output logic                          resp_valid,
  output logic [DATA_WIDTH-1:0]         resp_rdata,
  output logic                          resp_hit,

  // ---- memory-side interface (backing store / next level) ----
  output logic                          mem_req_valid,
  output logic [ADDR_WIDTH-1:0]         mem_addr,
  output logic                          mem_we,
  output logic [LINE_SIZE*8-1:0]        mem_wdata,
  input  logic                          mem_ready,
  input  logic                          mem_rvalid,
  input  logic [LINE_SIZE*8-1:0]        mem_rdata,

  // ---- debug / coverage visibility only (not part of the functional protocol) ----
  output logic [NUM_BANKS-1:0][1:0]     bank_state_dbg
);

  localparam int NUM_SETS      = cache_pkg::num_sets_f(CACHE_SIZE, LINE_SIZE, NUM_WAYS);
  localparam int SETS_PER_BANK = NUM_SETS / NUM_BANKS;
  localparam int TAG_BITS      = cache_pkg::tag_bits_f(ADDR_WIDTH, CACHE_SIZE, LINE_SIZE, NUM_WAYS);
  localparam int WAY_BITS      = $clog2(NUM_WAYS);
  localparam int BANK_BITS     = $clog2(NUM_BANKS);
  localparam int SET_BITS      = $clog2(NUM_SETS);

  // synthesis translate_off
  initial begin
    if (!cache_pkg::is_pow2_f(CACHE_SIZE))
      $fatal(1, "cache_top: CACHE_SIZE (%0d) must be a power of two", CACHE_SIZE);
    if (!cache_pkg::is_pow2_f(LINE_SIZE))
      $fatal(1, "cache_top: LINE_SIZE (%0d) must be a power of two", LINE_SIZE);
    if (!cache_pkg::is_pow2_f(NUM_WAYS))
      $fatal(1, "cache_top: NUM_WAYS (%0d) must be a power of two", NUM_WAYS);
    if (!cache_pkg::is_pow2_f(NUM_BANKS))
      $fatal(1, "cache_top: NUM_BANKS (%0d) must be a power of two", NUM_BANKS);
    if (LINE_SIZE < (DATA_WIDTH / 8))
      $fatal(1, "cache_top: LINE_SIZE (%0d bytes) must be >= DATA_WIDTH/8 (%0d bytes)",
             LINE_SIZE, DATA_WIDTH / 8);
    if ((NUM_SETS % NUM_BANKS) != 0)
      $fatal(1, "cache_top: NUM_SETS (%0d) must be an integer multiple of NUM_BANKS (%0d)",
             NUM_SETS, NUM_BANKS);
    if (SET_BITS != (BANK_BITS + $clog2(SETS_PER_BANK)))
      $fatal(1, "cache_top: bank/set-in-bank split does not reconstruct the full index width");
  end
  // synthesis translate_on

  // ---- inter-block wiring ----
  logic [$clog2(NUM_BANKS)-1:0]      bank_sel;
  logic [$clog2(SETS_PER_BANK)-1:0]  set_idx;
  logic [TAG_BITS-1:0]               req_tag;

  logic [NUM_WAYS-1:0][TAG_BITS-1:0] tag_out;
  logic [NUM_WAYS-1:0]               valid_out;
  logic [NUM_WAYS-1:0]               dirty_out;

  logic                              tag_fill_en;
  logic [WAY_BITS-1:0]               tag_fill_way;
  logic                              tag_dirty_set_en;
  logic [WAY_BITS-1:0]               tag_dirty_set_way;

  logic                              hit;
  logic [WAY_BITS-1:0]               hit_way;

  logic [WAY_BITS-1:0]               victim_way;
  logic                              rep_access_en;
  logic [WAY_BITS-1:0]               rep_access_way;
  logic                              rep_fill_en;
  logic [WAY_BITS-1:0]               rep_fill_way;

  logic [LINE_SIZE*8-1:0]            line_out;
  logic                              data_fill_en;
  logic [WAY_BITS-1:0]               data_fill_way;
  logic                              word_write_en;
  logic [WAY_BITS-1:0]               word_write_way;
  logic [$clog2((LINE_SIZE*8)/DATA_WIDTH)-1:0] word_idx;
  logic [DATA_WIDTH-1:0]             word_wdata;
  logic [(DATA_WIDTH/8)-1:0]         word_wstrb;
  logic [WAY_BITS-1:0]               data_read_way;

  logic [NUM_BANKS-1:0]              bank_enable;
  logic                              bank_ready;
  logic                              bankctrl_req_valid;

  cache_controller #(
    .ADDR_WIDTH    (ADDR_WIDTH),
    .DATA_WIDTH    (DATA_WIDTH),
    .LINE_SIZE     (LINE_SIZE),
    .NUM_WAYS      (NUM_WAYS),
    .NUM_BANKS     (NUM_BANKS),
    .SETS_PER_BANK (SETS_PER_BANK),
    .TAG_BITS      (TAG_BITS)
  ) u_controller (
    .clk               (clk),
    .rst_n             (rst_n),

    .req_valid         (req_valid),
    .req_addr          (req_addr),
    .req_we            (req_we),
    .req_wdata         (req_wdata),
    .req_wstrb         (req_wstrb),
    .req_ready         (req_ready),
    .resp_valid        (resp_valid),
    .resp_rdata        (resp_rdata),
    .resp_hit          (resp_hit),

    .bank_sel          (bank_sel),
    .set_idx           (set_idx),
    .req_tag           (req_tag),

    .tag_in            (tag_out),
    .dirty_in          (dirty_out),
    .tag_fill_en       (tag_fill_en),
    .tag_fill_way      (tag_fill_way),
    .tag_dirty_set_en  (tag_dirty_set_en),
    .tag_dirty_set_way (tag_dirty_set_way),

    .hit_in            (hit),
    .hit_way_in        (hit_way),

    .victim_way_in     (victim_way),
    .rep_access_en     (rep_access_en),
    .rep_access_way    (rep_access_way),
    .rep_fill_en       (rep_fill_en),
    .rep_fill_way      (rep_fill_way),

    .line_in           (line_out),
    .data_fill_en      (data_fill_en),
    .data_fill_way     (data_fill_way),
    .word_write_en     (word_write_en),
    .word_write_way    (word_write_way),
    .word_idx          (word_idx),
    .word_wdata        (word_wdata),
    .word_wstrb        (word_wstrb),
    .data_read_way     (data_read_way),

    .bank_enable_in    (bank_enable),
    .bank_ready_in     (bank_ready),
    .bankctrl_req_valid(bankctrl_req_valid),

    .mem_req_valid     (mem_req_valid),
    .mem_addr          (mem_addr),
    .mem_we            (mem_we),
    .mem_wdata         (mem_wdata),
    .mem_ready         (mem_ready),
    .mem_rvalid        (mem_rvalid)
  );

  tag_array #(
    .NUM_BANKS     (NUM_BANKS),
    .NUM_WAYS      (NUM_WAYS),
    .SETS_PER_BANK (SETS_PER_BANK),
    .TAG_BITS      (TAG_BITS)
  ) u_tag_array (
    .clk           (clk),
    .rst_n         (rst_n),
    .bank_sel      (bank_sel),
    .set_idx       (set_idx),
    .bank_enable   (bank_enable),
    .fill_en       (tag_fill_en),
    .fill_way      (tag_fill_way),
    .fill_tag      (req_tag),
    .fill_dirty    (1'b0),
    .dirty_set_en  (tag_dirty_set_en),
    .dirty_set_way (tag_dirty_set_way),
    .tag_out       (tag_out),
    .valid_out     (valid_out),
    .dirty_out     (dirty_out)
  );

  data_array #(
    .NUM_BANKS     (NUM_BANKS),
    .NUM_WAYS      (NUM_WAYS),
    .SETS_PER_BANK (SETS_PER_BANK),
    .LINE_SIZE     (LINE_SIZE),
    .DATA_WIDTH    (DATA_WIDTH)
  ) u_data_array (
    .clk            (clk),
    .bank_sel       (bank_sel),
    .set_idx        (set_idx),
    .bank_enable    (bank_enable),
    .fill_en        (data_fill_en),
    .fill_way       (data_fill_way),
    .fill_line      (mem_rdata),
    .word_write_en  (word_write_en),
    .word_write_way (word_write_way),
    .word_idx       (word_idx),
    .word_wdata     (word_wdata),
    .word_wstrb     (word_wstrb),
    .read_way       (data_read_way),
    .line_out       (line_out)
  );

  hit_miss_unit #(
    .NUM_WAYS (NUM_WAYS),
    .TAG_BITS (TAG_BITS)
  ) u_hit_miss (
    .valid_in (valid_out),
    .tag_in   (tag_out),
    .req_tag  (req_tag),
    .hit      (hit),
    .hit_way  (hit_way)
  );

  replacement_unit #(
    .NUM_BANKS     (NUM_BANKS),
    .NUM_WAYS      (NUM_WAYS),
    .SETS_PER_BANK (SETS_PER_BANK),
    .POLICY        (REPLACEMENT_POLICY)
  ) u_replacement (
    .clk         (clk),
    .rst_n       (rst_n),
    .bank_sel    (bank_sel),
    .set_idx     (set_idx),
    .access_en   (rep_access_en),
    .access_way  (rep_access_way),
    .fill_en     (rep_fill_en),
    .fill_way    (rep_fill_way),
    .victim_way  (victim_way)
  );

  adaptive_bank_controller #(
    .NUM_BANKS        (NUM_BANKS),
    .STANDBY_TIMEOUT  (STANDBY_TIMEOUT),
    .SLEEP_TIMEOUT    (SLEEP_TIMEOUT),
    .STANDBY_WAKE_CYC (STANDBY_WAKE_CYC),
    .SLEEP_WAKE_CYC   (SLEEP_WAKE_CYC)
  ) u_bank_ctrl (
    .clk          (clk),
    .rst_n        (rst_n),
    .req_valid    (bankctrl_req_valid),
    .req_bank_sel (bank_sel),
    .bank_enable  (bank_enable),
    .bank_ready   (bank_ready),
    .bank_state   (bank_state_dbg)
  );

endmodule : cache_top