// cache_controller.sv
// Main cache FSM: request handshaking, bank wake sequencing, hit/miss
// detection sequencing, victim writeback + refill orchestration, and word
// read/write servicing. See docs/02_architecture.md section 4 for the FSM
// diagram this directly implements.
//
// This module is deliberately "thick": it drives every control signal into
// tag_array / data_array / replacement_unit / adaptive_bank_controller, and
// reads back their combinational outputs the same cycle (all four arrays
// are combinational-read, see tag_array.sv/data_array.sv headers), which is
// what keeps the hit path single-cycle. cache_top.sv wires this module's
// outputs directly to the other blocks' like-named inputs.

module cache_controller
  import cache_pkg::*;
#(
  parameter int ADDR_WIDTH     = 32,
  parameter int DATA_WIDTH     = 32,
  parameter int LINE_SIZE      = 16,
  parameter int NUM_WAYS       = 4,
  parameter int NUM_BANKS      = 4,
  parameter int SETS_PER_BANK  = 16,
  parameter int TAG_BITS       = 22
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

  // ---- shared address decode (driven to tag_array/data_array/replacement_unit/hit_miss_unit) ----
  output logic [$clog2(NUM_BANKS)-1:0]      bank_sel,
  output logic [$clog2(SETS_PER_BANK)-1:0]  set_idx,
  output logic [TAG_BITS-1:0]               req_tag,

  // ---- tag_array control ----
  // (valid_in is intentionally not a port here: hit/miss is already fully
  // resolved by hit_miss_unit, which consumes valid_in directly -- this
  // module only needs the victim way's tag/dirty bits on a miss.)
  input  logic [NUM_WAYS-1:0][TAG_BITS-1:0] tag_in,
  input  logic [NUM_WAYS-1:0]               dirty_in,
  output logic                              tag_fill_en,
  output logic [$clog2(NUM_WAYS)-1:0]       tag_fill_way,
  output logic                              tag_dirty_set_en,
  output logic [$clog2(NUM_WAYS)-1:0]       tag_dirty_set_way,

  // ---- hit_miss_unit ----
  input  logic                              hit_in,
  input  logic [$clog2(NUM_WAYS)-1:0]       hit_way_in,

  // ---- replacement_unit ----
  input  logic [$clog2(NUM_WAYS)-1:0]       victim_way_in,
  output logic                              rep_access_en,
  output logic [$clog2(NUM_WAYS)-1:0]       rep_access_way,
  output logic                              rep_fill_en,
  output logic [$clog2(NUM_WAYS)-1:0]       rep_fill_way,

  // ---- data_array control ----
  input  logic [LINE_SIZE*8-1:0]            line_in,
  output logic                              data_fill_en,
  output logic [$clog2(NUM_WAYS)-1:0]       data_fill_way,
  output logic                              word_write_en,
  output logic [$clog2(NUM_WAYS)-1:0]       word_write_way,
  output logic [$clog2((LINE_SIZE*8)/DATA_WIDTH)-1:0] word_idx,
  output logic [DATA_WIDTH-1:0]             word_wdata,
  output logic [(DATA_WIDTH/8)-1:0]         word_wstrb,
  output logic [$clog2(NUM_WAYS)-1:0]       data_read_way,

  // ---- adaptive_bank_controller ----
  input  logic [NUM_BANKS-1:0]              bank_enable_in,
  input  logic                              bank_ready_in,
  output logic                              bankctrl_req_valid,

  // ---- memory-side interface ----
  // (mem_rdata is not a port here: the controller only needs the mem_rvalid
  // handshake pulse to sequence the FSM. cache_top wires mem_rdata straight
  // into data_array's fill_line, bypassing the controller entirely.)
  output logic                              mem_req_valid,
  output logic [ADDR_WIDTH-1:0]             mem_addr,
  output logic                              mem_we,
  output logic [LINE_SIZE*8-1:0]            mem_wdata,
  input  logic                              mem_ready,
  input  logic                              mem_rvalid
);

  localparam int OFFSET_BITS      = $clog2(LINE_SIZE);
  localparam int SET_IN_BANK_BITS = $clog2(SETS_PER_BANK);
  localparam int BANK_BITS        = $clog2(NUM_BANKS);
  localparam int BYTE_OFFSET_BITS = $clog2(DATA_WIDTH/8);
  localparam int WORD_OFFSET_BITS = OFFSET_BITS - BYTE_OFFSET_BITS;
  localparam int WAY_BITS         = $clog2(NUM_WAYS);

  cache_pkg::ctrl_state_e state;

  logic [ADDR_WIDTH-1:0]       addr_r;
  logic                        we_r;
  logic [DATA_WIDTH-1:0]       wdata_r;
  logic [(DATA_WIDTH/8)-1:0]   wstrb_r;

  logic [WAY_BITS-1:0]         victim_way_r;
  logic [TAG_BITS-1:0]         victim_tag_r;
  logic                        victim_dirty_r;
  logic [WAY_BITS-1:0]         service_way_r;
  logic                        was_hit_r;

  // Single shared address bus: req_addr while still in IDLE (a request may
  // be accepted this very cycle), the latched addr_r afterward. Keeping one
  // mux here means every downstream slice (bank_sel/set_idx/req_tag/word_idx)
  // is correct on the same cycle a request is accepted, with no duplicated
  // slicing logic for the two cases.
  logic [ADDR_WIDTH-1:0] addr_mux;
  assign addr_mux = (state == cache_pkg::ST_IDLE) ? req_addr : addr_r;

  assign bank_sel = addr_mux[OFFSET_BITS + SET_IN_BANK_BITS +: BANK_BITS];
  assign set_idx  = addr_mux[OFFSET_BITS +: SET_IN_BANK_BITS];
  assign req_tag  = addr_mux[ADDR_WIDTH-1 -: TAG_BITS];

  logic [WORD_OFFSET_BITS-1:0] word_idx_c;
  assign word_idx_c = addr_mux[BYTE_OFFSET_BITS +: WORD_OFFSET_BITS];

  // ---------------- sequential state / registers ----------------
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state          <= cache_pkg::ST_IDLE;
      addr_r         <= {ADDR_WIDTH{1'b0}};
      we_r           <= 1'b0;
      wdata_r        <= {DATA_WIDTH{1'b0}};
      wstrb_r        <= {(DATA_WIDTH/8){1'b0}};
      victim_way_r   <= {WAY_BITS{1'b0}};
      victim_tag_r   <= {TAG_BITS{1'b0}};
      victim_dirty_r <= 1'b0;
      service_way_r  <= {WAY_BITS{1'b0}};
      was_hit_r      <= 1'b0;
    end else begin
      case (state)

        cache_pkg::ST_IDLE: begin
          if (req_valid) begin
            addr_r  <= req_addr;
            we_r    <= req_we;
            wdata_r <= req_wdata;
            wstrb_r <= req_wstrb;
            state   <= bank_enable_in[bank_sel] ? cache_pkg::ST_LOOKUP :
                                                  cache_pkg::ST_BANK_CTRL;
          end
        end

        cache_pkg::ST_BANK_CTRL: begin
          if (bank_ready_in) state <= cache_pkg::ST_LOOKUP;
        end

        cache_pkg::ST_LOOKUP: begin
          if (hit_in) begin
            was_hit_r     <= 1'b1;
            service_way_r <= hit_way_in;
            state         <= cache_pkg::ST_HIT;
          end else begin
            was_hit_r      <= 1'b0;
            victim_way_r   <= victim_way_in;
            victim_tag_r   <= tag_in[victim_way_in];
            victim_dirty_r <= dirty_in[victim_way_in];
            state          <= cache_pkg::ST_MISS;
          end
        end

        cache_pkg::ST_MISS: begin
          state <= victim_dirty_r ? cache_pkg::ST_WRITEBACK : cache_pkg::ST_REFILL;
        end

        cache_pkg::ST_WRITEBACK: begin
          if (mem_ready) state <= cache_pkg::ST_REFILL;
        end

        cache_pkg::ST_REFILL: begin
          if (mem_rvalid) begin
            service_way_r <= victim_way_r;
            state         <= cache_pkg::ST_HIT;
          end
        end

        cache_pkg::ST_HIT: begin
          state <= cache_pkg::ST_IDLE;
        end

        default: state <= cache_pkg::ST_IDLE;

      endcase
    end
  end

  // ---------------- combinational control outputs ----------------
  always @(*) begin
    req_ready  = (state == cache_pkg::ST_IDLE);
    resp_valid = 1'b0;
    resp_rdata = {DATA_WIDTH{1'b0}};
    resp_hit   = was_hit_r;

    tag_fill_en        = 1'b0;
    tag_dirty_set_en   = 1'b0;
    data_fill_en       = 1'b0;
    word_write_en      = 1'b0;
    rep_access_en      = 1'b0;
    rep_fill_en        = 1'b0;
    bankctrl_req_valid = 1'b0;

    mem_req_valid = 1'b0;
    mem_we        = 1'b0;
    mem_addr      = {ADDR_WIDTH{1'b0}};
    mem_wdata     = {(LINE_SIZE*8){1'b0}};

    data_read_way = service_way_r;
    word_idx      = word_idx_c;
    word_wdata    = wdata_r;
    word_wstrb    = wstrb_r;

    case (state)

      cache_pkg::ST_BANK_CTRL: begin
        bankctrl_req_valid = 1'b1;
      end

      cache_pkg::ST_LOOKUP: begin
        // Bank must already be ACTIVE to have reached LOOKUP; asserting
        // this simply refreshes that bank's idle counter for real use.
        bankctrl_req_valid = 1'b1;
      end

      cache_pkg::ST_WRITEBACK: begin
        mem_req_valid = 1'b1;
        mem_we        = 1'b1;
        mem_addr      = {victim_tag_r, bank_sel, set_idx, {OFFSET_BITS{1'b0}}};
        data_read_way = victim_way_r;
        mem_wdata     = line_in;
      end

      cache_pkg::ST_REFILL: begin
        mem_req_valid = 1'b1;
        mem_we        = 1'b0;
        mem_addr      = {req_tag, bank_sel, set_idx, {OFFSET_BITS{1'b0}}};
        if (mem_rvalid) begin
          tag_fill_en  = 1'b1;
          data_fill_en = 1'b1;
          rep_fill_en  = 1'b1;
        end
      end

      cache_pkg::ST_HIT: begin
        resp_valid = 1'b1;
        if (we_r) begin
          word_write_en    = 1'b1;
          tag_dirty_set_en = 1'b1;
        end else begin
          resp_rdata = line_in[word_idx_c*DATA_WIDTH +: DATA_WIDTH];
        end
        if (was_hit_r) rep_access_en = 1'b1;
      end

      default: ; // ST_IDLE, ST_MISS: defaults above hold

    endcase
  end

  assign tag_fill_way      = victim_way_r;
  assign tag_dirty_set_way = service_way_r;
  assign data_fill_way     = victim_way_r;
  assign word_write_way    = service_way_r;
  assign rep_access_way    = service_way_r;
  assign rep_fill_way      = victim_way_r;

endmodule : cache_controller