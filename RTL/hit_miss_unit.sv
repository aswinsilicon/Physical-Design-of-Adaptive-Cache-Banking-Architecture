// hit_miss_unit.sv
// Combinational tag compare across the ways of the currently-selected
// bank/set. Reports at most one hit way; a same-cycle invariant check
// flags the (should-never-happen) case of two ways matching, which would
// indicate a fill/invalidate bug elsewhere in the design.

module hit_miss_unit #(
  parameter int NUM_WAYS = 4,
  parameter int TAG_BITS = 22
)(
  input  logic [NUM_WAYS-1:0]                valid_in,
  input  logic [NUM_WAYS-1:0][TAG_BITS-1:0]  tag_in,
  input  logic [TAG_BITS-1:0]                req_tag,

  output logic                               hit,
  output logic [$clog2(NUM_WAYS)-1:0]        hit_way
);

  logic [NUM_WAYS-1:0] match;

  genvar gw;
  generate
    for (gw = 0; gw < NUM_WAYS; gw = gw + 1) begin : g_cmp
      assign match[gw] = valid_in[gw] && (tag_in[gw] == req_tag);
    end
  endgenerate

  integer w;

  always @(*) begin
    hit     = |match;
    hit_way = {$clog2(NUM_WAYS){1'b0}};
    for (w = 0; w < NUM_WAYS; w = w + 1) begin
      if (match[w]) hit_way = w[$clog2(NUM_WAYS)-1:0];
    end
  end

  // Core invariant: at most one way may hit within a bank. Simulation-only;
  // has no effect on synthesized hardware.
  // synthesis translate_off
  always @(*) begin
    if ($countones(match) > 1) begin
      $error("hit_miss_unit: multiple ways hit simultaneously, match=%b", match);
    end
  end
  // synthesis translate_on

endmodule : hit_miss_unit