// cache_pkg.sv
// Shared types and derived-parameter helper functions for the cache subsystem.
// Keeping the address-split/width arithmetic in one place (rather than
// re-deriving it per module) is what guarantees every module agrees on the
// same bit layout for a given parameter set.

package cache_pkg;

  // Plain packed constants (not an enum) for REPLACEMENT_POLICY: this value
  // crosses a module *parameter* boundary (replacement_unit's POLICY
  // parameter), and an enum-typed parameter/port was found to fail to
  // elaborate on this project's Icarus 10.2 build (confirmed by direct
  // test) even though enum types used purely as internal registers elaborate
  // fine on both simulators. A plain 2-bit vector sidesteps the issue and
  // is exactly as readable at call sites (`cache_pkg::POLICY_LRU`).
  localparam logic [1:0] POLICY_LRU  = 2'd0;
  localparam logic [1:0] POLICY_FIFO = 2'd1;

  typedef enum logic [2:0] {
    ST_IDLE,
    ST_BANK_CTRL,
    ST_LOOKUP,
    ST_HIT,
    ST_MISS,
    ST_WRITEBACK,
    ST_REFILL
  } ctrl_state_e;

  // Bank power states. Encoded explicitly (rather than left to the tool) so
  // the encoding is stable across modules that exchange bank_state as a
  // plain packed vector at module boundaries.
  localparam logic [1:0] BANK_ACTIVE  = 2'd0;
  localparam logic [1:0] BANK_STANDBY = 2'd1;
  localparam logic [1:0] BANK_SLEEP   = 2'd2;

  function automatic int num_sets_f(int cache_size, int line_size, int num_ways);
    return cache_size / (line_size * num_ways);
  endfunction

  function automatic int offset_bits_f(int line_size);
    return $clog2(line_size);
  endfunction

  function automatic int set_bits_f(int cache_size, int line_size, int num_ways);
    return $clog2(num_sets_f(cache_size, line_size, num_ways));
  endfunction

  function automatic int bank_bits_f(int num_banks);
    return (num_banks > 1) ? $clog2(num_banks) : 1;
  endfunction

  function automatic int set_in_bank_bits_f(int cache_size, int line_size, int num_ways, int num_banks);
    return set_bits_f(cache_size, line_size, num_ways) - $clog2(num_banks);
  endfunction

  function automatic int tag_bits_f(int addr_width, int cache_size, int line_size, int num_ways);
    return addr_width - set_bits_f(cache_size, line_size, num_ways) - offset_bits_f(line_size);
  endfunction

  function automatic int byte_offset_bits_f(int data_width);
    return $clog2(data_width / 8);
  endfunction

  function automatic int word_offset_bits_f(int line_size, int data_width);
    return offset_bits_f(line_size) - byte_offset_bits_f(data_width);
  endfunction

  function automatic int words_per_line_f(int line_size, int data_width);
    return (line_size * 8) / data_width;
  endfunction

  function automatic int way_bits_f(int num_ways);
    return (num_ways > 1) ? $clog2(num_ways) : 1;
  endfunction

  function automatic bit is_pow2_f(int x);
    return (x > 0) && ((x & (x - 1)) == 0);
  endfunction

endpackage : cache_pkg