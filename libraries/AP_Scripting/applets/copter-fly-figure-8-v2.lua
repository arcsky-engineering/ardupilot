-- command a Copter to fly LiDAR calibration patterns
--
-- CAUTION: This script only works for Copter
--
-- PTRN_PNUM selects the vendor calibration routine:
--   1 = Phoenix LiDAR
--       CHN1: straight line -> figure 8 (seamless transition at speed)
--       CHN2: figure 8 -> straight line (seamless transition at speed)
--   2 = Inertial Labs
--       CHN1: straight line (accel/cruise/decel) -> dwell -> figure 8
--       CHN2: inactive
--   3 = YellowScan
--       CHN1: straight line (accel/cruise/decel) -> U-turn
--       CHN2: inactive
--
-- PTRN_SHAPE selects the figure-8 curve type:
--   0 = Lemniscate of Bernoulli (smooth, continuously varying curvature)
--   1 = Two tangential circles (constant curvature per lobe, curvature
--       reversal at center crossing)
--
-- All maneuvers use position control (set_target_location) with a
-- LEAD_TIME look-ahead so the position controller has enough error to
-- command the desired speed.
--
-- At pattern completion the drone switches to Loiter and awaits pilot
-- input to continue the mission or land.

local RUN_INTERVAL_MS = 100

-- MAVLink severity constants
local MAV_SEVERITY_WARNING = 4
local MAV_SEVERITY_NOTICE  = 5
local MAV_SEVERITY_INFO    = 6

-- Flight mode numbers
local copter_auto_mode_num   = 3
local copter_guided_mode_num = 4
local copter_loiter_mode_num = 5

-- ========================================================================
-- Parameters
-- ========================================================================
local function param_get(name)
  return assert(param:get(name), ('PTRN: %s parameter error'):format(name))
end

local PARAM_PREFIX    = 'PTRN'
local PARAM_TABLE_KEY = 72
local PARAM_TABLE     = {
  { 'ENABLE',  1},   -- 1:  master enable
  { 'RADIUS', 50},   -- 2:  figure-8 lobe radius (m)
  { 'SLTIME',  6},   -- 3:  straight line cruise time (s)
  { 'SPEED',   5},   -- 4:  figure-8 / maneuver speed (m/s)
  { 'PNUM',    1},   -- 5:  vendor pattern (1=Phoenix, 2=IL, 3=YS)
  { 'SLSPD',   8},   -- 6:  straight line speed (m/s)
  { 'CHN1',   10},   -- 7:  RC channel for primary trigger
  { 'CHN2',   11},   -- 8:  RC channel for secondary trigger
  { 'SHAPE',   0},   -- 9:  figure-8 shape (0=lemniscate, 1=circles)
  { 'ATIME',  10},   -- 10: accel/decel ramp time (s)
  { 'TRIGGER', 0},   -- 11: GCS trigger (0=idle, 1=start CHN1, 2=start CHN2)
  { 'TOAUTO',  0},   -- 12: exit mode (0=loiter, 1=auto) on pattern complete
}

local function add_params(key, prefix, tbl)
  assert(param:add_table(key, prefix, #tbl),
    ('Could not add %s param table.'):format(prefix))
  for num, data in ipairs(tbl) do
    assert(param:add_param(key, num, data[1], data[2]),
      ('Could not add %s%s.'):format(prefix, data[1]))
  end
end
add_params(PARAM_TABLE_KEY, PARAM_PREFIX .. '_', PARAM_TABLE)

-- Parameter documentation. Picked up by Tools/autotest/param_metadata so these
-- appear in apm.pdef.xml/json alongside the C++ params, which is what feeds the
-- GCS parameter descriptions and Tools/xplorer/gen_param_docs.py.
-- Ranges below are enforced by read_and_clamp_params() at read time, NOT by the
-- firmware clamp table, so an out-of-range value is stored and clamped on use.

--[[
  // @Param: PTRN_ENABLE
  // @DisplayName: Pattern script enable
  // @Description: Master enable for the LiDAR calibration pattern script.
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]

--[[
  // @Param: PTRN_RADIUS
  // @DisplayName: Figure-8 lobe radius
  // @Description: Radius of each figure-8 lobe. Values outside the range are clamped by the script at read time.
  // @Units: m
  // @Range: 25 150
  // @User: Standard
--]]

--[[
  // @Param: PTRN_SLTIME
  // @DisplayName: Straight line cruise time
  // @Description: Cruise duration of the straight-line segment, excluding the acceleration and deceleration ramps. Values outside the range are clamped by the script at read time.
  // @Units: s
  // @Range: 3 20
  // @User: Standard
--]]

--[[
  // @Param: PTRN_SPEED
  // @DisplayName: Figure-8 speed
  // @Description: Target ground speed during the figure-8 and U-turn segments. Values outside the range are clamped by the script at read time.
  // @Units: m/s
  // @Range: 2 8
  // @User: Standard
--]]

--[[
  // @Param: PTRN_PNUM
  // @DisplayName: Vendor calibration pattern
  // @Description: Selects the vendor calibration routine. Phoenix runs a straight line to figure-8 on CHN1 and figure-8 to straight line on CHN2, both transitioning seamlessly at speed. Inertial Labs runs a straight line with accel/cruise/decel, then a dwell, then a figure-8 on CHN1 only. YellowScan runs a straight line then a U-turn on CHN1 only.
  // @Values: 1:Phoenix LiDAR,2:Inertial Labs,3:YellowScan
  // @Range: 1 3
  // @User: Standard
--]]

--[[
  // @Param: PTRN_SLSPD
  // @DisplayName: Straight line speed
  // @Description: Target ground speed during the straight-line segment. Values outside the range are clamped by the script at read time.
  // @Units: m/s
  // @Range: 2 12
  // @User: Standard
--]]

--[[
  // @Param: PTRN_CHN1
  // @DisplayName: Primary pattern RC trigger channel
  // @Description: RC channel that starts the primary pattern when held above 1800us for 1 second. Set to 0 to disable RC triggering and use the GCS only (PTRN_TRIGGER), which is the default because RC 1-12 carry primary flight controls. Only 5-16 are accepted; any other value is treated as 0 rather than snapped to a real channel.
  // @Range: 0 16
  // @User: Standard
--]]

--[[
  // @Param: PTRN_CHN2
  // @DisplayName: Secondary pattern RC trigger channel
  // @Description: RC channel that starts the secondary pattern when held above 1800us for 1 second. Only used when PTRN_PNUM selects a vendor routine that defines a second pattern. Set to 0 to disable RC triggering and use the GCS only (PTRN_TRIGGER). Only 5-16 are accepted; any other value is treated as 0.
  // @Range: 0 16
  // @User: Standard
--]]

--[[
  // @Param: PTRN_SHAPE
  // @DisplayName: Figure-8 shape
  // @Description: Figure-8 curve type. The lemniscate gives smooth continuously varying curvature. Two tangential circles give constant curvature per lobe with a curvature reversal at the centre crossing. Any other value is treated as 0.
  // @Values: 0:Lemniscate of Bernoulli,1:Two tangential circles
  // @User: Standard
--]]

--[[
  // @Param: PTRN_ATIME
  // @DisplayName: Accel/decel ramp time
  // @Description: Time allowed to ramp up to and down from the target speed on segments that accelerate and decelerate. Values outside the range are clamped by the script at read time.
  // @Units: s
  // @Range: 4 20
  // @User: Standard
--]]

--[[
  // @Param: PTRN_TRIGGER
  // @DisplayName: GCS pattern trigger
  // @Description: Starts a pattern from the GCS, and is the primary trigger mechanism. Set to 1 for the vendor's primary sequence or 2 for its secondary sequence, as selected by PTRN_PNUM. This does not read any RC channel, so it works with PTRN_CHN1 and PTRN_CHN2 set to 0. The script resets this to 0 once the pattern starts, or immediately if the sequence does not exist or the aircraft is too low. Requires Loiter mode.
  // @Values: 0:Idle,1:Start primary sequence,2:Start secondary sequence
  // @User: Standard
--]]

--[[
  // @Param: PTRN_TOAUTO
  // @DisplayName: Exit mode on completion
  // @Description: Flight mode entered when the pattern completes. Loiter holds position and awaits pilot input. Auto resumes the mission. Any other value is treated as 0.
  // @Values: 0:Loiter,1:Auto
  // @User: Standard
--]]

-- Parameter clamping
local function clamp(val, lo, hi) return math.max(lo, math.min(hi, val)) end

-- Trigger channel validation. 0 disables RC triggering for that pattern, which
-- is the default: RC 1-12 carry primary flight controls, and a stray >1800us on
-- a control channel would arm a pattern. Valid channels are 5-16; anything else
-- is treated as disabled rather than silently snapped to a real channel.
local function trig_chan(val)
  local c = math.floor(val or 0)
  if c >= 5 and c <= 16 then return c end
  return 0
end

local function read_and_clamp_params()
  local e  = param_get(('%s_ENABLE'):format(PARAM_PREFIX))
  local r  = clamp(param_get(('%s_RADIUS'):format(PARAM_PREFIX)),  25, 150)
  local st = clamp(param_get(('%s_SLTIME'):format(PARAM_PREFIX)),   3,  20)
  local sp = clamp(param_get(('%s_SPEED'):format(PARAM_PREFIX)),    2,   8)
  local pn = clamp(math.floor(param_get(('%s_PNUM'):format(PARAM_PREFIX))),  1, 3)
  local ss = clamp(param_get(('%s_SLSPD'):format(PARAM_PREFIX)),    2,  12)
  local c1 = trig_chan(param_get(('%s_CHN1'):format(PARAM_PREFIX)))
  local c2 = trig_chan(param_get(('%s_CHN2'):format(PARAM_PREFIX)))
  local sh = math.floor(param_get(('%s_SHAPE'):format(PARAM_PREFIX)))
  if sh ~= 0 and sh ~= 1 then sh = 0 end
  local at = clamp(param_get(('%s_ATIME'):format(PARAM_PREFIX)),    4,  20)
  local ta = math.floor(param_get(('%s_TOAUTO'):format(PARAM_PREFIX)))
  if ta ~= 0 and ta ~= 1 then ta = 0 end
  return e, r, st, sp, pn, ss, c1, c2, sh, at, ta
end

-- Initial parameter reads (re-read in init state each trigger)
local ptrn_enabled, ptrn_radius, ptrn_sltime, ptrn_speed,
      ptrn_pnum, ptrn_slspd, ptrn_chn1, ptrn_chn2,
      ptrn_shape, ptrn_atime, ptrn_toauto = read_and_clamp_params()

-- ========================================================================
-- State machine variables
-- ========================================================================
local current_state  = "idle"
local state_entered  = false
local seq            = {}      -- current stage sequence
local seq_idx        = 0       -- current index into seq

-- Pattern flags
local pattern_complete  = false
local pattern_triggered = false
local trigger_cnt       = 0
local trigger_cnt_2     = 0
local active_channel    = 0    -- 1 or 2 (which RC channel triggered)

-- Heading captured at init
local yaw_cos = 0
local yaw_sin = 0

-- Speed control
local spd_adj   = 0
local spd_steps = 0

-- Timing
local accel_time = 10000.0  -- ms, set from PTRN_ATIME in init
local dwell_time = 1000     -- ms for zero-velocity dwell
local LEAD_TIME  = 1.0      -- seconds of position look-ahead

-- Straight line position tracking
local sl_ref_pos  = Location()
local sl_distance = 0
local hold_pos    = Location()

-- Lemniscate state
local startPos       = Location()  -- figure-8 center / tangent point
local rotation       = 0           -- lemniscate rotation angle
local t              = 0.0         -- lemniscate parameter
local b              = 1.1         -- lemniscate shape factor
local lem_complete_t = 8.05        -- set dynamically in fig8_init

-- Tangential circles state
local circ_lobe  = 1    -- current lobe (1=CCW/left, 2=CW/right)
local circ_theta = 0.0  -- angle within current lobe

-- U-turn state
local uturn_theta = 0.0
local uturn_ref   = Location()

-- ========================================================================
-- Helper functions
-- ========================================================================

-- Lemniscate parametric speed |v(t)|
local function lem_speed(t_val, a)
  local st = math.sin(t_val)
  local s2 = st * st
  local u2 = (1.0 + s2) * (1.0 + s2)
  local dx = -a * st * (3.0 - s2) / u2
  local dy =  a * b * (1.0 - 3.0 * s2) / u2
  return math.sqrt(dx * dx + dy * dy)
end

-- Lemniscate position at parameter t_val, offset from center_pos
-- Uses the global 'rotation' angle for orientation
local function lem_position(t_val, a, center_pos)
  local st = math.sin(t_val)
  local ct = math.cos(t_val)
  local u  = 1.0 + st * st
  local Xo = a * ct / u
  local Yo = a * b * st * ct / u
  local oe = Xo * math.cos(rotation) + Yo * math.sin(rotation)
  local on = Yo * math.cos(rotation) - Xo * math.sin(rotation)
  local loc = center_pos:copy()
  loc:offset(on, oe)
  return loc
end

-- Circle position at angle theta, radius r, lobe direction
-- lobe=1: CCW/left turn,  lobe=2: CW/right turn
-- Uses global yaw_cos/yaw_sin for heading orientation
local function circ_position(theta, r, lobe, ref_pos)
  local x_local = r * math.sin(theta)
  local y_local
  if lobe == 1 then
    y_local = r * (1.0 - math.cos(theta))
  else
    y_local = -(r * (1.0 - math.cos(theta)))
  end
  local n = x_local * yaw_cos + y_local * yaw_sin
  local e = x_local * yaw_sin - y_local * yaw_cos
  local loc = ref_pos:copy()
  loc:offset(n, e)
  return loc
end

-- Figure-8 circle target with lead overflow handling
-- When the lead-adjusted theta exceeds 2*pi on the current lobe, the
-- target spills into the next lobe (lobe 1 -> 2) or projects straight
-- forward from the center (past lobe 2).  This prevents the lead from
-- being clamped to zero at lobe boundaries, which would cause the
-- position controller to decelerate.
local function fig8_circ_target(theta_d, r, lobe, ref_pos)
  if theta_d <= 2.0 * math.pi then
    return circ_position(theta_d, r, lobe, ref_pos)
  elseif lobe == 1 then
    -- Lead overflows from lobe 1 into lobe 2
    return circ_position(theta_d - 2.0 * math.pi, r, 2, ref_pos)
  else
    -- Lead overflows past lobe 2: project straight forward from center
    local overflow_dist = (theta_d - 2.0 * math.pi) * r
    local loc = ref_pos:copy()
    loc:offset(overflow_dist * yaw_cos, overflow_dist * yaw_sin)
    return loc
  end
end

-- ========================================================================
-- Vendor sequences
--
-- Each vendor (PNUM) defines up to two sequences (CHN1, CHN2).
-- nil means that channel is inactive for that vendor.
-- The state names are reusable building blocks assembled per vendor.
-- ========================================================================
local vendor_sequences = {
  [1] = {  -- Phoenix LiDAR
    [1] = {"init", "sl_accel", "sl_cruise",
           "fig8_init", "fig8_run", "fig8_decel", "complete"},
    [2] = {"init", "fig8_init", "fig8_run",
           "sl_cruise", "sl_decel", "complete"},
  },
  [2] = {  -- Inertial Labs
    [1] = {"init", "sl_accel", "sl_cruise", "sl_decel", "dwell",
           "fig8_init", "fig8_run", "fig8_decel", "complete"},
    [2] = nil,
  },
  [3] = {  -- YellowScan
    [1] = {"init", "sl_accel", "sl_cruise", "sl_decel",
           "uturn", "sl_decel", "complete"},
    [2] = nil,
  },
}

-- Vendor display names for GCS messages
local vendor_names = {
  [1] = "Phoenix",
  [2] = "InertialLabs",
  [3] = "YellowScan",
}

-- ========================================================================
-- State machine utilities
-- ========================================================================
local function advance_state()
  seq_idx = seq_idx + 1
  if seq_idx <= #seq then
    current_state = seq[seq_idx]
  else
    current_state = "complete"
  end
  state_entered = false
  gcs:send_text(MAV_SEVERITY_NOTICE, ('Pattern: -> %s'):format(current_state))
end

local function check_altitude()
  local pos = ahrs:get_position()
  if pos then
    local alt_vec = pos:copy():get_vector_from_origin_NEU()
    if alt_vec then
      local alt = alt_vec:z() * 0.01
      if alt and alt > 10.0 then
        return true
      end
    end
  end
  return false
end

-- ========================================================================
-- is_active() — check RC channels and manage pattern trigger state
-- ========================================================================
local function is_active()
  -- Re-read trigger-relevant params every cycle so GCS changes take effect
  ptrn_enabled = param_get(('%s_ENABLE'):format(PARAM_PREFIX))
  ptrn_pnum    = clamp(math.floor(param_get(('%s_PNUM'):format(PARAM_PREFIX))),  1, 3)
  ptrn_chn1    = trig_chan(param_get(('%s_CHN1'):format(PARAM_PREFIX)))
  ptrn_chn2    = trig_chan(param_get(('%s_CHN2'):format(PARAM_PREFIX)))

  if ptrn_enabled <= 0 then return false end

  if pattern_triggered then
    if pattern_complete then return false end
    if current_state ~= "idle" and current_state ~= "init"
       and vehicle:get_mode() ~= copter_guided_mode_num then
      pattern_triggered = false
      trigger_cnt = 0
      trigger_cnt_2 = 0
      return false
    end
    return true
  end

  -- Only allow new activations when in LOITER mode.  This prevents the
  -- pattern from taking over during AUTO missions or other active flight
  -- modes.  Mid-pattern mode changes are handled by the block above.
  if vehicle:get_mode() ~= copter_loiter_mode_num then
    trigger_cnt = 0
    trigger_cnt_2 = 0
    return false
  end

  -- Not triggered yet -- check RC channels for the active vendor
  local vendor = vendor_sequences[ptrn_pnum]
  if not vendor then return false end

  -- CHN1 check (skipped entirely when PTRN_CHN1 is 0 / RC triggering disabled)
  local chn1_seq = vendor[1]
  if chn1_seq and ptrn_chn1 > 0 then
    local pwm1 = rc:get_pwm(ptrn_chn1)
    if pwm1 and pwm1 > 1800 then
      if trigger_cnt < 10 then
        trigger_cnt = trigger_cnt + 1
      else
        if check_altitude() then
          active_channel = 1
          pattern_triggered = true
          pattern_complete = false
          seq = chn1_seq
          seq_idx = 0
          current_state = "idle"
          state_entered = false
        end
      end
    else
      trigger_cnt = 0
    end
  end

  -- CHN2 check
  if not pattern_triggered then
    local chn2_seq = vendor[2]
    if chn2_seq and ptrn_chn2 > 0 then
      local pwm2 = rc:get_pwm(ptrn_chn2)
      if pwm2 and pwm2 > 1800 then
        if trigger_cnt_2 < 10 then
          trigger_cnt_2 = trigger_cnt_2 + 1
        else
          if check_altitude() then
            active_channel = 2
            pattern_triggered = true
            pattern_complete = false
            seq = chn2_seq
            seq_idx = 0
            current_state = "idle"
            state_entered = false
          end
        end
      else
        trigger_cnt_2 = 0
      end
    end
  end

  -- GCS trigger via PTRN_TRIGGER parameter (1=CHN1, 2=CHN2)
  if not pattern_triggered then
    local trig = math.floor(param_get(('%s_TRIGGER'):format(PARAM_PREFIX)))
    if trig >= 1 and trig <= 2 then
      local trig_seq = vendor[trig]
      if trig_seq and check_altitude() then
        active_channel = trig
        pattern_triggered = true
        pattern_complete = false
        seq = trig_seq
        seq_idx = 0
        current_state = "idle"
        state_entered = false
        param:set_and_save(('%s_TRIGGER'):format(PARAM_PREFIX), 0)
        gcs:send_text(MAV_SEVERITY_NOTICE,
          ('Pattern: GCS trigger CHN%d'):format(trig))
      else
        -- Reset the trigger, but say why: the GCS button is the primary trigger
        -- mechanism, so a silent reset looks like the button did nothing.
        param:set_and_save(('%s_TRIGGER'):format(PARAM_PREFIX), 0)
        if not trig_seq then
          gcs:send_text(MAV_SEVERITY_WARNING,
            ('Pattern: no sequence %d for PTRN_PNUM %d'):format(trig, ptrn_pnum))
        else
          gcs:send_text(MAV_SEVERITY_WARNING,
            'Pattern: too low to start (need >10m above origin)')
        end
      end
    end
  end

  return false
end

-- ========================================================================
-- do_pattern() — main pattern execution state machine
-- ========================================================================
function do_pattern()
  if not is_active() then
    gcs:send_text(MAV_SEVERITY_NOTICE, 'Pattern: Exiting pattern')
    return standby, RUN_INTERVAL_MS
  end

  -- ------------------------------------------------------------------
  -- IDLE: advance to first real state
  -- ------------------------------------------------------------------
  if current_state == "idle" then
    advance_state()

  -- ------------------------------------------------------------------
  -- INIT: switch to guided, re-read parameters, capture heading
  -- ------------------------------------------------------------------
  elseif current_state == "init" then
    vehicle:set_mode(copter_guided_mode_num)

    ptrn_enabled, ptrn_radius, ptrn_sltime, ptrn_speed,
      ptrn_pnum, ptrn_slspd, ptrn_chn1, ptrn_chn2,
      ptrn_shape, ptrn_atime, ptrn_toauto = read_and_clamp_params()

    accel_time = ptrn_atime * 1000.0  -- convert seconds to ms

    local vname = vendor_names[ptrn_pnum] or tostring(ptrn_pnum)
    gcs:send_text(MAV_SEVERITY_NOTICE,
      ('PTRN: %s SLSPD=%.1f SLTIME=%.0f SPD=%.1f R=%.0f SH=%d'):format(
        vname, ptrn_slspd, ptrn_sltime, ptrn_speed, ptrn_radius, ptrn_shape))

    sl_distance = 0
    local yaw_rad = ahrs:get_yaw()
    yaw_cos = math.cos(yaw_rad)
    yaw_sin = math.sin(yaw_rad)
    advance_state()

  -- ------------------------------------------------------------------
  -- SL_ACCEL: straight line acceleration from 0 to ptrn_slspd
  -- ------------------------------------------------------------------
  elseif current_state == "sl_accel" then
    if not state_entered then
      state_entered = true
      spd_steps = accel_time / RUN_INTERVAL_MS
      spd_adj = 0.0
      sl_ref_pos = ahrs:get_position():copy()
      sl_distance = 0
    end

    sl_distance = sl_distance + ptrn_slspd * spd_adj * RUN_INTERVAL_MS * 0.001
    local lead = ptrn_slspd * spd_adj * LEAD_TIME
    local d = sl_distance + lead
    local target = sl_ref_pos:copy()
    target:offset(d * yaw_cos, d * yaw_sin)
    vehicle:set_target_location(target)

    if spd_steps >= 0 then
      spd_steps = spd_steps - 1
      spd_adj = 1.0 - (spd_steps / (accel_time / RUN_INTERVAL_MS))
    else
      advance_state()
    end

  -- ------------------------------------------------------------------
  -- SL_CRUISE: straight line at constant ptrn_slspd
  -- ------------------------------------------------------------------
  elseif current_state == "sl_cruise" then
    if not state_entered then
      state_entered = true
      spd_steps = (ptrn_sltime * 1000) / RUN_INTERVAL_MS
    end

    sl_distance = sl_distance + ptrn_slspd * RUN_INTERVAL_MS * 0.001
    local lead = ptrn_slspd * LEAD_TIME
    local d = sl_distance + lead
    local target = sl_ref_pos:copy()
    target:offset(d * yaw_cos, d * yaw_sin)
    vehicle:set_target_location(target)

    if spd_steps >= 0 then
      spd_steps = spd_steps - 1
    else
      advance_state()
    end

  -- ------------------------------------------------------------------
  -- SL_DECEL: straight line deceleration from ptrn_slspd to 0
  -- ------------------------------------------------------------------
  elseif current_state == "sl_decel" then
    if not state_entered then
      state_entered = true
      spd_steps = accel_time / RUN_INTERVAL_MS
      spd_adj = 1.0
    end

    sl_distance = sl_distance + ptrn_slspd * spd_adj * RUN_INTERVAL_MS * 0.001
    local lead = ptrn_slspd * spd_adj * LEAD_TIME
    local d = sl_distance + lead
    local target = sl_ref_pos:copy()
    target:offset(d * yaw_cos, d * yaw_sin)
    vehicle:set_target_location(target)

    if spd_steps >= 0 then
      spd_steps = spd_steps - 1
      spd_adj = spd_steps / (accel_time / RUN_INTERVAL_MS)
    else
      advance_state()
    end

  -- ------------------------------------------------------------------
  -- DWELL: hold position for dwell_time
  -- ------------------------------------------------------------------
  elseif current_state == "dwell" then
    if not state_entered then
      state_entered = true
      spd_steps = dwell_time / RUN_INTERVAL_MS
      hold_pos = ahrs:get_position():copy()
    end

    vehicle:set_target_location(hold_pos)

    if spd_steps >= 0 then
      spd_steps = spd_steps - 1
    else
      advance_state()
    end

  -- ------------------------------------------------------------------
  -- FIG8_INIT: set up figure-8 parameters
  --
  -- Determines entry mode by checking the previous state in the
  -- sequence:
  --   prev = "sl_cruise" -> seamless entry at speed (no decel/dwell)
  --   anything else      -> standstill entry with acceleration ramp
  --
  -- For seamless exit, checks the next state after fig8_run:
  --   next = "sl_cruise" -> exact one-loop completion for clean handoff
  -- ------------------------------------------------------------------
  elseif current_state == "fig8_init" then
    local prev_st = (seq_idx >= 2) and seq[seq_idx - 1] or ""
    local seamless_entry = (prev_st == "sl_cruise")

    -- Look ahead to determine exit mode for lemniscate completion angle
    local fig8_run_idx = seq_idx + 1  -- fig8_run follows fig8_init
    local after_run = (fig8_run_idx < #seq) and seq[fig8_run_idx + 1] or ""
    local seamless_exit = (after_run == "sl_cruise")

    local a = ptrn_radius * 2.0

    if ptrn_shape == 0 then
      -- LEMNISCATE setup
      if seamless_entry then
        rotation = math.atan(-(yaw_cos + b * yaw_sin),
                              yaw_sin - b * yaw_cos)
        t = -math.pi / 2.0
        spd_adj = 1.0
        spd_steps = -1
        lem_complete_t = 3.0 * math.pi / 2.0

        startPos = sl_ref_pos:copy()
        startPos:offset(sl_distance * yaw_cos, sl_distance * yaw_sin)

        local sl = lem_speed(t, a)
        local t_bridge = t
        if sl > 0.001 then
          t_bridge = t + (ptrn_speed * LEAD_TIME) / sl
        end
        vehicle:set_target_location(lem_position(t_bridge, a, startPos))
      else
        rotation = ahrs:get_yaw() + math.pi / 2.0
        t = math.pi / 2.0
        startPos = ahrs:get_position():copy()
        spd_adj = 0.0
        spd_steps = accel_time / RUN_INTERVAL_MS
        if seamless_exit then
          lem_complete_t = math.pi / 2.0 + 2.0 * math.pi
        else
          lem_complete_t = 8.05
        end
        vehicle:set_target_location(startPos)
      end
    else
      -- TANGENTIAL CIRCLES setup
      circ_lobe = 1
      circ_theta = 0.0

      if seamless_entry then
        spd_adj = 1.0
        spd_steps = -1
        startPos = sl_ref_pos:copy()
        startPos:offset(sl_distance * yaw_cos, sl_distance * yaw_sin)

        local lead_theta = ptrn_speed * LEAD_TIME / ptrn_radius
        vehicle:set_target_location(
          circ_position(lead_theta, ptrn_radius, 1, startPos))
      else
        spd_adj = 0.0
        spd_steps = accel_time / RUN_INTERVAL_MS
        startPos = ahrs:get_position():copy()
        vehicle:set_target_location(startPos)
      end
    end

    advance_state()

  -- ------------------------------------------------------------------
  -- FIG8_RUN: trace figure-8 at constant speed
  --
  -- SHAPE=0 (lemniscate): variable-rate t advancement for constant
  --   ground speed, plus LEAD_TIME look-ahead on the curve.
  -- SHAPE=1 (tangential circles): constant angular rate omega = v/r,
  --   two full circles (lobe 1 CCW, lobe 2 CW), plus lead.
  -- ------------------------------------------------------------------
  elseif current_state == "fig8_run" then

    if ptrn_shape == 0 then
      -- === LEMNISCATE ===
      local a = ptrn_radius * 2.0
      local speed_local = lem_speed(t, a)
      local desired_speed = ptrn_speed * spd_adj

      if speed_local > 0.001 then
        t = t + (desired_speed / speed_local) * RUN_INTERVAL_MS * 0.001
      end

      local t_display = t
      local sl = lem_speed(t, a)
      if sl > 0.001 then
        t_display = t + (desired_speed * LEAD_TIME) / sl
      end

      vehicle:set_target_location(lem_position(t_display, a, startPos))

      if spd_steps >= 0 then
        spd_steps = spd_steps - 1
        spd_adj = 1.0 - (spd_steps / (accel_time / RUN_INTERVAL_MS))
      else
        if t >= lem_complete_t then
          local next_st = (seq_idx < #seq) and seq[seq_idx + 1] or ""
          if next_st == "sl_cruise" then
            local current_yaw = ahrs:get_yaw()
            yaw_cos = math.cos(current_yaw)
            yaw_sin = math.sin(current_yaw)
            sl_ref_pos = lem_position(lem_complete_t, a, startPos)
            sl_distance = 0
          end
          advance_state()
        end
      end

    else
      -- === TANGENTIAL CIRCLES ===
      local desired_speed = ptrn_speed * spd_adj
      local omega = desired_speed / ptrn_radius
      circ_theta = circ_theta + omega * RUN_INTERVAL_MS * 0.001

      local lead_theta = desired_speed * LEAD_TIME / ptrn_radius
      local theta_d = circ_theta + lead_theta

      vehicle:set_target_location(
        fig8_circ_target(theta_d, ptrn_radius, circ_lobe, startPos))

      if spd_steps >= 0 then
        spd_steps = spd_steps - 1
        spd_adj = 1.0 - (spd_steps / (accel_time / RUN_INTERVAL_MS))
      else
        if circ_theta >= 2.0 * math.pi then
          if circ_lobe == 1 then
            circ_lobe = 2
            circ_theta = circ_theta - 2.0 * math.pi
          else
            -- Both lobes complete
            local next_st = (seq_idx < #seq) and seq[seq_idx + 1] or ""
            if next_st == "sl_cruise" then
              local current_yaw = ahrs:get_yaw()
              yaw_cos = math.cos(current_yaw)
              yaw_sin = math.sin(current_yaw)
              sl_ref_pos = startPos:copy()
              sl_distance = 0
            end
            advance_state()
          end
        end
      end
    end

  -- ------------------------------------------------------------------
  -- FIG8_DECEL: decelerate while continuing to trace the curve
  -- ------------------------------------------------------------------
  elseif current_state == "fig8_decel" then
    if not state_entered then
      state_entered = true
      spd_steps = accel_time / RUN_INTERVAL_MS
      spd_adj = 1.0
    end

    if ptrn_shape == 0 then
      -- === LEMNISCATE decel ===
      local a = ptrn_radius * 2.0
      local speed_local = lem_speed(t, a)
      local desired_speed = ptrn_speed * spd_adj

      if speed_local > 0.001 then
        t = t + (desired_speed / speed_local) * RUN_INTERVAL_MS * 0.001
      end

      local t_display = t
      local sl = lem_speed(t, a)
      if sl > 0.001 then
        t_display = t + (desired_speed * LEAD_TIME) / sl
      end

      vehicle:set_target_location(lem_position(t_display, a, startPos))
    else
      -- === TANGENTIAL CIRCLES decel ===
      local desired_speed = ptrn_speed * spd_adj
      local omega = desired_speed / ptrn_radius
      circ_theta = circ_theta + omega * RUN_INTERVAL_MS * 0.001

      local lead_theta = desired_speed * LEAD_TIME / ptrn_radius
      local theta_d = circ_theta + lead_theta

      vehicle:set_target_location(
        fig8_circ_target(theta_d, ptrn_radius, circ_lobe, startPos))
    end

    if spd_steps >= 0 then
      spd_steps = spd_steps - 1
      spd_adj = spd_steps / (accel_time / RUN_INTERVAL_MS)
    else
      advance_state()
    end

  -- ------------------------------------------------------------------
  -- UTURN: half-circle (180 deg) left turn with acceleration ramp
  --
  -- The turn uses position control on a semicircle.  An acceleration
  -- ramp brings the drone from zero to ptrn_speed.  The reference
  -- point is the computed straight-line endpoint (not GPS) so the
  -- circle is tangent to the outgoing leg by construction.  Lead
  -- overflow past 180 deg projects straight in the reversed heading
  -- (same technique as fig8_circ_target) so the position controller
  -- is pulled through the full turn.  On completion the heading is
  -- reversed mathematically for an exact 180 deg reversal.
  -- ------------------------------------------------------------------
  elseif current_state == "uturn" then
    if not state_entered then
      state_entered = true
      uturn_theta = 0.0
      -- Use the computed straight-line endpoint, not GPS position
      uturn_ref = sl_ref_pos:copy()
      uturn_ref:offset(sl_distance * yaw_cos, sl_distance * yaw_sin)
      spd_steps = accel_time / RUN_INTERVAL_MS
      spd_adj = 0.0
    end

    local desired_speed = ptrn_speed * spd_adj
    local omega = desired_speed / ptrn_radius
    uturn_theta = uturn_theta + omega * RUN_INTERVAL_MS * 0.001

    local lead_theta = desired_speed * LEAD_TIME / ptrn_radius
    local theta_d = uturn_theta + lead_theta          -- no pi clamp

    -- U-turn is always lobe 1 (CCW / left turn)
    if theta_d <= math.pi then
      vehicle:set_target_location(
        circ_position(theta_d, ptrn_radius, 1, uturn_ref))
    else
      -- Lead overflows past 180 deg: project straight in reversed heading
      local end_pos = circ_position(math.pi, ptrn_radius, 1, uturn_ref)
      local overflow = (theta_d - math.pi) * ptrn_radius
      end_pos:offset(-overflow * yaw_cos, -overflow * yaw_sin)
      vehicle:set_target_location(end_pos)
    end

    if spd_steps >= 0 then
      spd_steps = spd_steps - 1
      spd_adj = 1.0 - (spd_steps / (accel_time / RUN_INTERVAL_MS))
    else
      if uturn_theta >= math.pi then
        -- Compute U-turn endpoint before reversing heading
        local end_pos = circ_position(math.pi, ptrn_radius, 1, uturn_ref)
        -- Reverse heading mathematically (exact 180 deg)
        yaw_cos = -yaw_cos
        yaw_sin = -yaw_sin
        -- Set up SL reference from geometric endpoint
        sl_ref_pos = end_pos
        sl_distance = 0
        gcs:send_text(MAV_SEVERITY_NOTICE, 'Pattern: U-turn complete')
        advance_state()
      end
    end

  -- ------------------------------------------------------------------
  -- COMPLETE: switch to loiter mode and mark pattern done
  -- ------------------------------------------------------------------
  elseif current_state == "complete" then
    param:set_and_save(('%s_TRIGGER'):format(PARAM_PREFIX), 0)
    if ptrn_toauto == 1 then
      vehicle:set_mode(copter_auto_mode_num)
      gcs:send_text(MAV_SEVERITY_NOTICE, 'Pattern Complete - Auto')
    else
      vehicle:set_mode(copter_loiter_mode_num)
      gcs:send_text(MAV_SEVERITY_NOTICE, 'Pattern Complete - Loiter')
    end
    pattern_complete = true
    pattern_triggered = false
  end

  return do_pattern, RUN_INTERVAL_MS
end

-- ========================================================================
-- standby() — wait for arm and pattern trigger
-- ========================================================================
function standby()
  if not arming:is_armed() then return standby, RUN_INTERVAL_MS end

  if is_active() then
    local vname = vendor_names[ptrn_pnum] or tostring(ptrn_pnum)
    gcs:send_text(MAV_SEVERITY_NOTICE,
      ('Pattern: %s CHN%d'):format(vname, active_channel))
    return do_pattern, RUN_INTERVAL_MS
  end

  return standby, RUN_INTERVAL_MS
end

-- ========================================================================
-- Script initialization
-- ========================================================================
gcs:send_text(MAV_SEVERITY_INFO, 'Pattern: Script ready (multi-vendor)')

local guid_settings = param_get('GUID_OPTIONS')
if not (guid_settings == 0) then
  guid_settings = 0
  param:set_and_save('GUID_OPTIONS', guid_settings)
end

return standby, RUN_INTERVAL_MS
