// ================================================================
// MODULAR LIGHTSABER — OpenSCAD Model
//
// ── v0.9 (archived) ─────────────────────────────────────────────
//   Handle: 44mm OD, hull-based external thread both ends
//   5 tapered blade segments with snap-detent spigots
//   Flat endcaps (open + closed), threaded onto handle
//   Full-circumference retention ring on Seg 1
//
// ── v2 (current) ────────────────────────────────────────────────
//   Uniform blade segments: 20mm OD / 12mm ID / 4mm wall
//   Threaded segment joints (no bulkheads), all segments identical
//   V2.1: 4 segments (down from 5) — 4:1→3.2:1 blade:hilt ratio, wobble eliminated
//   Modular LED channel inserts (slide inside 12mm bore)
//   Handle: 32mm OD, 180mm, thread blade-end, bayonet pommel-end
//   Open endcap: threads onto handle, 20mm through-hole, LED ring pocket
//   Closed endcap: bayonet socket receiver for pommel tabs
//   Double-blade coupler: 180mm, bayonet both ends (+ half-coupler version)
//
// Printer: Bambu A1 Mini (180x180mm bed, ~170mm working height)
// Wall: 2.0mm constant
// Material: ASA recommended (impact resistance), PETG also viable
//
// USAGE:
//   Scroll to V2 RENDER section at bottom
//   Uncomment ONE component, then F5 preview / F6 render / Export STL
// ================================================================

$fn = 256;  // Smoothness - drop to 32/64 for faster preview, 128 for final STL

// ── GLOBAL PARAMETERS ──────────────────────────────────────────
BODY_H        = 150;   // mm - visible body height per blade segment
SPIGOT_H      = 20;    // mm - internal spigot height (blade joints)
SPIGOT_WALL   = 1.5;   // mm - spigot wall thickness
INTERFERENCE  = 0.25;  // mm - press fit interference per side
WALL          = 2.0;   // mm - standard wall thickness
TIP_R         = 9.0;   // mm - hemisphere tip radius (= Seg 5 top OD / 2, unchanged)

// ── HANDLE PARAMETERS ──────────────────────────────────────────
HANDLE_OD     = 44.0;   // matches old Seg1 OD (v0.9 resize)
HANDLE_ID     = 40.0;   // matches old Seg1 ID (v0.9 resize)
HANDLE_H      = 150.0;

// ── BAYONET PARAMETERS ─────────────────────────────────────────
BAYO_TABS     = 4;     // number of tabs
BAYO_TAB_W    = 6.0;   // tab width (arc length at OD)
BAYO_TAB_H    = 3.0;   // tab radial height (how far it sticks out)
BAYO_TAB_T    = 4.0;   // tab thickness (axial length)
BAYO_SLOT_CLR = 0.3;   // clearance on slot dimensions (slot is bigger than tab)
BAYO_TWIST    = 45;    // degrees of rotation to lock (1/8 turn - gentle)

// ── SEG1 RETENTION RING PARAMETERS ─────────────────────────────
// Full-circumference ring at Seg1 base replaces the 4 bayonet tabs.
// Seats against end_cap_open inner shelf to retain blade assembly.
// OD must be > endcap through-hole (HANDLE_ID=40mm) and < endcap bore (44.7mm).
SEG1_RING_OD  = 42.0;  // mm - ring outer diameter
SEG1_RING_H   = 4.0;   // mm - ring axial height

// ── SPIGOT SNAP DETENT PARAMETERS ──────────────────────────────
SNAP_BUMP_H   = 0.4;   // mm - radial bump height (outward from spigot tab)
SNAP_BUMP_L   = 1.5;   // mm - axial length of bump
SNAP_DEPTH    = 6;     // mm - depth into upper bore where snap engages
                       //      (= distance from upper bore bottom to bump center
                       //       when fully seated. Must be < SPIGOT_H.)
SNAP_CHAMFER  = 0.3;   // mm - chamfer top and bottom of bump for entry/exit
SNAP_GROOVE_CLR = 0.1; // mm - extra clearance on groove dimensions

// ── END CAP PARAMETERS ─────────────────────────────────────────
ENDCAP_H      = 30.0;

// ── THREAD PARAMETERS ──────────────────────────────────────────
// Hull-based helix: each turn is built from THREAD_STEPS hulled slab pairs.
// THREAD_H controls axial "fatness" independently of THREAD_PITCH.
// Chamfer = THREAD_H * 0.15 on top/bottom edges → gentle overhang, no supports.
THREAD_PITCH  = 8.0;   // mm — center-to-center spacing between turns
THREAD_DEPTH  = 2.0;   // mm — radial protrusion of ridge
THREAD_H      = 4.0;   // mm — axial height of each ridge (fatness); must be < PITCH
THREAD_TURNS  = 3;     // turns per handle end (3 × 8mm = 24mm zone)
THREAD_CLR    = 0.55;  // mm — radial clearance between mating surfaces
THREAD_STEPS  = 32;    // facets per turn (32=fast preview, 64=final render)

// ================================================================
// SEGMENT DIMENSIONS REFERENCE (v0.9 — 5 segs, each shifted down one slot from v1.5)
// Handle: OD 44.00mm / ID 40.00mm  (= old Seg1 dimensions)
// Seg 1: OD 39.67 -> 35.33mm  |  ID 35.67 -> 31.33mm  |  Spigot OD: 31.33mm  |  notch_clr=1.5
// Seg 2: OD 35.33 -> 31.00mm  |  ID 31.33 -> 27.00mm  |  Spigot OD: 27.00mm  |  notch_clr=1.7
// Seg 3: OD 31.00 -> 26.67mm  |  ID 27.00 -> 22.67mm  |  Spigot OD: 22.67mm  |  notch_clr=2.0
// Seg 4: OD 26.67 -> 22.33mm  |  ID 22.67 -> 18.33mm  |  Spigot OD: 18.33mm  |  notch_clr=2.3
// Seg 5: OD 22.33 -> 18.00mm  |  ID 18.33 -> 14.00mm  |  + integrated tip     |  TIP_R=9mm
// ================================================================


// ================================================================
// MODULE: bayonet_tab
//   Creates a single bayonet tab on a cylinder of given OD
//   Tab extends radially outward from the surface
// ================================================================
module bayonet_tab(host_od, tab_w=BAYO_TAB_W, tab_h=BAYO_TAB_H, tab_t=BAYO_TAB_T) {
    tc = 1.0;  // leading-face chamfer radius (mm)
    // Hull of 4 spheres at inset leading-face corners + flat trailing-face slab.
    //
    // Leading face (z=0): enters V-guide channel first when inserting handle.
    //   4 spheres placed at (tc inset from each edge) create smooth entry chamfer
    //   on all three exposed leading corners (Y-near, Y-far, X-outer tip).
    //   Chamfer guides the tab into the narrowing V-channel without catching.
    //
    // Trailing face (z=tab_t): locks against groove lock wall — stays flat/square
    //   for clean angular contact and a positive retention stop.
    translate([host_od/2.1, -tab_w/2, 0])
        hull() {
            // Trailing face — full rectangle, sharp corners
            translate([0, 0, tab_t - 0.001])
                cube([tab_h, tab_w, 0.001]);
            // Leading face — 4 spheres at inset corners; hull creates chamfer between them
            for (xi = [tc, tab_h - tc], yi = [tc, tab_w - tc])
                translate([xi, yi, tc]) sphere(r=tc, $fn=8);
        }
}

// ================================================================
// MODULE: bayonet_tabs_ring
//   Places N bayonet tabs around a cylinder
// ================================================================
module bayonet_tabs_ring(host_od, count=BAYO_TABS) {
    for (i = [0:count-1]) {
        rotate([0, 0, i * (360/count)])
            bayonet_tab(host_od);
    }
}

// ================================================================
// MODULE: bayonet_slot
//   Creates an L-shaped slot for bayonet locking
//   - Axial entry portion (open at top, runs down)
//   - Circumferential locking portion (runs sideways with twist)
//   Cuts THROUGH the wall (radial extrusion)
// ================================================================
module bayonet_slot(host_od, wall, tab_w=BAYO_TAB_W, tab_t=BAYO_TAB_T, twist=BAYO_TWIST) {
    slot_w    = tab_w + BAYO_SLOT_CLR * 2;
    slot_t    = tab_t + BAYO_SLOT_CLR * 2;
    cut_depth = BAYO_TAB_H + BAYO_SLOT_CLR;
    lock_drop = -2.0;  // mm - lock groove is this much shallower than entry bottom
                      //      tab rises into it on rotation, step prevents return
    angular_w = (slot_w / (host_od/2)) * (180/PI);
    
    union() {
        // Axial entry (unchanged - opens at face, tab slides in)
        rotate([0, 0, -angular_w/2])
            rotate_extrude(angle=angular_w)
                translate([host_od/2 - wall - 0.5, 0])
                    square([cut_depth, slot_t]);
        
        // Circumferential lock - shifted 2mm toward face from entry bottom
        // Was: translate([0, 0, 0])  ← no step, nothing to lock against
        // Now: translate([0, 0, lock_drop]) ← tab rises into this, step holds it
        translate([0, 0, lock_drop])
            rotate([0, 0, -angular_w/2])
                rotate_extrude(angle=twist + angular_w)
                    translate([host_od/2 - wall - 0.5, 0])
                        square([cut_depth, BAYO_TAB_T + BAYO_SLOT_CLR]);
    }
}

// ================================================================
// MODULE: bayonet_slots_ring
//   Places N bayonet slots around a host cylinder
// ================================================================
module bayonet_slots_ring(host_od, wall, count=BAYO_TABS) {
    for (i = [0:count-1]) {
        rotate([0, 0, i * (360/count)])
            bayonet_slot(host_od, wall);
    }
}


// ================================================================
// MODULE: thread_helix_ext
//   Fat external helical thread built from hulled radial slab pairs.
//   Each step hulls two thin profile slabs at consecutive (angle, z)
//   positions — giving the ridge genuine axial height THREAD_H
//   independent of pitch. Right-hand thread (angle decreases with z).
//
//   Chamfer = thread_h * 0.15 on top/bottom edges: gentle overhang,
//   prints without supports.
// ================================================================
module thread_helix_ext(r, n_turns=THREAD_TURNS, pitch=THREAD_PITCH,
                         depth=THREAD_DEPTH, thread_h=THREAD_H,
                         steps=THREAD_STEPS) {
    chamf = thread_h * 0.15;
    for (turn = [0:n_turns-1]) {
        for (i = [0:steps-1]) {
            a0 = i       * (-360/steps);
            a1 = (i + 1) * (-360/steps);
            z0 = turn * pitch + i       * pitch/steps;
            z1 = turn * pitch + (i + 1) * pitch/steps;
            hull() {
                for (az = [[a0, z0], [a1, z1]]) {
                    rotate([0, 0, az[0]]) translate([0, 0, az[1]])
                        rotate([90, 0, 0])
                            linear_extrude(0.001)
                                polygon([
                                    [r,         0             ],
                                    [r + depth, chamf         ],
                                    [r + depth, thread_h-chamf],
                                    [r,         thread_h      ]
                                ]);
                }
            }
        }
    }
}

// ================================================================
// MODULE: thread_helix_int_cut
//   Fat internal thread groove: same hull approach, profile starts
//   at r_bore (bore surface) and extends outward into the wall by depth.
//   Subtract this from the endcap bore to create matching female thread.
// ================================================================
module thread_helix_int_cut(r_bore, n_turns=THREAD_TURNS, pitch=THREAD_PITCH,
                              depth=THREAD_DEPTH, thread_h=THREAD_H,
                              steps=THREAD_STEPS) {
    chamf = thread_h * 0.15;
    for (turn = [0:n_turns-1]) {
        for (i = [0:steps-1]) {
            a0 = i       * (-360/steps);
            a1 = (i + 1) * (-360/steps);
            z0 = turn * pitch + i       * pitch/steps;
            z1 = turn * pitch + (i + 1) * pitch/steps;
            hull() {
                for (az = [[a0, z0], [a1, z1]]) {
                    rotate([0, 0, az[0]]) translate([0, 0, az[1]])
                        rotate([90, 0, 0])
                            linear_extrude(0.001)
                                polygon([
                                    [r_bore,         0             ],
                                    [r_bore + depth, chamf         ],
                                    [r_bore + depth, thread_h-chamf],
                                    [r_bore,         thread_h      ]
                                ]);
                }
            }
        }
    }
}


// ================================================================
// MODULE: handle
//   Hollow tube, smooth bore (v0.9 - no bayonet slots)
//   External thread at both ends — mates with endcap internal thread
//   OD 44mm / ID 40mm / 150mm height
// ================================================================
module handle() {
    thread_zone = THREAD_TURNS * THREAD_PITCH;
    // clip_d: wide enough to contain the full thread OD for intersection trimming
    clip_d   = HANDLE_OD + 2 * (THREAD_DEPTH + 1);
    // 0.1mm inset so thread base overlaps tube wall — no floating-point gap
    r_thread = HANDLE_OD/2 - 0.1;

    union() {
        difference() {
            cylinder(h=HANDLE_H, d=HANDLE_OD);
            translate([0, 0, -0.1])
                cylinder(h=HANDLE_H + 0.2, d=HANDLE_ID);
        }
        // Bottom thread — starts one pitch before z=0, clipped to [0, thread_zone].
        // The intersection trims both ends to a clean diagonal helix cross-section.
        intersection() {
            translate([0, 0, -THREAD_PITCH])
                thread_helix_ext(r=r_thread, n_turns=THREAD_TURNS + 1);
            cylinder(h=thread_zone, d=clip_d);
        }
        // Top thread — same trick, translated to top end
        translate([0, 0, HANDLE_H - thread_zone])
            intersection() {
                translate([0, 0, -THREAD_PITCH])
                    thread_helix_ext(r=r_thread, n_turns=THREAD_TURNS + 1);
                cylinder(h=thread_zone, d=clip_d);
            }
    }
}


// ================================================================
// MODULE: blade_segment_with_bulkhead
//   Tapered hollow segment with:
//   - Integrated bulkhead AT BOTTOM (flush with build plate, no supports needed)
//   - Triangle prism stub extends up 25mm from bulkhead
//   - Spigot at top with notches that clear the next segment's spokes
//   - Optional bayonet tabs at bottom (Seg 1 only)
//
//   PRINT ORIENTATION:
//   Segment prints right-side-up. Bulkhead spokes + triangle base sit
//   directly on the build plate at z=0. No support material needed.
//   Outer wall + spigot grow upward.
//
//   STACKING:
//   When upper segment sits on lower, lower's spigot rises into upper's
//   bore. Spigot has 3 notches at 90°/210°/330° that pass through upper's
//   bulkhead spokes. This also provides rotational alignment - segments
//   only seat in positions where triangle faces align.
//
// Parameters:
//   bot_od, bot_id      - outer/inner diameter at base
//   top_od, top_id      - outer/inner diameter at top
//   spigot_od           - spigot outer diameter (0 = no spigot)
//   has_spigot          - true for Seg 1-5, false for Seg 6
//   bayonet             - true for Seg 1 (adds tabs at bottom)
//   wire_passthrough    - true for Seg 1 (no bulkhead, just empty base for wires)
//   bulkhead_h          - thickness of bulkhead (3mm default)
//   tri_h               - height of triangle prism stub above bulkhead (25mm)
//   tri_side            - equilateral triangle side length (12mm)
//   spoke_w             - width of each spoke (3mm)
// ================================================================
module blade_segment_with_bulkhead(
    bot_od, bot_id, top_od, top_id,
    spigot_od=0, has_spigot=true,
    bayonet=false, wire_passthrough=false,
    bulkhead_h=3, tri_h=50,
    tri_side=12, spoke_w=3, notch_clr=1.5
) {
    R_tri = tri_side / sqrt(3);

    // ── Spigot geometry (calculated upfront - needed for bore blend) ──
    spigot_outer_r = (spigot_od + INTERFERENCE * 2) / 2;
    spigot_inner_r = spigot_outer_r - SPIGOT_WALL;

    // ── Bore blend: inner bore starts transitioning to spigot_inner_r
    //    this far below the body top. Eliminates horizontal ledge entirely.
    //    Spigot wall has full body material below it for support.
    blend_h = 50;  // mm - tune this, 10-20mm works well

    // Bore radius at the point where blend starts (linear interp along taper)
    bore_r_at_blend = bot_id/2 + (top_id/2 - bot_id/2) * ((BODY_H - blend_h) / BODY_H);
    
    union() {

        // ── TAPERED OUTER WALL WITH SNAP GROOVES ──
        difference() {
            cylinder(h=BODY_H, r1=bot_od/2, r2=top_od/2);

            // BORE: split into lower (normal taper) + upper (blends to spigot inner r)
            if (has_spigot && spigot_od > 0) {
                // Lower bore - normal taper
                translate([0, 0, -0.1])
                    cylinder(h=BODY_H - blend_h + 0.1,
                             r1=bot_id/2,
                             r2=bore_r_at_blend);
                // Upper bore - tapers inward to match spigot inner r exactly at top
                // This ramp IS the spigot support - no ledge, no overhang
                translate([0, 0, BODY_H - blend_h])
                    cylinder(h=blend_h + 0.1,
                             r1=bore_r_at_blend,
                             r2=spigot_inner_r);
            } else {
                // No spigot (Seg 6 tip) - normal bore all the way
                translate([0, 0, -0.1])
                    cylinder(h=BODY_H + 0.2, r1=bot_id/2, r2=top_id/2);
            }
            
            // ── SNAP GROOVES (3 grooves around the bore, near the bottom) ──
            // Position: SNAP_DEPTH mm from bottom of bore.
            // This matches the bump position in the lower segment's spigot
            // when fully seated.
            //
            // The groove cuts inward from the bore outer wall to give the bump
            // a recess to snap into. Groove cross-section is a small notch:
            //   width (axial) = SNAP_BUMP_L + 2*SNAP_GROOVE_CLR
            //   depth (radial) = SNAP_BUMP_H + SNAP_GROOVE_CLR
            //
            // Angular position: 30°, 150°, 270° - matches the spigot tab
            // centers (midway between spoke notches at 90/210/330).
            //
            // Angular width of groove: ~10° (covers the bump arc + slop)
            groove_arc = 14;
            for (angle = [30, 150, 270]) {
                rotate([0, 0, angle - groove_arc/2])
                    translate([0, 0, SNAP_DEPTH - SNAP_BUMP_L/2 - SNAP_GROOVE_CLR])
                        rotate_extrude(angle=groove_arc)
                            // Compute bore radius at SNAP_DEPTH for taper-aware groove placement
                            translate([bot_id/2 + (top_id - bot_id)/2 * (SNAP_DEPTH/BODY_H), 0])
                                square([SNAP_BUMP_H + SNAP_GROOVE_CLR, SNAP_BUMP_L + 2*SNAP_GROOVE_CLR]);
            }
        }
        
        // ── BULKHEAD AT BOTTOM (z=0 to z=bulkhead_h) ──
        // Skip bulkhead for Seg 1 if wire_passthrough - leave base open for wire entry
        if (!wire_passthrough) {
            // Three spokes from outer wall to triangle vertices
            for (angle = [90, 210, 330]) {
                rotate([0, 0, angle])
                    translate([R_tri-3, -spoke_w/2, 0])
                        cube([bot_id/1.4 - R_tri, spoke_w, bulkhead_h]);
            }
        } else {
            // For wire passthrough seg: still need the triangle to anchor
            // somehow. Use a partial bulkhead - just two spokes near top of
            // the empty space, leaving the bottom fully open for wire routing.
            // Spokes at z=15 instead of z=0 to leave wire entry room.
            translate([0, 0, 15]) {
                for (angle = [90, 210, 330]) {
                    rotate([0, 0, angle])
                        translate([R_tri-4, -spoke_w/2, 0])
                            cube([bot_id/1.6 - R_tri, spoke_w, bulkhead_h]);
                }
            }
        }
        
        // ── CENTER TRIANGLE PRISM STUB (z=0 to z=tri_h, or z=15 to z=15+tri_h for wire seg) ──
        tri_z_start = wire_passthrough ? 15 : 0;
        translate([0, 0, tri_z_start])
            linear_extrude(height=tri_h)
                polygon(points=[
                    [R_tri * cos(90),  R_tri * sin(90)],
                    [R_tri * cos(210), R_tri * sin(210)],
                    [R_tri * cos(330), R_tri * sin(330)]
                ]);
        
        // ── SPIGOT WITH NOTCHES AND SNAP BUMPS (top) ──
        // The spigot is a hollow ring that slots into next segment's bore.
        // It has 3 notches at 90°/210°/330° to clear that segment's spokes.
        // Between the notches, the spigot has 3 arc tabs.
        // Each arc tab has a snap bump on its outer surface (radially outward)
        // that engages with the next segment's bore groove.
        if (has_spigot && spigot_od > 0) {
            translate([0, 0, BODY_H]) {
                difference() {
                    difference() {
                        cylinder(h=SPIGOT_H, r=spigot_outer_r);
                        // Straight inner bore - matches body bore at junction exactly
                        translate([0, 0, -0.1])
                            cylinder(h=SPIGOT_H + 0.2, r=spigot_inner_r);
                    }
                    // Notches (unchanged)
                    notch_arc = (spoke_w + notch_clr) / (spigot_od/2) * (180/PI);
                    for (angle = [90, 210, 330]) {
                        rotate([0, 0, angle - notch_arc/2])
                            rotate_extrude(angle=notch_arc)
                                translate([(spigot_od/2) - SPIGOT_WALL - 1, 0])
                                    square([SPIGOT_WALL + 2, bulkhead_h + 30]);
                    }
                }
                
                // ── SNAP BUMPS on spigot arc tabs ──
                // Each bump sits centered on an arc tab (between two notches).
                // Arc tabs are centered at 30°, 150°, 270° (midway between
                // notches at 90/210/330).
                //
                // Position math:
                //   When upper segment fully seats on lower, lower's spigot
                //   base (z=0 in spigot local frame) aligns with upper's bore
                //   bottom (z=0 in upper bore frame).
                //   Therefore z in spigot local frame = z in upper bore frame.
                //   Groove is at z=SNAP_DEPTH in upper bore frame.
                //   Bump must be at z=SNAP_DEPTH in spigot local frame.
                //
                // Construction: use rotate_extrude to build each bump as an
                // arc segment. This is simpler than rotating linear extrusions.
                // The cross-section is defined in the (radius, z) plane and
                // extruded through an angular sweep around the Z axis.
                //
                // Cross-section is a chamfered ridge in the (r, z) plane:
                //   - r runs from spigot_outer_r (base) to spigot_outer_r + SNAP_BUMP_H (tip)
                //   - z runs through the bump axial extent, with chamfers on top and bottom
                
                bump_center_z = SNAP_DEPTH;
                bump_arc_w_deg = 12;  // angular width of each bump (degrees)
                
                // Cross-section polygon points (in r, z plane):
                //   Lower-left (base, bottom-axial): bump_center_z - L/2
                //   Lower-right (base, top-axial):   bump_center_z + L/2
                //   Upper-right (tip, top-axial):    bump_center_z + L/2 - C (chamfered)
                //   Upper-left (tip, bottom-axial):  bump_center_z - L/2 + C (chamfered)
                
                for (tab_center_angle = [30, 150, 270]) {
                    rotate([0, 0, tab_center_angle - bump_arc_w_deg/2])
                        rotate_extrude(angle=bump_arc_w_deg)
                            polygon(points=[
                                [spigot_outer_r, bump_center_z - SNAP_BUMP_L/2],
                                [spigot_outer_r + SNAP_BUMP_H, bump_center_z - SNAP_BUMP_L/2 + SNAP_CHAMFER],
                                [spigot_outer_r + SNAP_BUMP_H, bump_center_z + SNAP_BUMP_L/2 - SNAP_CHAMFER],
                                [spigot_outer_r, bump_center_z + SNAP_BUMP_L/2]
                            ]);
                }
            }
        }
        
        // ── RETENTION RING (bottom, for Seg 1 only) ──
        // Full circumference ring seats against end_cap_open inner shelf.
        // Ring OD (42mm) > endcap through-hole (40mm) → blade retained.
        // Ring OD (42mm) < endcap bore (44.7mm) → slides through freely.
        if (bayonet) {
            difference() {
                cylinder(h=SEG1_RING_H, d=SEG1_RING_OD);
                translate([0, 0, -0.1])
                    cylinder(h=SEG1_RING_H + 0.2, d=bot_od);
            }
        }
    }
}


// ================================================================
// LEGACY: blade_segment (without bulkhead) - kept for reference
// ================================================================
module blade_segment(bot_od, bot_id, top_od, top_id, spigot_od=0, has_spigot=true, bayonet=false) {
    blade_segment_with_bulkhead(
        bot_od, bot_id, top_od, top_id,
        spigot_od=spigot_od, has_spigot=has_spigot, bayonet=bayonet
    );
}


// ================================================================
// MODULE: tip_segment (Seg 6)
//   Tapered body + integrated hemisphere + bulkhead AT BOTTOM (build plate)
//   Triangle prism stub extends up 25mm from bulkhead.
//   No spigot needed - this is the top of the blade.
//   No notches needed in the body - nothing stacks on top.
// ================================================================
module tip_segment(bot_od, bot_id, top_od, top_id,
                   bulkhead_h=3, tri_h=25,
                   tri_side=12, spoke_w=3) {
    R_tri = tri_side / sqrt(3);
    
    union() {
        // ── HOLLOW TAPERED BODY + HEMISPHERE ──
        difference() {
            union() {
                cylinder(h=BODY_H, r1=bot_od/2, r2=top_od/2);
                translate([0, 0, BODY_H])
                    difference() {
                        sphere(r=TIP_R);
                        translate([0, 0, -TIP_R - 0.1])
                            cylinder(h=TIP_R + 0.1, r=TIP_R + 1);
                    }
            }
            // Subtract bore (hemisphere stays solid because subtraction stops at BODY_H)
            translate([0, 0, -0.1])
                cylinder(h=BODY_H + 0.1, r1=bot_id/2, r2=top_id/2);
        }
        
        // ── BULKHEAD AT BOTTOM ──
        for (angle = [90, 210, 330]) {
            rotate([0, 0, angle])
                translate([R_tri-3, -spoke_w/2, 0])
                    cube([bot_id/1.4 - R_tri, spoke_w, bulkhead_h]);
        }
        
        // ── CENTER TRIANGLE PRISM STUB ──
        linear_extrude(height=tri_h+50)
            polygon(points=[
                [R_tri * cos(90),  R_tri * sin(90)],
                [R_tri * cos(210), R_tri * sin(210)],
                [R_tri * cos(330), R_tri * sin(330)]
            ]);
        
        groove_arc = 14;
            for (angle = [30, 150, 270]) {
                rotate([0, 0, angle - groove_arc/2])
                    translate([0, 0, SNAP_DEPTH - SNAP_BUMP_L/2 - SNAP_GROOVE_CLR])
                        rotate_extrude(angle=groove_arc)
                            // Compute bore radius at SNAP_DEPTH for taper-aware groove placement
                            translate([bot_id/2 + (top_id - bot_id)/2 * (SNAP_DEPTH/BODY_H), 0])
                                square([SNAP_BUMP_H + SNAP_GROOVE_CLR, SNAP_BUMP_L + 2*SNAP_GROOVE_CLR]);
            }
        
    }
}


// ================================================================
// MODULE: triangle_prism
//   Triangular prism - constant cross-section along length
//   LED strips mount to the three flat faces
//   side_length = side of equilateral triangle (12mm to match LED strip width)
//   length = how long the prism is (along blade axis)
// ================================================================
module triangle_prism(side_length=12, length=150) {
    R = side_length / sqrt(3);  // circumradius
    linear_extrude(height=length)
        polygon(points=[
            [R * cos(90),  R * sin(90)],
            [R * cos(210), R * sin(210)],
            [R * cos(330), R * sin(330)]
        ]);
}


// ================================================================
// MODULE: stabilizer_collar
//   Outer ring sized to fit snugly inside blade bore at a specific point
//   Centered around the inner triangle prism
//   3 spokes connect the collar to the triangle (printed as one piece)
//   
//   collar_od = outer diameter (size to match bore ID at insertion depth)
//   tri_side = inner triangle side (12mm to match LED frame)
//   thickness = axial length of the collar (5-10mm is plenty)
//   spoke_w = width of each spoke (3-4mm structural)
// ================================================================
module stabilizer_collar(collar_od, tri_side=12, thickness=5, spoke_w=3) {
    R_tri = tri_side / sqrt(3);  // triangle circumradius
    R_collar = collar_od / 2;
    
    union() {
        // Outer ring (thin)
        linear_extrude(height=thickness)
            difference() {
                circle(r=R_collar);
                circle(r=R_collar - 1.5);  // 1.5mm wall on collar
            }
        
        // Three spokes connecting collar inner edge to triangle vertices
        // Triangle vertices are at angles 90, 210, 330
        for (angle = [90, 210, 330]) {
            rotate([0, 0, angle])
                translate([0, -spoke_w/2, 0])
                    cube([R_collar - 0.5, spoke_w, thickness]);
        }
        
        // Inner triangle (extruded same height as collar)
        linear_extrude(height=thickness)
            polygon(points=[
                [R_tri * cos(90),  R_tri * sin(90)],
                [R_tri * cos(210), R_tri * sin(210)],
                [R_tri * cos(330), R_tri * sin(330)]
            ]);
    }
}


// ================================================================
// MODULE: stabilizer_collar_with_passthrough
//   Same as stabilizer_collar but with wire passthrough hole through the
//   center of the inner triangle (for base of blade where wires come up
//   from the handle/controller)
// ================================================================
module stabilizer_collar_with_passthrough(collar_od, tri_side=12, thickness=5, spoke_w=3, hole_d=8) {
    difference() {
        stabilizer_collar(collar_od, tri_side, thickness, spoke_w);
        translate([0, 0, -0.1])
            cylinder(h=thickness + 0.2, d=hole_d);
    }
}


// ================================================================
// MODULE: end_cap_open (v0.9)
//   Flat-top collar that threads over handle end via internal thread.
//   Through-hole in face passes blade segments; Seg1 base ring (task 4)
//   seats against the inner face, retaining the blade assembly.
//
//   Thread geometry:
//     bore_d = HANDLE_OD + 2*THREAD_CLR  (minor-diameter bore, 44.7mm)
//     groove goes outward by THREAD_DEPTH — matches handle ext thread
//     ec_od  = bore_d + 2*(THREAD_DEPTH + WALL)
//
//   Through-hole = HANDLE_ID (40mm) — passes blade body (≤39.67mm).
//   Update blade_pass_d after task 5 seg rework if needed.
// ================================================================
module end_cap_open() {
    bore_d        = HANDLE_OD + 2 * THREAD_CLR;           // 45.10mm at CLR=0.55
    ec_od         = bore_d + 2 * (THREAD_DEPTH + WALL);   // 53.10mm
    thread_h      = THREAD_TURNS * THREAD_PITCH;           // 24mm — threaded zone
    ring_pocket_h = SEG1_RING_H + 1.0;                    // 5mm — ring clears thread zone
    collar_h      = 8.0;                                   // grip flange above pocket
    blade_pass_d  = HANDLE_ID;                             // 40mm through-hole
    r_groove      = bore_d/2 - 0.1;

    // Z stack (from open/bottom end):
    //   0           → thread_h       : bore_d, threaded  (handle threads engage here)
    //   thread_h    → thread_h+ring_pocket_h : bore_d, SMOOTH  (ring sits here, no thread conflict)
    //   thread_h+ring_pocket_h       : shelf — annulus bore_d→blade_pass_d catches ring
    //   thread_h+ring_pocket_h → top : blade_pass_d  (blade exits)
    //
    //   SEG1_RING_OD (42mm) > blade_pass_d (40mm) → caught by shelf ✓
    //   SEG1_RING_OD (42mm) < bore_d (45.1mm)     → clears bore walls ✓
    //   Ring pocket is above all thread grooves   → no interference ✓

    difference() {
        union() {
            cylinder(h=thread_h, d=ec_od);
            translate([0, 0, thread_h])
                cylinder(h=ring_pocket_h + collar_h, d=ec_od + 4.0);
        }
        // Bore — spans thread zone + ring pocket, shelf face at top of pocket
        translate([0, 0, -0.1])
            cylinder(h=thread_h + ring_pocket_h + 0.1, d=bore_d);
        // Blade through-hole above shelf
        translate([0, 0, thread_h + ring_pocket_h - 0.1])
            cylinder(h=collar_h + 0.2, d=blade_pass_d);
        // Internal thread — clipped to thread zone only, ring pocket stays smooth
        intersection() {
            translate([0, 0, -THREAD_PITCH])
                thread_helix_int_cut(r_bore=r_groove, n_turns=THREAD_TURNS + 1);
            cylinder(h=thread_h, r=100);
        }
    }
}

// ================================================================
// MODULE: end_cap_closed (v0.9)
//   Flat-top pommel that threads onto handle end — solid bore, no passthrough.
//   Same thread geometry as end_cap_open; bore is blind (solid above thread zone).
// ================================================================
module end_cap_closed() {
    bore_d   = HANDLE_OD + 2 * THREAD_CLR;           // 44.70mm
    ec_od    = bore_d + 2 * (THREAD_DEPTH + WALL);   // 51.70mm
    collar_h = 8.0;
    thread_h = THREAD_TURNS * THREAD_PITCH;
    total_h  = thread_h + collar_h;
    r_groove = bore_d/2 - 0.1;

    difference() {
        union() {
            cylinder(h=thread_h, d=ec_od);
            translate([0, 0, thread_h])
                cylinder(h=collar_h, d=ec_od + 4.0);
        }
        translate([0, 0, -0.1])
            cylinder(h=thread_h + 0.1, d=bore_d);
        // Internal thread — starts one pitch early, clipped flush with diagonal entry
        intersection() {
            translate([0, 0, -THREAD_PITCH])
                thread_helix_int_cut(r_bore=r_groove, n_turns=THREAD_TURNS + 1);
            cylinder(h=thread_h, r=100);
        }
    }
}


// ================================================================
// ██╗   ██╗██████╗     ██████╗ ███████╗███████╗██╗ ██████╗ ███╗   ██╗
// ██║   ██║╚════██╗    ██╔══██╗██╔════╝██╔════╝██║██╔════╝ ████╗  ██║
// ██║   ██║ █████╔╝    ██║  ██║█████╗  ███████╗██║██║  ███╗██╔██╗ ██║
// ╚██╗ ██╔╝██╔═══╝     ██║  ██║██╔══╝  ╚════██║██║██║   ██║██║╚██╗██║
//  ╚████╔╝ ███████╗    ██████╔╝███████╗███████║██║╚██████╔╝██║ ╚████║
//   ╚═══╝  ╚══════╝    ╚═════╝ ╚══════╝╚══════╝╚═╝ ╚═════╝ ╚═╝  ╚═══╝
// ================================================================
// V2 REDESIGN — new approach:
//   • Uniform blade segments: 20mm OD / 12mm ID / 4mm wall
//   • Threaded segment-to-segment joints (no bulkheads)
//   • Modular LED channel inserts (fit inside 12mm ID bore)
//   • Handle: 180mm max, threads blade-end, bayonet pommel-end
//   • Flat endcaps, double-blade coupler
// ================================================================

// ── V2 BLADE SEGMENT PARAMETERS ────────────────────────────────
BLADE_OD     = 24.0;   // mm - uniform body OD (bumped from 20mm to wall-budget threading)
BLADE_ID     = 12.0;   // mm - LED channel bore — constant throughout entire blade
BLADE_H      = 150.0;  // mm - body height per segment

// Blade thread — spigot-in-bore approach (uniform external OD, connection hidden inside)
//
//   CONCEPT: every segment has the same BLADE_OD wall from z=0 to z=BLADE_H.
//   At the BOTTOM (z=0..fem_h): bore is enlarged to FEM_D for female thread.
//     OD stays BLADE_OD — thread engagement zone is purely internal.
//   At the TOP (z=BLADE_H..+spigot_h): a narrower spigot (spigot_d < BLADE_OD)
//     extends above the body with male thread. When assembled, this spigot is
//     hidden inside the segment above. External view = two BLADE_OD tubes, seam only.
//
//   Key geometry:
//     FEM_D     = 20.0mm  female bore (fits inside BLADE_OD wall with 1mm margin past groove)
//     spigot_d  = FEM_D - 2*B_CLR = 19.2mm  (spigot OD = male minor diam)
//     spigot_h  = B_TURNS * B_PITCH = 12mm
//     Groove bottom = FEM_D/2 + B_DEPTH = 11mm radius < BLADE_OD/2 (12mm) ✓
//     Print height  = BLADE_H + spigot_h = 150 + 12 = 162mm (fits 170mm bed) ✓
//     45° internal chamfer transitions FEM_D bore → BLADE_ID bore (printable, no support)
B_PITCH      = 4.0;    // mm - thread pitch
B_DEPTH      = 1.0;    // mm - radial thread depth
B_THREAD_H   = 2.5;    // mm - axial thread height (fatness); must be < B_PITCH
B_TURNS      = 3;      // turns of engagement per joint
B_CLR        = 0.40;   // mm - radial clearance
B_STEPS      = 32;     // hull facets per turn (32=preview, 64=final)
FEM_D        = 18.4;   // mm - female bore diameter (thread engagement zone) — balanced: female+spigot walls both 2.8mm (was 20.0: female=2mm/1mm-behind-threads, spigot=3.6mm — severely unbalanced)
CHAMFER_H    = 4.0;    // mm - bore transition zone (FEM_D → BLADE_ID, ~45°, no supports)

// Seg1 retention ring — sits at base of Seg1, caught by v2_end_cap_open shelf.
// Ring OD > endcap through-hole (BLADE_OD + 1.0 = 25mm) so it catches.
// Ring OD < endcap bore_d (44.9mm) so it enters the pocket.
// Ring OD > V2_HDL_ID (38mm) so it cannot slide into handle bore.
// When assembled ring is hidden inside endcap.
BLADE_RING_OD = 41.0;  // mm - Seg1 retention ring outer diameter
                       //      > V2_HDL_ID (38mm) → can't slide into handle bore ✓
                       //      < endcap bore_d (44.9mm) → fits in endcap ring pocket ✓
                       //      > blade_pass_d (25mm) → caught by endcap shelf ✓
BLADE_RING_H  = 3.0;   // mm - Seg1 retention ring axial height
SEG1_FILLET_R = 2.0;   // mm - ring-to-body fillet torus minor radius (tune up for more blend)

// ── INVERTED BAYONET PARAMETERS ────────────────────────────────
// External L-slots cut into handle OD (groove floor faces bed — prints clean ✓).
// Matching inward tab lugs inside cap/coupler bore (solid protrusions, 1.5mm radial).
// No bore-interior groove = no overhang problem. Full coupler viable again.
IBAYO_TAB_H    = 1.8;   // mm - groove depth cut into handle OD (was 1.5; increased for more engagement)
IBAYO_GROOVE_Z = 8.0;   // mm - groove start z from handle pommel face (z=0)
                         //      Keyway opens from z=0; groove arc at z=IBAYO_GROOVE_Z..groove_end
                         //      groove_end = IBAYO_GROOVE_Z + BAYO_TAB_T + BAYO_SLOT_CLR*2 = 12.6mm
TRACE_CLR      = 0.0;   // mm - trace relief no longer needed; traces are now indented grooves
                         //      engage_bore in cap/coupler = V2_HDL_OD + 2*BAYO_SLOT_CLR (44.6mm)

// ── V2 HANDLE PARAMETERS ───────────────────────────────────────
V2_HDL_OD   = 44.0;   // mm - handle outer diameter (3mm wall around PCB bore)
V2_HDL_ID   = 38.0;   // mm - bore ID = former OD — clears full PCB+switch assembly
V2_HDL_H    = 165.0;  // mm - handle height (fits A1 Mini 170mm working height with margin)

// Handle ↔ endcap thread (proportional to 38mm OD)
V2_H_PITCH  = 6.0;    // mm
V2_H_DEPTH  = 1.5;    // mm
V2_H_TRDH   = 3.0;    // mm - axial thread height
V2_H_TURNS  = 3;      // turns
V2_H_CLR    = 0.45;   // mm - radial clearance

// ── PCB TRACE GRIP TEXTURE PARAMETERS ──────────────────────────
// Traces union onto handle surface — raised relief, no support needed.
// Pattern: two power-bus rings (top+bottom of grip zone) + signal traces
// routed between them with 45° jogs and via pads at junctions.
TRACE_D   = 1.6;   // mm - groove depth into handle surface (bumped 1.2→1.6: fixes printer sag, enables insert)
TRACE_W   = 1.6;   // mm - standard signal trace width (bumped 1.0→1.6: paint pen + insert clearance)
TRACE_WW  = 2.4;   // mm - power bus ring width (bumped 1.8→2.4)
TRACE_PAD = 2.2;   // mm - via pad radius (bumped 1.8→2.2)

// ── TRACE INSERT PARAMETERS ──────────────────────────────────────
// Black Tough+ inserts press into silver Tough+ grooves.
// Same material = identical shrinkage = predictable press-fit tolerance.
// INS_CLR = clearance per side. At 0.10mm same-material Tough+ = snug press-fit.
INS_CLR  = 0.10;                   // mm — press-fit clearance per side
INS_W    = TRACE_W  - 2*INS_CLR;  // mm — signal trace insert rod width  (1.4mm)
INS_WW   = TRACE_WW - 2*INS_CLR;  // mm — bus ring insert width          (2.2mm)
INS_D    = TRACE_D  - 0.1;        // mm — insert depth (0.1mm below flush, sand proud if needed)
INS_PAD  = TRACE_PAD - INS_CLR;   // mm — pad cap radius                 (2.1mm)

// ── V2 RETENTION RING (LED Seg1 insert ring, stacks with BLADE_RING in endcap pocket) ─
V2_RING_OD  = 41.0;   // mm - LED insert ring OD  > V2_HDL_ID (38mm) → can't slide into handle bore ✓
                      //                         < endcap bore_d (44.9mm) → fits in endcap ring pocket ✓
                      //                         > blade_pass_d (25mm) → caught by endcap shelf ✓
V2_RING_H   = 3.0;    // mm - LED insert ring axial height

// ── V2 LED CHANNEL INSERT PARAMETERS ───────────────────────────
// [-] profile: flat channel with two side rails, fits inside BLADE_ID (12mm) bore.
// Each insert section = one blade segment length (BLADE_H = 150mm).
// Sections join end-to-end; alignment tab keeps them rotationally indexed.
//
// Cross-section geometry:
//   Insert OD (rail tips): LED_INS_OD = 11.75mm  (0.25mm clearance in 12mm bore)
//   Channel width (face):  LED_CHAN_W = 10.0mm   (fits standard 10mm LED strip)
//   Rail height:           LED_RAIL_H = 1.5mm    (holds strip in channel)
//   Rail wall thickness:   LED_RAIL_T = 1.0mm
//   Web thickness:         LED_WEB_T  = 1.2mm    (back wall of channel, structural)
//   Alignment tab:         LED_TAB_W  = 2.0mm wide, LED_TAB_H = 1.5mm tall (radial)
//                          sits on bottom of channel opposite face (12 o'clock outside)
LED_INS_OD   = 11.75;  // mm - insert outer rail diameter (clears 12mm bore)
LED_CHAN_W   = 10.0;   // mm - LED channel opening width (10mm strip)
LED_RAIL_H   = 1.5;   // mm - rail height above channel face (retains strip sides)
LED_RAIL_T   = 1.0;   // mm - rail wall thickness
LED_WEB_T    = 1.2;   // mm - back web thickness (sets channel depth)
LED_TAB_W    = 2.0;   // mm - alignment tab width
LED_TAB_H    = 1.5;   // mm - alignment tab radial height (protrudes inward from bore)


// ================================================================
// MODULE: v2_blade_segment
//   Uniform-OD hollow tube (BLADE_OD throughout body length).
//   Threading is hidden inside: spigot inserts into segment above, female
//   bore receives spigot from below. External view = smooth BLADE_OD cylinder.
//
//   Print orientation: BOTTOM DOWN (female bore end on build plate).
//     - Female bore opens downward onto bed = fine (open hole on bed)
//     - Chamfer inside bore transitions FEM_D → BLADE_ID at 45° = printable
//     - Spigot grows upward from body top = no overhang
//
//   Z layout:
//     z = 0              bottom face (build plate)
//     z = 0..fem_h       body tube, BLADE_OD OD / FEM_D bore + female thread
//     z = fem_h          bore chamfer starts (FEM_D → BLADE_ID over CHAMFER_H)
//     z = fem_h+CHAMFER_H..BLADE_H  body tube, BLADE_OD OD / BLADE_ID bore
//     z = BLADE_H..BLADE_H+spigot_h spigot (spigot_d OD) — hides inside seg above
//
//   Assembled: spigot of lower seg screws into female bore of upper seg.
//   Both bodies flush at seam. Spigot invisible. External OD = uniform BLADE_OD.
//
//   seg1=true: adds BLADE_RING at z=0 base (retention ring for endcap sandwich).
//             Ring OD (41mm) > V2_HDL_ID (38mm) → won't slide into handle bore ✓
//             Ring OD (41mm) > endcap through-hole (25mm) → caught by shelf ✓
//             Straight BLADE_ID bore throughout — no female threads at bottom.
// ================================================================
module v2_blade_segment(seg1=false) {
    fem_h     = B_TURNS * B_PITCH;              // 12mm female engagement zone
    spigot_h  = fem_h;                          // 12mm spigot (same as female zone)
    spigot_d  = FEM_D - 2 * B_CLR;             // 19.2mm spigot OD (male minor diam)
    r_spigot  = spigot_d / 2 - 0.1;            // male thread base (0.1mm inset, overlap)
    r_bore    = FEM_D / 2 - 0.1;               // female groove base (0.1mm inset, overlap)
    clip_d    = spigot_d + 2 * (B_DEPTH + 1);  // ext clip cylinder for intersection

    union() {
        // ── BODY (z = 0..BLADE_H) ────────────────────────────────────
        // Outer wall = BLADE_OD from bottom to top. No step, no flange.
        difference() {
            cylinder(h=BLADE_H, d=BLADE_OD);

            if (seg1) {
                // Seg1: nothing connects into the bottom — straight LED bore throughout.
                // No female thread zone, no chamfer transition.
                translate([0,0,-0.1])
                    cylinder(h=BLADE_H + 0.2, d=BLADE_ID);
            } else {
                // Female bore zone (z=0..fem_h) — FEM_D enlarged bore, same OD outside
                translate([0,0,-0.1])
                    cylinder(h=fem_h + 0.1, d=FEM_D);

                // Chamfer transition (z=fem_h..fem_h+CHAMFER_H) — FEM_D → BLADE_ID
                // ~45° half-angle = printable without support; acts as spigot stop face
                translate([0,0, fem_h])
                    cylinder(h=CHAMFER_H + 0.1, r1=FEM_D/2, r2=BLADE_ID/2);

                // LED bore (z=fem_h+CHAMFER_H..BLADE_H) — 12mm, clear for LED inserts
                translate([0,0, fem_h + CHAMFER_H - 0.1])
                    cylinder(h=BLADE_H - fem_h - CHAMFER_H + 0.2, d=BLADE_ID);

                // Female thread groove — clipped to female bore zone only
                intersection() {
                    translate([0,0,-B_PITCH])
                        thread_helix_int_cut(r_bore=r_bore, n_turns=B_TURNS+1,
                                             pitch=B_PITCH, depth=B_DEPTH,
                                             thread_h=B_THREAD_H, steps=B_STEPS);
                    cylinder(h=fem_h, r=100);
                }

                // ── BORE-TO-CHAMFER FILLET ────────────────────────────
                // Drop test: segments 2+4 snapped at z=fem_h (base above threads).
                // Sharp inside corner at (r=FEM_D/2, z=fem_h) where the 2mm-wall
                // thread zone meets the chamfer is a stress riser. This torus cut
                // rounds that corner — reduces stress concentration at the transition.
                // fillet_r=1.5mm: spans z=10.5..13.5mm, entirely clear of thread zone.
                rotate_extrude($fn=128)
                    translate([FEM_D/2 - 1.5, fem_h])
                        circle(r=1.5, $fn=32);
            }
        }

        // ── SPIGOT (z = BLADE_H..BLADE_H+spigot_h) ─────────────────
        // Threaded post, narrower than body. Screws into female bore of segment above.
        // Carries LED bore through (BLADE_ID throughout).
        translate([0,0, BLADE_H]) {
            difference() {
                cylinder(h=spigot_h, d=spigot_d);
                translate([0,0,-0.1])
                    cylinder(h=spigot_h + 0.2, d=BLADE_ID);
            }
            // Male thread — clipped diagonal entry/exit same as handle thread
            intersection() {
                translate([0,0,-B_PITCH])
                    thread_helix_ext(r=r_spigot, n_turns=B_TURNS+1,
                                      pitch=B_PITCH, depth=B_DEPTH,
                                      thread_h=B_THREAD_H, steps=B_STEPS);
                cylinder(h=spigot_h, d=clip_d);
            }
        }

        // ── SEG 1 RETENTION RING (seg1=true only, z=0..BLADE_RING_H) ─
        // Thin collar at base of Seg1, slightly wider than BLADE_OD.
        // Sits in endcap ring pocket; caught by endcap shelf when blade is inserted.
        // Hidden inside endcap when assembled.
        if (seg1) {
            difference() {
                cylinder(h=BLADE_RING_H, d=BLADE_RING_OD);
                translate([0,0,-0.1])
                    cylinder(h=BLADE_RING_H + 0.2, d=BLADE_ID);
            }

            // ── RING-TO-BODY FILLET TORUS ──────────────────────────────
            // Failure mode: blade snapped at ring-to-body junction under
            // cantilever bending. BLADE_RING_OD (41mm) steps abruptly to
            // BLADE_OD (24mm) at z=BLADE_RING_H — massive stress concentration.
            // This torus fills the concave corner on the outside, blending
            // the ring into the body with a gradual taper.
            //
            // Center at (BLADE_OD/2 + SEG1_FILLET_R, BLADE_RING_H + SEG1_FILLET_R)
            // Spans r = BLADE_OD/2 .. BLADE_OD/2 + 2*SEG1_FILLET_R
            //       z = BLADE_RING_H .. BLADE_RING_H + 2*SEG1_FILLET_R
            // No LED bore conflict — torus is entirely outside body OD.
            // ⚠️  Also add slicer modifier at base zone (z=0..~12mm) to
            //     solid-fill the hollow shells there — see v2_blade_seg1_base_modifier()
            rotate_extrude($fn=128)
                translate([BLADE_OD/2 + SEG1_FILLET_R - 2.0, BLADE_RING_H + SEG1_FILLET_R - 2.0])
                    circle(r=SEG1_FILLET_R, $fn=64);
        }
    }
}


// ================================================================
// MODULE: v2_blade_seg1_base_modifier
//   Slicer modifier mesh for Seg1 base zone.
//   Import this STL into slicer as a modifier, set region to 100% infill.
//   Fills the hollow shell gap between inner/outer walls at the base,
//   making the ring-to-body junction a solid collar rather than two thin shells.
//   Height covers retention ring + fillet torus zone + small margin.
// ================================================================
module v2_blade_seg1_base_modifier() {
    mod_h = BLADE_RING_H + 2*SEG1_FILLET_R + 2.0;  // ring + fillet zone + 2mm margin
    cylinder(h=mod_h, d=BLADE_OD);
}

// ================================================================
// MODULE: v2_tip_segment  (Seg 5 — top of blade)
//   Same as v2_blade_segment body + female connection at bottom,
//   but with integrated hemisphere tip at top instead of spigot.
//   No segment above it (unless using double-blade coupler, which
//   replaces this segment and threads onto Seg4's spigot directly).
//
//   Hemisphere radius = BLADE_OD/2 = 12mm, flush with tube OD.
//   Bore is solid in hemisphere (bore stops at BLADE_H).
//   Print orientation: bottom down (same as standard segment).
//   Total print height = BLADE_H + hemisphere_r = 150 + 12 = 162mm ✓
// ================================================================
module v2_tip_segment() {
    fem_h  = B_TURNS * B_PITCH;
    r_bore = FEM_D / 2 - 0.1;
    tip_r  = BLADE_OD / 2;   // 12mm hemisphere, flush with tube OD

    union() {
        // ── BODY + HEMISPHERE (z = 0..BLADE_H + hemisphere) ─────────
        difference() {
            union() {
                cylinder(h=BLADE_H, d=BLADE_OD);
                translate([0,0,BLADE_H])
                    difference() {
                        sphere(r=tip_r);
                        translate([0,0,-tip_r - 0.1])
                            cylinder(h=tip_r + 0.1, r=tip_r + 1);
                    }
            }
            // Female bore zone (z=0..fem_h)
            translate([0,0,-0.1])
                cylinder(h=fem_h + 0.1, d=FEM_D);
            // Chamfer transition
            translate([0,0, fem_h])
                cylinder(h=CHAMFER_H + 0.1, r1=FEM_D/2, r2=BLADE_ID/2);
            // LED bore (stops at BLADE_H — hemisphere solid above)
            translate([0,0, fem_h + CHAMFER_H - 0.1])
                cylinder(h=BLADE_H - fem_h - CHAMFER_H + 0.1, d=BLADE_ID);
            // Female thread groove
            intersection() {
                translate([0,0,-B_PITCH])
                    thread_helix_int_cut(r_bore=r_bore, n_turns=B_TURNS+1,
                                         pitch=B_PITCH, depth=B_DEPTH,
                                         thread_h=B_THREAD_H, steps=B_STEPS);
                cylinder(h=fem_h, r=100);
            }
            // Bore-to-chamfer fillet (same as v2_blade_segment — see comment there)
            rotate_extrude($fn=128)
                translate([FEM_D/2 - 1.5, fem_h])
                    circle(r=1.5, $fn=32);
        }
    }
}


// ================================================================
// MODULE: v2_led_channel_insert
//   [-] profile LED channel insert for inside BLADE_ID (12mm) bore.
//   One section per blade segment; sections stack end-to-end.
//
//   Cross-section (looking down the Z axis, channel opens toward +Y):
//
//       ←— LED_INS_OD (11.75mm diam circle) —→
//        ___________________________________
//       |  rail  |  channel opening  |  rail |  ← LED_RAIL_H above face
//       |        |___________________|        |
//       |        ←  LED_CHAN_W (10mm) →       |  ← face (LED strip sits here)
//       |_____________________________________| ← back web (LED_WEB_T thick)
//              ↑ LED_TAB_H tab protrudes here (opposite side, alignment)
//
//   The cross-section is derived from the intersection of:
//     - A circle of radius LED_INS_OD/2 (outer rail envelope)
//     - A rectangle covering the lower half + channel + rails
//
//   Alignment tab: small rectangular protrusion on the back (bottom of channel
//   exterior) that fits a matching slot in the bore. Keeps all sections
//   rotationally indexed so LED strips line up at every joint.
//   (Slot in bore is added in v2_blade_segment when LED version is used —
//    for now the plain bore version is coded; tab slot is a future refinement.)
//
//   Length: LED_INS_H parameter (defaults to BLADE_H = 150mm).
//   Prints flat on its back face — no supports needed.
// ================================================================
module v2_led_channel_insert(ins_h=BLADE_H) {
    r_out   = LED_INS_OD / 2;        // 5.875mm — outer radius (fits in BLADE_ID 12mm bore)
    arc_t   = 0.6;                    // mm — ring wall thickness (pocket ~10.5mm wide for 10mm strip)
    r_inner = r_out - arc_t;          // 5.275mm — ring inner radius
    spine_t = LED_WEB_T;              // 1.2mm — center spine thickness

    // Access notch cut into ring arc, each side of spine, at top and bottom poles.
    // notch_w: tangential width of each notch gap beside spine
    // notch_y: rect base set low enough to intersect the curved ring arc across the full notch_w
    //          (ring inner boundary at x=notch_w is sqrt(r_inner²-notch_w²) ≈ 4.6mm; 0.85·r_inner≈4.5mm ✓)
    notch_w = 2.0;                    // mm — notch width each side of spine
    notch_y = r_inner * 0.85;         // 4.48mm — safely below ring inner arc within notch_w
    notch_h = r_out - notch_y + 0.1;  // ~1.50mm — clears ring OD

    // Cross-section = (|) with diagonal access notches:
    //   Spine runs full height, connecting to ring inner wall at ±Y poles.
    //   Two diagonal notch cuts (top-right + bottom-left) open access on each
    //   strip side without severing the ring — each arc half has one open corner
    //   and one closed corner, keeping the whole profile one connected piece.
    //   → LED strip slides in past the notch; squeeze glue in through same gap.
    //
    // Pocket width at spine face: 2·sqrt(r_inner² − (spine_t/2)²) ≈ 10.5mm  (10mm strip fits ✓)

    linear_extrude(height=ins_h) {
        difference() {
            union() {
                // ── THIN RING: both ( and ) arc shells ───────────────────
                difference() {
                    circle(r=r_out);
                    circle(r=r_inner);
                }
                // ── CENTER SPINE: | ───────────────────────────────────────
                // Full height — endpoints meet ring inner wall at ±Y poles.
                translate([-spine_t/2, -r_inner])
                    square([spine_t, r_inner * 2]);
            }
            // ── NOTCH CUTS: diagonal pair — one per LED strip side, opposing corners ─
            // Cutting ALL 4 corners severs each arc half at both ends → 3 floating pieces.
            // Cutting only a diagonal pair (top-right + bottom-left) leaves each arc half
            // with one open end and one closed end → entire ring stays one connected piece.
            translate([ spine_t/2,              notch_y])              square([notch_w, notch_h]);  // top-right
            translate([-(spine_t/2 + notch_w), -(notch_y + notch_h)]) square([notch_w, notch_h]);  // bottom-left
        }
    }
}


// ================================================================
// MODULE: v2_led_channel_seg1
//   LED channel insert for Seg 1 position only.
//   Identical cross-section to v2_led_channel_insert, but with a
//   retention ring at the BOTTOM end.
//
//   The ring is sandwiched between the handle top face and the underside
//   of Seg 1's female flange, locking the LED channel stack axially.
//   Ring OD must clear the handle bore (V2_HDL_ID) and be stopped by
//   the open endcap's inner shelf.
//
//   Ring geometry:
//     Ring OD  = LED_SEG1_RING_OD  (sits on endcap shelf, > endcap through-hole)
//     Ring ID  = BLADE_ID          (bore passes through ring)
//     Ring H   = V2_RING_H
//
//   Total insert length = V2_RING_H + BLADE_H (ring at bottom, insert runs full seg length)
//
//   Print orientation: ring-end DOWN.
// ================================================================

LED_SEG1_RING_OD = 41.0;   // mm — matches BLADE_RING_OD so both rings stack flush in pocket.
                             //       > V2_HDL_ID (38mm) → can't slide into handle bore ✓
                             //       < endcap bore_d (44.9mm) → fits in endcap ring pocket ✓
                             //       > blade_pass_d (25mm) → caught by endcap shelf ✓

module v2_led_channel_seg1() {
    union() {
        // ── INSERT BODY — starts at z=0, runs full seg length + ring zone ──
        // Channel body begins flush with the ring face, no gap or transition.
        v2_led_channel_insert(ins_h=V2_RING_H + BLADE_H);

        // ── RETENTION RING — annular flange at base ───────────────────
        // OD=LED_SEG1_RING_OD (caught by endcap shelf).
        // Bore = LED_INS_OD: matches insert outer diameter exactly →
        // shared cylindrical surface at r=5.875mm makes ring and insert one solid piece.
        difference() {
            cylinder(h=V2_RING_H, d=LED_SEG1_RING_OD);
            translate([0,0,-0.1])
                cylinder(h=V2_RING_H + 0.2, d=LED_INS_OD);
        }
    }
}


// ================================================================
// HELPER: _bdiag  (diagonal board trace — surface-hugging)
//   For traces that change BOTH angle and Z significantly.
//   Hull of two points cuts a chord through air; stepping in N
//   increments keeps each sub-hull nearly tangent to the surface.
//   Use for any diagonal with more than ~20° of angular sweep.
// ================================================================
module _bdiag(r, a0, z0, a1, z1, w=TRACE_W, steps=10) {
    for (i=[0:steps-1]) {
        fa = i/steps;  fb = (i+1)/steps;
        hull() {
            rotate([0,0, a0 + (a1-a0)*fa]) translate([r-TRACE_D, 0, z0 + (z1-z0)*fa])
                rotate([0,90,0]) cylinder(h=TRACE_D, r=w/2, $fn=8);
            rotate([0,0, a0 + (a1-a0)*fb]) translate([r-TRACE_D, 0, z0 + (z1-z0)*fb])
                rotate([0,90,0]) cylinder(h=TRACE_D, r=w/2, $fn=8);
        }
    }
}

// ================================================================
// HELPER: _bt  (board trace segment)
//   Hull of two radial cylinders at (angle,z) endpoints.
//   Creates a raised strip on a cylinder surface — straight, angled, or diagonal.
//   w = trace width, TRACE_D = protrusion depth (global).
// ================================================================
module _bt(r, a0, z0, a1, z1, w=TRACE_W) {
    hull() {
        rotate([0,0,a0]) translate([r-TRACE_D,0,z0]) rotate([0,90,0])
            cylinder(h=TRACE_D, r=w/2, $fn=8);
        rotate([0,0,a1]) translate([r-TRACE_D,0,z1]) rotate([0,90,0])
            cylinder(h=TRACE_D, r=w/2, $fn=8);
    }
}

// ================================================================
// HELPER: _bpad  (via pad)
//   Filled disc on cylinder surface at (angle, z). Marks junctions.
// ================================================================
module _bpad(r, a, z, pr=TRACE_PAD) {
    rotate([0,0,a]) translate([r-TRACE_D,0,z]) rotate([0,90,0])
        cylinder(h=TRACE_D+0.1, r=pr, $fn=16);
}

// ================================================================
// HELPER: _bring  (bus ring arc)
//   rotate_extrude arc on the cylinder surface. Full ring = a0=0,a1=360.
//   w = ring axial width.
// ================================================================
module _bring(r, z, a0=0, a1=360, w=TRACE_WW) {
    // Groove stays plain rectangular — untouched.
    // Chamfer = separate triangular cuts on the handle SURFACE just outside the
    // groove lips. Does not modify the groove shape or width at all.
    // Top chamfer: triangle from (r, z+w/2) → (r, z+w/2+ch) → (r-ch, z+w/2+ch)
    // Bottom chamfer: mirror below groove.
    ch = 0.4;

    // ── MAIN GROOVE ──────────────────────────────────────────────
    translate([0,0, z - w/2])
    rotate([0,0,a0])
        rotate_extrude(angle=a1-a0, $fn=128)
            translate([r-TRACE_D,0])
                square([TRACE_D+0.1, w]);

    // ── TOP EDGE CHAMFER (above groove, on handle cylinder surface) ─
    rotate([0,0,a0])
        rotate_extrude(angle=a1-a0, $fn=128)
            polygon([
                [r-ch,  z+w/2    ],   // inward at groove top edge
                [r+0.1, z+w/2    ],   // outer surface at groove top edge
                [r+0.1, z+w/2+ch ],   // up along outer surface
            ]);

    // ── BOTTOM EDGE CHAMFER (below groove, on handle cylinder surface) ─
    rotate([0,0,a0])
        rotate_extrude(angle=a1-a0, $fn=128)
            polygon([
                [r+0.1, z-w/2-ch ],   // down along outer surface
                [r+0.1, z-w/2    ],   // outer surface at groove bottom edge
                [r-ch,  z-w/2    ],   // inward at groove bottom edge
            ]);
}

// ================================================================
// INSERT HELPERS — positive-geometry counterparts to _bt / _bdiag / _bring / _bpad.
//   Produce the physical insert pieces to press into trace grooves.
//   Place at the SAME handle coordinates as the groove helpers they mirror.
//   All use INS_* params — slightly narrower/shallower than grooves for press fit.
//
//   Assembly order: place rod inserts first, drop pad caps last to cap joints.
// ================================================================

module _ins_bt(r, a0, z0, a1, z1, w=INS_W) {
    hull() {
        rotate([0,0,a0]) translate([r-INS_D,0,z0]) rotate([0,90,0])
            cylinder(h=INS_D, r=w/2, $fn=8);
        rotate([0,0,a1]) translate([r-INS_D,0,z1]) rotate([0,90,0])
            cylinder(h=INS_D, r=w/2, $fn=8);
    }
}

module _ins_diag(r, a0, z0, a1, z1, w=INS_W, steps=10) {
    for (i=[0:steps-1]) {
        fa = i/steps;  fb = (i+1)/steps;
        hull() {
            rotate([0,0, a0+(a1-a0)*fa]) translate([r-INS_D,0, z0+(z1-z0)*fa])
                rotate([0,90,0]) cylinder(h=INS_D, r=w/2, $fn=8);
            rotate([0,0, a0+(a1-a0)*fb]) translate([r-INS_D,0, z0+(z1-z0)*fb])
                rotate([0,90,0]) cylinder(h=INS_D, r=w/2, $fn=8);
        }
    }
}

// Bus rings and arc jogs. Full 360° rings must be printed as arc segments
// (cannot flex a rigid full ring onto the handle). See pcb_insert_print_plate().
module _ins_ring(r, z, a0=0, a1=360, w=INS_WW) {
    // Plain square profile — groove chamfer is external (doesn't affect insert fit).
    // Insert sits in the full-depth zone of the groove; chamfer relief is outside it.
    translate([0,0, z - w/2])
    rotate([0,0,a0])
        rotate_extrude(angle=a1-a0, $fn=128)
            translate([r-INS_D,0])
                square([INS_D, w]);
}

module _ins_bpad(r, a, z, pr=INS_PAD) {
    rotate([0,0,a]) translate([r-INS_D,0,z]) rotate([0,90,0])
        cylinder(h=INS_D+0.1, r=pr, $fn=16);
}


// ================================================================
// FLAT PRINT HELPERS — generate insert pieces in print-flat orientation.
//   Outer (handle-surface) face is at Z = d (top). Bed face at Z = 0.
//   Import directly into slicer — no rotation needed.
// ================================================================

// Straight rod lying along X axis. Length along X; circular cross-section w/2.
module ins_rod_flat(length, w=INS_W, d=INS_D) {
    hull() {
        cylinder(h=d, r=w/2, $fn=8);
        translate([length, 0, 0]) cylinder(h=d, r=w/2, $fn=8);
    }
}

// Arc segment, flat in XY plane (bed = bottom face).
// arc_deg = angular span. r_h = handle outer radius (sets the curve radius).
// Width w spans axially (Z); depth d is radial.
module ins_arc_flat(arc_deg, r_h=V2_HDL_OD/2, w=INS_W, d=INS_D) {
    rotate_extrude(angle=arc_deg, $fn=128)
        translate([r_h - d, 0])
            square([d, w]);
}

// Pad cap disc — flat on bed. Place last to cap trace-rod joints.
module ins_pad_flat(pr=INS_PAD, h=INS_D) {
    cylinder(h=h, r=pr, $fn=16);
}


// ================================================================
// MODULE: pcb_trace_inserts_placed
//   All trace insert pieces rendered at their handle positions.
//   Use for visual preview alongside v2_handle().
//   *** NOT for direct printing — pieces overlap at this assembly position. ***
//   For printing use pcb_insert_print_plate().
//
//   Bus rings rendered full 360° here for preview clarity.
//   For printing they are split into 3 × 120° arcs in the print plate.
// ================================================================
module pcb_trace_inserts_placed(r, z_bot, z_top) {
    h = z_top - z_bot;

    // ── POWER BUS RINGS ──────────────────────────────────────────
    _ins_ring(r, z_bot);
    _ins_ring(r, z_top);
    _ins_ring(r, z_bot + 0.48*h, 0, 180, INS_W+0.3);  // mid bus — 0°→180°

    // ── PAD CAPS ─────────────────────────────────────────────────
    for (a=[0,90,180,270]) _ins_bpad(r, a, z_bot);
    for (a=[0,135,180,315,45]) _ins_bpad(r, a, z_top);
    _ins_bpad(r,   0, z_bot+0.48*h);
    _ins_bpad(r,   0, z_bot+0.73*h);   // 45° upper arc meets 0° rail
    _ins_bpad(r, 180, z_bot+0.48*h);   // mid bus terminates at 180° rail
    _ins_bpad(r,  90, z_bot+0.48*h);   // 90° branch meets mid-bus ring
    _ins_bpad(r, 135, z_bot+0.48*h);  // 135° branch meets mid-bus ring
    _ins_bpad(r, 270, z_bot+0.55*h);
    _ins_bpad(r, 315, z_bot+0.70*h);
    // 45° pad at z_bot+0.73*h omitted — button hole at 45°, z=104.5..111.5mm

    // ── MAIN AXIAL RAILS ─────────────────────────────────────────
    _ins_bt(r,   0, z_bot,   0, z_top);
    _ins_bt(r, 180, z_bot, 180, z_top);

    // ── 90°→135° BRANCH — junction on mid-bus ring ───────────────
    _ins_bt(r,   90, z_bot,        90, z_bot+0.48*h);
    _ins_bt(r,  135, z_bot+0.48*h, 135, z_top);

    // ── 270°→315° BRANCH ─────────────────────────────────────────
    _ins_bt(r, 270, z_bot,         270, z_bot+0.55*h);
    _ins_diag(r, 270, z_bot+0.55*h, 315, z_bot+0.70*h);
    _ins_bt(r, 315, z_bot+0.70*h,  315, z_top);

    // 45° lower stub removed — pad at (45°, z_bot+0.48*h) on mid-bus marks the junction

    // ── 45° UPPER STUB ───────────────────────────────────────────
    _ins_ring(r, z_bot+0.73*h, 0, 36, INS_W);   // stops at button hole edge; pad at 0° junction
    _ins_bt(r,  45, 112.0,    45, z_top);

}


// ================================================================
// MODULE: pcb_insert_print_plate
//   All insert pieces laid flat for printing in BLACK Tough+.
//   Pieces are separated by type and arranged in rows.
//
//   SECTIONS:
//     [A] Straight rods — lying along X, ordered by length
//     [B] Arc jogs     — flat arcs in XY plane, handle radius curvature
//     [C] Bus ring arcs — 120° segments × 3 per ring (split for installation)
//     [D] Pad caps     — small discs, cluster at far right
//
//   Bed: 180×180mm. Longest piece = full rail ~125mm; fits diagonally if needed.
//   Print separately from handle (black filament).
// ================================================================
module pcb_insert_print_plate() {
    r_h  = V2_HDL_OD / 2;  // 22mm

    groove_end_g = IBAYO_GROOVE_Z + BAYO_TAB_T + 2*BAYO_SLOT_CLR;  // 12.6mm
    gz_bot_g     = groove_end_g + 4.0;                               // 16.6mm
    gz_top_g     = V2_HDL_H - V2_H_TURNS*V2_H_PITCH - 5;           // 142mm
    h            = gz_top_g - gz_bot_g;                              // 125.4mm
    gap          = 3;   // mm between rows

    // Trim applied to each rod END to clear pad circle pockets.
    // Rods stop short of pad centers; pad caps drop in after.
    // trim = INS_PAD + 0.3mm clearance per end.
    t = INS_PAD + 0.3;   // 2.4mm per end

    // ── [A] STRAIGHT RODS ────────────────────────────────────────
    // All rods trimmed by t each end to clear pad circles.
    // 0° rail split at z_bot+0.48h and z_bot+0.73h (internal pad junctions).
    // 180° rail split at z_bot+0.48h (internal pad junction).
    // Other rails only trimmed at endpoints (already split at mid-junction).
    //
    //   0° rail seg A:   z_bot       → z_bot+0.48h   (trimmed both ends)
    //   0° rail seg B:   z_bot+0.48h → z_bot+0.73h   (trimmed both ends)
    //   0° rail seg C:   z_bot+0.73h → z_top          (trimmed both ends)
    //   180° rail seg A: z_bot       → z_bot+0.48h   (trimmed both ends)
    //   180° rail seg B: z_bot+0.48h → z_top          (trimmed both ends)
    //   270° lower:      z_bot       → z_bot+0.55h   (trimmed both ends)
    //   135° upper:      z_bot+0.48h → z_top          (trimmed both ends)
    //   90°  lower:      z_bot       → z_bot+0.48h   (trimmed both ends)
    //   315° upper:      z_bot+0.70h → z_top          (trimmed both ends)
    //   45°  upper:      z=112       → z_top=142       (trimmed both ends)

    row = INS_W + gap;
    translate([0, row*0,  0]) ins_rod_flat(0.48*h - 2*t);    // 0° seg A
    translate([0, row*1,  0]) ins_rod_flat(0.25*h - 2*t);    // 0° seg B
    translate([0, row*2,  0]) ins_rod_flat(0.27*h - 2*t);    // 0° seg C
    translate([0, row*3,  0]) ins_rod_flat(0.48*h - 2*t);    // 180° seg A
    translate([0, row*4,  0]) ins_rod_flat(0.52*h - 2*t);    // 180° seg B
    translate([0, row*5,  0]) ins_rod_flat(0.55*h - 2*t);    // 270° lower
    translate([0, row*6,  0]) ins_rod_flat(0.52*h - 2*t);    // 135° upper
    translate([0, row*7,  0]) ins_rod_flat(0.48*h - 2*t);    // 90° lower
    translate([0, row*8,  0]) ins_rod_flat(0.30*h - 2*t);    // 315° upper
    translate([0, row*9,  0]) ins_rod_flat(30.0   - 2*t);    // 45° upper

    // ── [B] ARC PIECES ───────────────────────────────────────────
    arc_x = 0.55*h + gap + 5;  // offset past longest rod

    // Mid-bus: 0°→180°, wider groove (INS_W+0.3) — split into 2 × 90° arcs
    translate([arc_x,  0, 0]) ins_arc_flat(90, r_h, INS_W+0.3);
    translate([arc_x, 12, 0]) ins_arc_flat(90, r_h, INS_W+0.3);

    // Button approach arc: 0°→36°
    translate([arc_x, 24, 0]) ins_arc_flat(36, r_h, INS_W);

    // 270°→315° diagonal — approximated as a 45° arc for printing
    translate([arc_x, 34, 0]) ins_arc_flat(45, r_h, INS_W);

    // ── [C] BUS RING ARCS (INS_WW, split 3 × 120° per ring) ─────
    ring_x = arc_x + r_h*2 + gap + 5;

    // Bottom ring × 3
    translate([ring_x,  0, 0]) ins_arc_flat(120, r_h, INS_WW);
    translate([ring_x, 10, 0]) ins_arc_flat(120, r_h, INS_WW);
    translate([ring_x, 20, 0]) ins_arc_flat(120, r_h, INS_WW);

    // Top ring × 3 (identical shape)
    translate([ring_x, 30, 0]) ins_arc_flat(120, r_h, INS_WW);
    translate([ring_x, 40, 0]) ins_arc_flat(120, r_h, INS_WW);
    translate([ring_x, 50, 0]) ins_arc_flat(120, r_h, INS_WW);

    // ── [D] PAD CAPS ─────────────────────────────────────────────
    // 20 pads: 16 in pattern + 4 spare.
    pad_x = ring_x + r_h*2 + gap;
    pad_spacing = INS_PAD*2 + 2;
    for (i = [0:19])
        translate([pad_x + (i % 4) * pad_spacing,
                   floor(i / 4) * pad_spacing, 0])
            ins_pad_flat();
}


// ================================================================
// MODULE: pcb_insert_coupon
//   Single-piece coupon for fit-testing before committing to full plate.
//   Prints: one straight rod (signal trace) + one bus ring arc (120°) + one pad cap.
//   Test in a silver Tough+ coupon with matching grooves before full run.
// ================================================================
module pcb_insert_coupon() {
    r_h = V2_HDL_OD / 2;
    // Rod — standard signal trace, 30mm long
    translate([0, 0, 0]) ins_rod_flat(30);
    // Bus ring arc — 120° segment
    translate([35, 0, 0]) ins_arc_flat(120, r_h, INS_WW);
    // Pad cap
    translate([35 + r_h*2 + 5, 5, 0]) ins_pad_flat();
}


// ================================================================
// HELPER: v_guide_channel
//   Tapered axial channel carved into a bore wall — V funnel for bayonet tabs.
//   Channel is wide at the entry face and narrows linearly to exact slot width
//   at the lock position. Builds from stacked arc slices (no hull needed).
//
//   Function: the tab enters the wide end and is funneled to the slot.
//   Also provides radial clearance for tabs (BAYO_TAB_H=3mm) to pass through
//   a bore that's snug on the handle body (engage_bore). Without these channels
//   the 44mm tab-tip diameter would jam against the 38.6mm bore immediately.
//
//   r      = bore radius (engage_bore/2)
//   z0     = start z (entry face — wide end)
//   z1     = end z   (lock position — narrow end)
//   ang0   = angular width at entry (degrees; 60° = wide funnel, easy alignment)
//   ang1   = angular width at lock  (exact slot angular width)
//   depth  = radial depth of channel (= BAYO_TAB_H + BAYO_SLOT_CLR)
//   steps  = slices per channel (30 = smooth taper, fine for subtraction)
// ================================================================
module v_guide_channel(r, z0, z1, ang0, ang1, depth, steps=30) {
    slice_h = (z1 - z0) / steps;
    // union() first: CGAL subtracts one merged solid instead of N sequential children.
    // No z-overlap between slices (was +0.1) — eliminates coincident-face artifacts.
    // r offset -0.01: ensures channel starts just inside bore wall so no seam with bore cut.
    union() {
        for (i = [0 : steps-1]) {
            t   = i / steps;
            ang = ang0 * (1-t) + ang1 * t;
            z   = z0 + (z1 - z0) * t;
            translate([0, 0, z])
                rotate([0, 0, -ang/2])
                    rotate_extrude(angle=ang)
                        translate([r - 0.01, 0])
                            square([depth + 0.01, slice_h]);
        }
    }
}


// ================================================================
// MODULE: pcb_grip_traces
//   PCB trace pattern for the handle grip zone.
//   r      = handle outer radius
//   z_bot  = bottom of texture zone (above bayonet tab clearance)
//   z_top  = top of texture zone (below thread zone)
//
//   Layout (viewed unrolled, 0° = front):
//
//   POWER BUS RINGS at z_bot and z_top (full circumference, wide).
//   PARTIAL MID BUS at ~48% height (0°→210°, signal bus).
//
//   MAIN ROUTES (full-height axial traces):
//     0°  : z_bot→z_top  (primary power rail)
//     180°: z_bot→z_top  (return rail)
//
//   BRANCH ROUTING (signal traces with 45° jogs):
//     90°  → 135° jog at 45% height → 135° to z_top
//     270° → diagonal to 315° at 70% → 315° to z_top
//     45°  stub (lower) jogging back to 0° rail
//     45°  stub (upper) branching off 0° rail near top
//     225° fill stub connecting to 270° route
//     160° fill stub connecting to 180° rail
//     20°  fill stub connecting back to 0° rail
// ================================================================
module pcb_grip_traces(r, z_bot, z_top) {
    h = z_top - z_bot;

    // ── POWER BUS RINGS ──────────────────────────────────────────
    _bring(r, z_bot);                        // bottom power ring
    _bring(r, z_top);                        // top power ring
    _bring(r, z_bot + 0.48*h, 0, 180, TRACE_W+0.3);  // mid signal bus — 0°→180°

    // ── PADS at all major junctions ──────────────────────────────
    // Bottom ring intersections
    for (a=[0,90,180,270]) _bpad(r, a, z_bot);
    // Top ring intersections — only angles where traces actually reach z_top
    for (a=[0,135,180,315,45]) _bpad(r, a, z_top);
    // Mid bus intersections and branch junction pads
    _bpad(r,   0, z_bot+0.48*h);
    _bpad(r,   0, z_bot+0.73*h);   // 45° upper arc meets 0° rail
    _bpad(r, 180, z_bot+0.48*h);   // mid bus terminates at 180° rail
    _bpad(r,  90, z_bot+0.48*h);   // 90° branch meets mid-bus ring
    _bpad(r, 135, z_bot+0.48*h);  // 135° branch meets mid-bus ring
    _bpad(r, 270, z_bot+0.55*h);
    _bpad(r, 315, z_bot+0.70*h);
    _bpad(r,  45, z_bot+0.73*h);

    // ── MAIN AXIAL RAILS (full height) ───────────────────────────
    _bt(r,   0, z_bot,   0, z_top);
    _bt(r, 180, z_bot, 180, z_top);

    // ── 90°→135° BRANCH — junction moved onto mid-bus ring ───────
    // Arc jog removed — mid-bus ring (0°→180°) passes through 90° and 135°,
    // making the junction at z_bot+0.48*h. Pads at 90° and 135° on mid-bus mark it.
    _bt(r,   90, z_bot,        90, z_bot+0.48*h);        // 90° lower vertical
    _bt(r,  135, z_bot+0.48*h, 135, z_top);              // 135° upper vertical

    // ── 270°→315° BRANCH (surface-hugging diagonal) ──────────────
    _bt(r, 270, z_bot,         270, z_bot+0.55*h);        // 270° lower vertical
    _bdiag(r, 270, z_bot+0.55*h, 315, z_bot+0.70*h);      // diagonal — stepped, hugs surface
    _bt(r, 315, z_bot+0.70*h,  315, z_top);               // 315° upper vertical

    // 45° lower stub removed — pad at (45°, z_bot+0.48*h) on mid-bus marks the junction

    // ── 45° UPPER STUB (arc jog off 0° rail, branches up) ────────
    _bring(r, z_bot+0.73*h,      0,  36, TRACE_W);        // stops at button hole edge
    _bt(r,  45, 112.0,          45, z_top);
}


// ================================================================
// MODULE: ibayo_handle_slots
//   Subtracts 4× inverted-bayonet L-slots from handle exterior.
//   Each slot = axial keyway (z=0..groove_end) + circumferential lock arc.
//
//   Keyway:   axial cut at each tab position, allows lug to slide axially.
//   Lock arc: circumferential cut spanning BAYO_TWIST (45°) from keyway edge.
//   Groove floor at r = V2_HDL_OD/2 - groove_depth.  Prints groove-floor-down ✓
// ================================================================
module ibayo_handle_slots() {
    r_od         = V2_HDL_OD / 2;                              // 22mm
    groove_depth = IBAYO_TAB_H + BAYO_SLOT_CLR;               // 1.8mm
    groove_z     = IBAYO_GROOVE_Z;                             // 8.0mm from pommel face
    slot_t       = BAYO_TAB_T + BAYO_SLOT_CLR * 2;            // 4.6mm axial thickness
    groove_end   = groove_z + slot_t;                          // 12.6mm
    ang_key      = (BAYO_TAB_W + BAYO_SLOT_CLR*2) / r_od * (180/PI);  // ~17.2°

    for (i = [0:BAYO_TABS-1]) {
        rotate([0, 0, i * (360/BAYO_TABS)]) {
            // Axial keyway: entry slot from pommel face to groove_end
            rotate([0,0, -ang_key/2])
                rotate_extrude(angle=ang_key)
                    translate([r_od - groove_depth, -0.01])
                        square([groove_depth + 0.1, groove_end + 0.02]);

            // Circumferential lock arc: spans groove_z..groove_end, sweeps BAYO_TWIST
            rotate([0,0, -ang_key/2])
                rotate_extrude(angle=BAYO_TWIST + ang_key)
                    translate([r_od - groove_depth, groove_z])
                        square([groove_depth + 0.1, slot_t]);
        }
    }
}

// ================================================================
// MODULE: ibayo_int_tab
//   Single inward tab lug — protrudes IBAYO_TAB_H from bore wall.
//   Place at z=lug_z (base toward open face).
//   Chamfered inner corners for smooth entry and printability.
//   With bore open-end-down: lug underside at lug_z=8.3mm is 8.3mm above bed,
//   supported by bore wall below ✓.  Only 1.5mm radial span → bridges clean ✓.
// ================================================================
// lug_h = radial height of the lug protrusion from bore wall.
// Computed by caller: engage_bore/2 - V2_HDL_OD/2 + IBAYO_TAB_H - BAYO_SLOT_CLR
// This ensures lug_tip = groove_floor + BAYO_SLOT_CLR regardless of bore size.
module ibayo_int_tab(engage_bore, lug_h) {
    r_outer = engage_bore / 2 + 0.05;  // +0.05mm into wall — ensures union connectivity, not just flush touch
    r_inner = r_outer - lug_h;
    tab_t   = BAYO_TAB_T;
    tab_w   = BAYO_TAB_W;
    chamf   = 0.6;
    ang     = tab_w / r_outer * (180/PI);

    // Profile: rectangular lug with chamfered inner corners (r=r_inner, z=0 & z=tab_t).
    // Outer corners (r=r_outer) are clean joins to bore wall.
    rotate([0,0, -ang/2])
        rotate_extrude(angle=ang)
            polygon([
                [r_outer,         0           ],   // outer-bottom
                [r_outer,         tab_t       ],   // outer-top
                [r_inner + chamf, tab_t       ],   // inner-top chamfer base
                [r_inner,         tab_t-chamf ],   // inner-top chamfer apex
                [r_inner,         chamf       ],   // inner-bottom chamfer apex
                [r_inner + chamf, 0           ],   // inner-bottom chamfer base
            ]);
}

// ================================================================
// MODULE: ibayo_int_tabs_ring
//   Places BAYO_TABS inward lugs evenly around bore at z=lug_z.
//   lug_z = base of tab (closer to open/entry face).
//   lug_h = radial height of each lug (see ibayo_int_tab comment).
// ================================================================
module ibayo_int_tabs_ring(engage_bore, lug_z, lug_h) {
    for (i = [0:BAYO_TABS-1]) {
        rotate([0, 0, i * (360/BAYO_TABS)])
            translate([0, 0, lug_z])
                ibayo_int_tab(engage_bore, lug_h);
    }
}


// ================================================================
// MODULE: v2_handle
//   Smooth-bore hollow tube, 44mm OD / 38mm ID / 165mm height.
//
//   TOP (blade end):
//     External thread (V2_H_PITCH / V2_H_DEPTH / V2_H_TRDH) — hull-helix.
//     Mates with v2_end_cap_open internal thread.
//     Thread zone = V2_H_TURNS * V2_H_PITCH = 18mm from top.
//
//   BOTTOM (pommel end):
//     4× inverted-bayonet L-slots cut into handle exterior.
//     Keyway opens at pommel face; lock arc at z=8..12.6mm from pommel.
//     Cap / coupler has matching inward tab lugs.
//
//   Bore: smooth throughout (no L-slots, no thread on inside).
//   Print any orientation — slots open downward when blade-end-up ✓
// ================================================================
module v2_handle() {
    thread_zone = V2_H_TURNS * V2_H_PITCH;                             // 18mm
    clip_d      = V2_HDL_OD + 2 * (V2_H_DEPTH + 1);
    r_thread    = V2_HDL_OD / 2 - 0.1;
    groove_end  = IBAYO_GROOVE_Z + BAYO_TAB_T + BAYO_SLOT_CLR*2;      // 12.6mm
    // Grip texture zone: clear of pommel groove zone (bottom) and thread zone (top)
    gz_bot      = groove_end + 4.0;                                     // 16.6mm
    gz_top      = V2_HDL_H - thread_zone - 5;                          // 142mm

    union() {
        // ── MAIN TUBE + POMMEL SLOTS + HARDWARE HOLES ────────────
        // union() wrapper on subtracted children: CGAL merges all cuts into
        // one solid before applying difference — avoids assertion errors on
        // complex multi-child difference operations.
        difference() {
            cylinder(h=V2_HDL_H, d=V2_HDL_OD);
            union() {
                // Bore
                translate([0,0,-0.1])
                    cylinder(h=V2_HDL_H + 0.2, d=V2_HDL_ID);
                // Inverted bayonet L-slots cut into exterior near pommel
                ibayo_handle_slots();
                // ── MIC + BUTTON HOLES at -45° ────────────────────────
                // Mic: z=95mm, 9.7mm dia, 0.6mm membrane (inner bore outward)
                // Button: z=108mm, 7mm dia, full bore
                // Teardrop: hull(full-size hole + tiny top cylinder) gives
                // circle lower half + 45° spike upper half — no bridging needed.
                rotate([0, 0, -45]) {
                    // ── MIC POCKET — teardrop ─────────────────────────
                    // z=90, d=10. y_start pulled back to ID/2 - d/2 - 1 = 13mm so cut
                    // clears the full chord on the curved inner bore (bore curves to
                    // y≈18.33 at x=±5mm; old y=18.9 left a thin membrane at the sides).
                    // h = OD/2 - 0.6membrane - y_start = 22 - 0.6 - 13 = 8.4mm.
                    hull() {
                        translate([0, 13.0, 90])
                            rotate([-90, 0, 0])
                                cylinder(h = 8.4, d = 10.0);
                        translate([0, 13.0, 90 + 5.0])
                            rotate([-90, 0, 0])
                                cylinder(h = 8.4, d = 0.1);
                    }
                    // ── BUTTON HOLE — teardrop ────────────────────────
                    // y_start = ID/2 - d/2 - 1 = 14.5mm; h = OD/2 + 0.2 - 14.5 = 7.7mm.
                    hull() {
                        translate([0, 14.5, 108])
                            rotate([-90, 0, 0])
                                cylinder(h = 7.7, d = 7.0);
                        translate([0, 14.5, 108 + 3.5])
                            rotate([-90, 0, 0])
                                cylinder(h = 7.7, d = 0.1);
                    }
                }
                // ── PCB TRACE GRIP TEXTURE (indented) ─────────────────
                pcb_grip_traces(r=V2_HDL_OD/2, z_bot=gz_bot, z_top=gz_top);
            }
        }

        // ── BLADE-END THREAD (top) ────────────────────────────────
        translate([0,0, V2_HDL_H - thread_zone])
            intersection() {
                translate([0,0,-V2_H_PITCH])
                    thread_helix_ext(r=r_thread, n_turns=V2_H_TURNS+1,
                                      pitch=V2_H_PITCH, depth=V2_H_DEPTH,
                                      thread_h=V2_H_TRDH, steps=THREAD_STEPS);
                cylinder(h=thread_zone, d=clip_d);
            }
    }
}


// ================================================================
// MODULE: v2_end_cap_open
//   Blade-side endcap. Threads onto handle top (internal female thread).
//   Through-hole (BLADE_OD = 20mm) passes blade Seg 1 body.
//
//   Z stack (open/bottom face down):
//     0..thread_h       : threaded bore (bore_d), mates with handle ext thread
//     thread_h..thread_h+ring_pocket_h : smooth bore (bore_d), LED Seg1 ring sits here
//     thread_h+ring_pocket_h           : shelf — bore steps to BLADE_OD (20mm)
//     ..top                            : BLADE_OD through-hole, blade exits
//
//   Shelf catches BLADE_RING_OD (41mm) ring → ring > blade_pass_d (25mm) ✓
//   Ring clears bore_d (V2_HDL_OD + 2*V2_H_CLR = 44.9mm) ✓
//   Ring does NOT pass through handle bore (41mm > V2_HDL_ID 38mm) — correct,
//     it stacks in the endcap pocket during assembly ✓
// ================================================================
module v2_end_cap_open() {
    bore_d        = V2_HDL_OD + 2 * V2_H_CLR;               // 44.9mm — fit bore over handle OD
    ec_od         = bore_d + 2 * (V2_H_DEPTH + WALL) +0.6;        // 51.9mm — endcap OD; wall=3.5mm ✓
    thread_h      = V2_H_TURNS * V2_H_PITCH;                 // 18mm — threaded zone
    // Ring pocket holds BLADE_RING + V2_RING (LED insert ring) stacked.
    // Both are BLADE_RING_H + V2_RING_H = 3+3 = 6mm. Add 1mm slop.
    ring_pocket_h = BLADE_RING_H + V2_RING_H;               // 6mm — dual ring pocket (BLADE_RING_H=3 + V2_RING_H=3)
    collar_h      = 6.0;                                     // grip flange above pocket
    entry_collar  = 3.0;                                     // plain socket before threads — matches closed cap aesthetic
    blade_pass_d  = BLADE_OD + 0.8;                          // 25mm — blade passes with 0.5mm clearance
    r_groove      = bore_d / 2 - 0.1;

    difference() {
        union() {
            // Entry socket (plain) + threaded zone — same OD throughout
            cylinder(h=entry_collar + thread_h, d=ec_od);
            // Ring pocket + blade collar — wider OD
            translate([0,0, entry_collar + thread_h])
                cylinder(h=ring_pocket_h + collar_h, d=ec_od);
        }
        // Entry collar: wider bore clears handle thread tips (no engagement).
        // tip OD = V2_HDL_OD + 2×V2_H_DEPTH = 47mm; +1mm clearance = 48mm bore.
        translate([0,0,-0.1])
            cylinder(h=entry_collar + 0.1, d=V2_HDL_OD + 2*V2_H_DEPTH + 1.0);
        // Threaded bore + ring pocket (bore_d — thread engagement zone)
        translate([0,0, entry_collar - 0.1])
            cylinder(h=thread_h + ring_pocket_h + 0.1, d=bore_d);
        // Blade through-hole above shelf
        translate([0,0, entry_collar + thread_h + ring_pocket_h - 0.1])
            cylinder(h=collar_h + 0.2, d=blade_pass_d);
        // Internal thread groove — starts at entry_collar, clipped to thread zone only
        intersection() {
            translate([0,0, entry_collar - V2_H_PITCH])
                thread_helix_int_cut(r_bore=r_groove, n_turns=V2_H_TURNS+1,
                                      pitch=V2_H_PITCH, depth=V2_H_DEPTH,
                                      thread_h=V2_H_TRDH, steps=THREAD_STEPS);
            translate([0,0, entry_collar])
                cylinder(h=thread_h, r=100);
        }
    }
}


// ================================================================
// MODULE: v2_end_cap_closed  (pommel cap — inverted bayonet receiver)
//   Smooth bore slides over handle OD (44mm). 4× inward tab lugs engage
//   the external L-slots cut into the handle exterior.
//   Insert aligned with keyways → push in → twist 45° to lock.
//
//   Assembly: handle slides in POMMEL-FIRST until pommel face hits bore ceiling (z=bore_depth).
//   Groove (handle-z=8..12.6mm) maps to cap-z = bore_depth-12.6..bore_depth-8 = 9.4..14mm.
//   Lug at cap-z=9.7..13.7mm aligns with groove when fully seated. Twist to lock.
//
//   Z stack (z=0 = open face):
//     0..entry_chamf   : 45° entry lead-in (bore widens toward open face)
//     0..bore_depth    : smooth bore (engage_bore = 44.6mm)
//     lug_z..lug_z+4mm : 4× inward tab lugs at cap-z=9.7..13.7mm
//     bore_depth..total_h : solid pommel face
//
//   Geometry: cap_od=50mm. bore_depth=22mm. total_h=32mm.
//   lug_z = bore_depth − groove_end + BAYO_SLOT_CLR = 22 − 12.6 + 0.3 = 9.7mm
//
//   *** PRINT ORIENTATION: OPEN END DOWN (z=0 on build plate) ***
//     Smooth bore = no overhang issues ✓
//     Tab lugs at z≈8.3mm: only 1.5mm radial span from bore wall — bridges clean ✓
//     Solid pommel face at top ✓
// ================================================================
module v2_end_cap_closed() {
    cap_od      = 50.0;
    // fit_clr: tightened from BAYO_SLOT_CLR=0.3 → 0.15mm/side on test print feedback
    fit_clr     = 0.15;
    engage_bore = V2_HDL_OD + 2*fit_clr;                      // 44.3mm — snug fit
    bore_depth  = 15.0;    // handle pommel enters this far; bore ceiling is the insertion stop
    solid_h     = 10.0;
    total_h     = bore_depth + solid_h;            // 32mm
    entry_chamf = 1.5;                             // 45° lead-in at open face
    // Lug base z (from open face) = bore_depth - groove_end + BAYO_SLOT_CLR
    lug_z       = bore_depth - IBAYO_GROOVE_Z - BAYO_TAB_T - BAYO_SLOT_CLR;  // 9.7mm
    // Lug height computed to maintain BAYO_SLOT_CLR radial clearance in groove,
    lug_h       = engage_bore/2 - V2_HDL_OD/2 + IBAYO_TAB_H - fit_clr;  // 1.8mm — matches groove depth

    union() {
        difference() {
            cylinder(h=total_h, d=cap_od);
            union() {
                // Main bore — handle OD slides in smoothly
                translate([0,0,-0.1])
                    cylinder(h=bore_depth + 0.1, d=engage_bore);
                // Entry lead-in: 45° chamfer expands bore opening to ease insertion
                translate([0,0,-0.1])
                    cylinder(h=entry_chamf + 0.1,
                             r1=engage_bore/2 + entry_chamf,
                             r2=engage_bore/2);
            }
        }
        // 4× inward tab lugs engage handle exterior L-slots
        ibayo_int_tabs_ring(engage_bore=engage_bore, lug_z=lug_z, lug_h=lug_h);
    }
}


// ================================================================
// MODULE: v2_double_blade_coupler
//   180mm tube connecting two handles for double-blade config.
//   Inverted bayonet: smooth bore + 4× inward tab lugs at each end.
//   Handles insert 80mm and twist 45° to lock.
//
//   Inverted bayonet viability:
//     Tab lugs are solid protrusions — no groove cuts into wall.
//     No bore-interior grooves = no orientation constraint. Print either end down ✓
//     Half-coupler still provided for those who want faster/simpler prints.
//
//   Insertion depth = half=90mm (pommel face hits center detent ring).
//   Groove (handle-z=8..12.6mm) maps to coupler-z = 77.4..82mm from entry face.
//     lug_z_bot = 77.7mm from z=0 (bottom entry face)
//     lug_z_top = mirrored from COUPLER_H (top entry face)
//
//   Center detent: symmetric V-ring (triangular cross-section, 45° both sides).
//   Both handles butt against it when fully inserted.
//   Two 80mm insertions + 20mm center gap = 180mm ✓
//
//   *** PRINT ORIENTATION: any — print upright (z=0 on bed), no issues ✓ ***
//     Tab lugs at z≈67.7mm and z≈112.3mm: 1.5mm radial span → bridges clean ✓
//     No groove cuts = no overhang problem at either end ✓
// ================================================================

COUPLER_H        = 180.0;  // mm — total coupler length
GRIP_N           = 3;      // number of grip rings
GRIP_WIDTH       = 10.0;   // mm — axial width of each ring groove
GRIP_DEPTH       = 1.2;    // mm — groove depth (radial)
GRIP_CHAMF       = 3.0;    // mm — chamfer on each side of groove (flat floor between)

// ================================================================
// MODULE: coupler_grip_rings
//   Simple circumferential groove rings for coupler grip texture.
//   n evenly-spaced grooves between z_bot and z_top.
//   Replaces PCB trace pattern — coupler has no electronics, clean
//   ring grooves look intentionally mechanical and are safe for thin walls.
//
//   depth = 0.8mm (safe for 2.7mm coupler wall — leaves 1.9mm) ✓
//   width = 1.5mm axial
// ================================================================
module coupler_grip_rings(r, z_bot, z_top, n=GRIP_N, depth=GRIP_DEPTH, width=GRIP_WIDTH) {
    // Groove cross-section: GRIP_CHAMF entry on each side, flat floor between.
    // chamf fixed (not width/2) → sharp angled entry, flat channel floor. Machined-channel look.
    chamf = GRIP_CHAMF;
    step = (z_top - z_bot) / (n + 1);
    for (i = [1:n]) {
        translate([0, 0, z_bot + i*step - width/2])
            rotate_extrude($fn=128)
                polygon([
                    [r - depth, chamf        ],   // floor left
                    [r - depth, width - chamf],   // floor right
                    [r + 0.1,   width        ],   // surface right
                    [r + 0.1,   0            ],   // surface left
                ]);
    }
}

module v2_double_blade_coupler() {
    cap_od           = 50.0;  // slimmed from 56mm — matches closed cap OD, 2.45mm wall ✓
    fit_clr          = 0.15;  // tightened from BAYO_SLOT_CLR=0.3 → snug fit on test print
    engage_bore      = V2_HDL_OD + 2*fit_clr + 1;              // 44.3mm — snug fit
    half             = COUPLER_H / 2;                       // 90mm
    entry_chamf      = 1.5;
    lug_z_coupler    = half - IBAYO_GROOVE_Z - BAYO_TAB_T - BAYO_SLOT_CLR;  // 77.7mm
    lug_h            = engage_bore/2 - V2_HDL_OD/2 + IBAYO_TAB_H - fit_clr;  // 1.8mm

    union() {
        // ── MAIN TUBE ─────────────────────────────────────────────
        difference() {
            cylinder(h=COUPLER_H, d=cap_od);
            union() {
                // Full bore
                translate([0,0,-0.1])
                    cylinder(h=COUPLER_H+0.2, d=engage_bore);
                // Entry lead-ins: 45° chamfer at each face
                translate([0,0,-0.1])
                    cylinder(h=entry_chamf+0.1,
                             r1=engage_bore/2+entry_chamf, r2=engage_bore/2);
                translate([0,0, COUPLER_H - entry_chamf])
                    cylinder(h=entry_chamf+0.1,
                             r1=engage_bore/2, r2=engage_bore/2+entry_chamf);
                // ── GRIP RINGS — bottom half ─────────────────────────
                coupler_grip_rings(r=cap_od/2, z_bot=8.0, z_top=half - 3.0);
                // ── GRIP RINGS — top half (mirrored) ─────────────────
                translate([0,0, COUPLER_H]) mirror([0,0,1])
                    coupler_grip_rings(r=cap_od/2, z_bot=8.0, z_top=half - 3.0);
            }
        }

        // ── TAB LUGS — BOTTOM END (z=0 entry face) ───────────────
        ibayo_int_tabs_ring(engage_bore=engage_bore, lug_z=lug_z_coupler, lug_h=lug_h);

        // ── TAB LUGS — TOP END (COUPLER_H entry face, mirrored) ──
        // Same angular position as bottom — mirror only, no rotation offset.
        // +45° offset was tested and made entry/lock asymmetric relative to the
        // coupler seam reference. Symmetric lugs = consistent feel both sides.
        translate([0,0, COUPLER_H]) mirror([0,0,1])
            ibayo_int_tabs_ring(engage_bore=engage_bore, lug_z=lug_z_coupler, lug_h=lug_h);

        // ── CENTER STOP DETENT — removed ──────────────────────────
        // V-ring caused pommel face to bind against ridge during bayonet twist.
        // Handles bottom out naturally at lug engagement depth; detent not needed.
    }
}


// ================================================================
// MODULE: v2_double_blade_coupler_half
//   90mm half-coupler — print 2×, bond at center seam with CA/epoxy.
//   Inverted bayonet: smooth bore + 4× inward tab lugs at entry end (z=0).
//   Bond face at z=half (z=90mm, flat top).
//
//   Lug position: same lug_z_coupler = 67.7mm as full coupler.
//   Half-detent at bond face — chamfered underside (45°), no flat overhang.
//   When two halves bond, detent halves form full stop ring for both handles.
//
//   *** PRINT ORIENTATION: ENTRY FACE DOWN (z=0 on build plate) ***
//     Tab lugs at z≈67.7mm: 1.5mm radial span, bridges clean ✓
//     Half-detent at z≈88mm: chamfered underside = no floating cantilever ✓
//     Bond face flat on top ✓
// ================================================================
module v2_double_blade_coupler_half() {
    cap_od           = 50.0;  // slimmed from 56mm — matches closed cap OD, 2.45mm wall ✓
    fit_clr          = 0.15;  // tightened from BAYO_SLOT_CLR=0.3 → snug fit on test print
    engage_bore      = V2_HDL_OD + 2*fit_clr;              // 44.3mm — snug fit
    half             = COUPLER_H / 2;                       // 90mm
    entry_chamf      = 1.5;
    lug_z_coupler    = half - IBAYO_GROOVE_Z - BAYO_TAB_T - BAYO_SLOT_CLR;  // 77.7mm
    lug_h            = engage_bore/2 - V2_HDL_OD/2 + IBAYO_TAB_H - fit_clr;  // 1.8mm

    union() {
        // ── MAIN TUBE ─────────────────────────────────────────────
        difference() {
            cylinder(h=half, d=cap_od);
            union() {
                translate([0,0,-0.1])
                    cylinder(h=half+0.2, d=engage_bore);
                // Entry lead-in at bayonet face (z=0)
                translate([0,0,-0.1])
                    cylinder(h=entry_chamf+0.1,
                             r1=engage_bore/2+entry_chamf, r2=engage_bore/2);
                // ── GRIP RINGS ───────────────────────────────────────
                coupler_grip_rings(r=cap_od/2, z_bot=8.0, z_top=half - 3.0);
            }
        }

        // ── TAB LUGS — entry face (z=0) ──────────────────────────
        ibayo_int_tabs_ring(engage_bore=engage_bore, lug_z=lug_z_coupler, lug_h=lug_h);

        // ── HALF-DETENT — removed ─────────────────────────────────
        // Removed along with full coupler detent — caused pommel binding on twist.
    }
}


// ================================================================
// MODULE: v2_spacer_ring
//   Flat annular shim ring for the open endcap blade sandwich.
//   Same OD and height as the Seg1 / LED-insert retention rings,
//   but bore is large enough to slip over a blade segment body.
//
//   Use to add thickness to the endcap ring sandwich if needed
//   (e.g., to take up axial slop or extend the stack height).
//
//   ring_od = BLADE_RING_OD (41mm) — stacks flush with retention rings in pocket
//   ring_id = BLADE_OD + 1.0 (25mm) — 0.5mm radial clearance over blade body
//   ring_h  = BLADE_RING_H  (3mm)   — same height as retention ring
//
//   Print flat on either face — no supports needed.
// ================================================================
module v2_spacer_ring() {
    ring_od = BLADE_RING_OD;          // 41mm
    ring_id = BLADE_OD + 1.0;         // 25mm — clears blade segment OD (24mm) ✓
    ring_h  = BLADE_RING_H;           // 3mm

    difference() {
        cylinder(h=ring_h, d=ring_od);
        translate([0,0,-0.1])
            cylinder(h=ring_h + 0.2, d=ring_id);
    }
}


// ================================================================
// RENDER SECTION  (v0.9)
// Uncomment ONE component at a time, then F6 and export STL
// ================================================================

// ── HANDLE ─────────────────────────────────────────────────────
// handle();

// ── BLADE SEGMENTS (v0.9 — 5 segments, each shifted down one slot) ──
// notch_clr increases for smaller segments to maintain consistent fit.
//
// Seg 1 — retention ring at base (bayonet=true), spigot at top
// blade_segment_with_bulkhead(39.67, 35.67, 35.33, 31.33, spigot_od=31.33, has_spigot=true, bayonet=true, notch_clr=1.5);

// Seg 2
// blade_segment_with_bulkhead(35.33, 31.33, 31.00, 27.00, spigot_od=27.00, notch_clr=1.7);

// Seg 3
// blade_segment_with_bulkhead(31.00, 27.00, 26.67, 22.67, spigot_od=22.67, notch_clr=2.0);

// Seg 4
// blade_segment_with_bulkhead(26.67, 22.67, 22.33, 18.33, spigot_od=18.33, notch_clr=2.3);

// Seg 5 — integrated hemisphere tip, no spigot
// tip_segment(22.33, 18.33, 18.00, 14.00);

// ── END CAPS ───────────────────────────────────────────────────
// end_cap_open();    // v0.9 blade-side endcap
// end_cap_closed();  // v0.9 pommel-side endcap

// ── ACTIVE RENDER — change this line to switch parts ───────────
// PRINT NOTES (inverted bayonet — all parts print support-free):
//   v2_handle()                    → print any orientation; blade end up recommended
//   v2_end_cap_closed()            → print OPEN END DOWN (z=0 on bed)
//   v2_double_blade_coupler()      → print any orientation; no groove cuts ✓
//   v2_double_blade_coupler_half() → print ENTRY FACE DOWN (z=0 on bed); bond 2× at center
//
// TRACE INSERT WORKFLOW:
//   1. Preview inserts on handle: uncomment both lines below and render together
//      v2_handle();
//      pcb_trace_inserts_placed(V2_HDL_OD/2, gz_bot_val, gz_top_val);  // SEE NOTE
//      NOTE: gz_bot=16.6mm, gz_top=142mm — pass those values directly
//   2. Coupon test (print first, before full plate):
//      pcb_insert_coupon()         → rod + ring arc + pad; print in BLACK Tough+
//   3. Full print plate:
//      pcb_insert_print_plate()    → all pieces flat; print in BLACK Tough+
//      IMPORTANT: Bus rings are 120° arcs — flex slightly to install (same-material fine)
//
// v2_handle();
// v2_end_cap_open();
// v2_end_cap_closed();
// v2_led_channel_insert();
// coupler_ring_insert_plate();
// v2_double_blade_coupler();
// v2_double_blade_coupler_half();  // 90mm half — print 2×, bond at center seam
// v2_double_blade_coupler();       // full 180mm — single print, any orientation ✓
// v2_spacer_ring();                // 41mm OD shim ring for endcap sandwich (25mm ID, 3mm H)
// v2_blade_segment();              // Seg 2–4
// v2_blade_segment(seg1=true);     // Seg 1
// v2_tip_segment();                // Seg 5
// v2_led_channel_insert();
// v2_led_channel_seg1();

// ── STANDALONE TRIANGLE/STABILIZER MODULES (kept for flexibility) ─
// triangle_prism(side_length=12, length=150);
// stabilizer_collar(collar_od=13.5, tri_side=12, thickness=5);
// stabilizer_collar(collar_od=26.5, tri_side=12, thickness=5);
// stabilizer_collar_with_passthrough(collar_od=39.5, tri_side=12, thickness=8, hole_d=8);

// ── V2 COMPONENTS ──────────────────────────────────────────────
// All 5 body segments are identical (v2_blade_segment).
// Only the tip differs (v2_tip_segment — hemisphere, no male thread).
// Print orientation: flange-end DOWN (flat face on build plate, no supports).
// Total print height per segment: B_TURNS*B_PITCH + BLADE_H + B_TURNS*B_PITCH
//                                = 12 + 150 + 12 = 174mm (fits 180mm bed).
//
// v2_blade_segment(seg1=true); // Seg 1 only — adds retention ring at base
// v2_blade_segment();          // Seg 2–4 (all identical — print 3 of these)
// v2_tip_segment();            // Seg 5 (top of blade — hemisphere tip)
// v2_led_channel_insert();     // LED channel insert section (1 per blade segment, Seg 2–5)
// v2_led_channel_seg1();       // LED channel insert with retention ring (Seg 1 only)
// v2_handle();                 // V2 handle (thread top / bayonet tabs bottom)
// pcb_insert_coupon();         // fit-test coupon: 1 rod + 1 ring arc + 1 pad — print FIRST
// pcb_insert_print_plate();    // full insert plate — all pieces flat, BLACK Tough+
// coupler_ring_insert_half();  // single half-ring preview — one coupler groove insert
// coupler_ring_insert_plate(); // print plate: 12 halves (2/groove × 6 grooves) — BLACK Tough+
// v2_end_cap_open();           // V2 open endcap (threads onto handle blade-end, 20mm through-hole)
v2_end_cap_closed();         // V2 closed endcap (bayonet socket over handle pommel tabs)
// v2_double_blade_coupler();      // V2 coupler (180mm, V-guide both ends, uniform 50mm OD) — exceeds bed, use half
// v2_double_blade_coupler_half(); // print 2× and bond at center seam (90mm each — fits A1 Mini)

// ================================================================
// MODULE: coupler_ring_insert_half
// ================================================================
//   Decorative half-ring insert for one coupler V-groove.
//   Printed in contrast color (black Tough+) and CA-glued into groove.
//
//   Geometry matches the coupler_grip_rings() V-profile with clr=0.15mm
//   clearance on all faces — leaves a thin CA glue gap, prevents proud fit.
//
//   Cross-section (triangle, r-z plane):
//     outer left:  [r-clr, 0]            outer right: [r-clr, width-2*clr]
//     inner tip:   [r-depth+clr, center]
//   Outer face sits 0.15mm below coupler OD (flush, no proud edge).
//   Inner tip stops 0.15mm above groove floor.
//   Both axial faces inset 0.15mm from groove side walls.
//
//   PRINT: 2 halves per groove × 6 grooves = 12 pieces total.
//          See coupler_ring_insert_plate() for a ready-to-print layout.
//          Orientation: arch lying flat in XY, height = width-2*clr ≈ 3.7mm.
//          No supports needed. Use a brim if first-layer adhesion is marginal.
// ================================================================

module coupler_ring_insert_half(
    r     = 25.0,        // coupler OD/2 — cap_od=50mm → r=25mm
    depth = GRIP_DEPTH,  // tracks GRIP_DEPTH — groove depth
    width = GRIP_WIDTH,  // tracks GRIP_WIDTH — groove axial width
    clr   = 0.15,        // radial clearance at inner tip — CA glue gap
    proud = 2.0,         // mm — outer face protrusion above coupler OD
                         //      0 = flush band; try 0.5–1.5 for a rounded bump
    n_arc = 16           // outer arc smoothness (more = rounder, slower preview)
) {
    w = width;

    // Outer profile:
    //   proud=0 → flat band flush at r (2 points)
    //   proud>0 → circular arc through (r,0), (r+proud, w/2), (r,w)
    //             Arc centre at (cr, w/2), radius Rarc.
    cr   = (proud > 0) ? r + proud/2 - w*w/(8*proud) : r;
    Rarc = (proud > 0) ? proud/2     + w*w/(8*proud)  : 0;

    outer = (proud > 0)
        ? [for (i = [0 : n_arc])
            let(a = asin(-w / (2*Rarc))
                  + (asin(w / (2*Rarc)) - asin(-w / (2*Rarc))) * i / n_arc)
            [cr + Rarc*cos(a), w/2 + Rarc*sin(a)]
          ]
        : [[r, 0], [r, w]];

    rotate_extrude(angle=180, $fn=256)
        polygon(concat(
            outer,
            [
                [r - depth + clr, w - GRIP_CHAMF],  // inner right
                [r - depth + clr, GRIP_CHAMF    ],  // inner left
            ]
        ));
}

// ================================================================
// MODULE: coupler_ring_insert_plate
// ================================================================
//   Print plate: 12 half-ring inserts (2 halves per groove × 6 grooves).
//   All lying flat (arch in XY, ≈3.7mm print height). No supports needed.
//   Footprint: ~156mm × ~112mm — fits A1 Mini (180×180mm bed).
//   Layout: 3 columns × 4 rows, centred around X=0.
//
//   PRINT IN BLACK TOUGH+. After printing:
//     • CA-glue matching pairs at cut faces → full ring.
//     • Press ring into coupler groove while glue is wet, hold ~60s.
//     • Outer face will sit just below coupler OD for a recessed accent look.
// ================================================================

module coupler_ring_insert_plate() {
    r     = 25.0;
    clr   = 0.15;

    col_step = r*2 + 3;   // 53mm = piece diameter (50mm) + 3mm gap
    row_step = r   + 3;   // 28mm = arch depth (25mm) + 3mm gap

    // GRIP_N cols × 2 rows = GRIP_N*2 halves → GRIP_N full rings, one per groove.
    // Plate auto-scales with GRIP_N. Footprint: ~(GRIP_N*53)mm × ~53mm.
    for (i = [0 : GRIP_N*2 - 1]) {
        col = i % GRIP_N;
        row = floor(i / GRIP_N);
        translate([(col - floor(GRIP_N/2)) * col_step, row * row_step, 0])
            coupler_ring_insert_half(r=r, clr=clr);
    }
}

// ── V2 ASSEMBLY PREVIEW (visual only) ─────────────────────────
// Bodies are flush (no flange gap). Spigot of lower seg hides inside upper seg.
// seg_step = BLADE_H (bodies butt face-to-face, spigots hidden inside).
/*
translate([0,0, 0])          v2_blade_segment(seg1=true); // Seg 1 (retention ring at base)
translate([0,0, BLADE_H])    v2_blade_segment();           // Seg 2
translate([0,0, BLADE_H*2])  v2_blade_segment();           // Seg 3
translate([0,0, BLADE_H*3])  v2_blade_segment();           // Seg 4
translate([0,0, BLADE_H*4])  v2_tip_segment();             // Seg 5 (hemisphere tip)
*/

// ── FULL ASSEMBLY PREVIEW (visual only, not for printing) ──────
// end_cap_closed at bottom, handle, 5 blade segs stacked, end_cap_open at top junction
/*
ec_h = THREAD_TURNS * THREAD_PITCH + 8.0;  // endcap total height = 24mm
translate([0, 0, -ec_h])                    end_cap_closed();
translate([0, 0, 0])                        handle();
translate([0, 0, HANDLE_H - ec_h])         end_cap_open();
translate([0, 0, HANDLE_H])                blade_segment_with_bulkhead(39.67, 35.67, 35.33, 31.33, spigot_od=31.33, bayonet=true, notch_clr=1.5);
translate([0, 0, HANDLE_H + 150])          blade_segment_with_bulkhead(35.33, 31.33, 31.00, 27.00, spigot_od=27.00, notch_clr=1.7);
translate([0, 0, HANDLE_H + 300])          blade_segment_with_bulkhead(31.00, 27.00, 26.67, 22.67, spigot_od=22.67, notch_clr=2.0);
translate([0, 0, HANDLE_H + 450])          blade_segment_with_bulkhead(26.67, 22.67, 22.33, 18.33, spigot_od=18.33, notch_clr=2.3);
translate([0, 0, HANDLE_H + 600])          tip_segment(22.33, 18.33, 18.00, 14.00);
*/