// ================================================================
// V3 HANDLE — Standalone module file
//
// Extracted from lightsaber.scad. Contains all geometry needed for
// the v3 handle: threads, inverted-bayonet slots, PCB trace extrusions,
// insert helpers, and the main v2_handle() assembly module.
//
// USAGE: Uncomment one of the render lines at the bottom.
// ================================================================

$fn = 256;

// ── GLOBAL ──────────────────────────────────────────────────────
WALL          = 2.0;

// ── THREAD PARAMETERS ───────────────────────────────────────────
THREAD_PITCH  = 8.0;
THREAD_DEPTH  = 2.0;
THREAD_H      = 4.0;
THREAD_TURNS  = 3;
THREAD_CLR    = 0.55;
THREAD_STEPS  = 32;

// ── BAYONET PARAMETERS ──────────────────────────────────────────
BAYO_TABS     = 4;
BAYO_TAB_W    = 6.0;
BAYO_TAB_H    = 3.0;
BAYO_TAB_T    = 4.0;
BAYO_SLOT_CLR = 0.3;
BAYO_TWIST    = 45;

// ── INVERTED BAYONET ────────────────────────────────────────────
IBAYO_TAB_H    = 1.8;
IBAYO_GROOVE_Z = 8.0;

// ── V2 HANDLE DIMENSIONS ────────────────────────────────────────
V2_HDL_OD  = 44.0;
V2_HDL_ID  = 38.0;
V2_HDL_H   = 165.0;

V2_H_PITCH  = 6.0;
V2_H_DEPTH  = 1.5;
V2_H_TRDH   = 3.0;
V2_H_TURNS  = 3;
V2_H_CLR    = 0.45;

MIC_H    = 100; // Mic hole height on handle
BUTTON_H = 125; // Button hole height on handle

// ── BLADE SEGMENT PARAMETERS ────────────────────────────────────
BLADE_OD      = 24.0;  // blade body OD
BLADE_ID      = 12.0;  // LED channel bore
BLADE_H       = 300.0; // body height per segment

// Blade thread (spigot-in-bore)
B_PITCH    = 4.0;
B_DEPTH    = 1.0;
B_THREAD_H = 2.5;
B_TURNS    = 3;
B_CLR      = 0.40;
B_STEPS    = 32;
FEM_D      = 18.4 + 7;  // female bore diameter — balanced walls
CHAMFER_H  = 4.0;       // bore transition zone (FEM_D → BLADE_ID)

// Seg1 retention ring
BLADE_RING_OD  = 41.0;  // Seg1 ring OD — caught by endcap shelf
BLADE_RING_H   = 3.0;   // Seg1 ring axial height
SEG1_FILLET_R  = 2.0;   // ring-to-body fillet radius

// ── LED INSERT PARAMETERS ────────────────────────────────────────
LED_INS_OD = 11.75;  // insert outer rail diameter (clears 12mm bore)
LED_CHAN_W  = 10.0;  // LED channel opening width
LED_RAIL_H  = 1.5;  // rail height above channel face
LED_RAIL_T  = 1.0;  // rail wall thickness
LED_WEB_T   = 1.2;  // back web thickness
LED_TAB_W   = 2.0;  // alignment tab width
LED_TAB_H   = 1.5;  // alignment tab radial height

LED_SEG1_RING_OD = 41.0;  // LED seg1 insert ring OD (matches BLADE_RING_OD)

// ── RING DIMENSIONS (endcap pocket) ─────────────────────────────
V2_RING_OD    = 41.0;  // LED insert ring OD
V2_RING_H     = 3.0;   // LED insert ring axial height

// ── PCB TRACE PARAMETERS ────────────────────────────────────────
TRACE_CLR  = 0.0;
TRACE_D    = 3.6;   // protrusion depth (starts at handle ID, extrudes outward)
TRACE_W    = 1.6;
TRACE_WW   = 2.4;
TRACE_PAD  = 2.2;


PI = 3.14159265358979;


// ================================================================
// MODULE: thread_helix_ext
//   Fat external helical thread built from hulled radial slab pairs.
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
//   Fat internal thread groove — subtract from endcap bore for female thread.
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
}// ================================================================
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
}// ================================================================
// MODULE: ibayo_handle_slots
//   Subtracts 4× inverted-bayonet L-slots from handle exterior.
//   Each slot = axial keyway (z=0..groove_end) + circumferential lock arc.
// ================================================================
module ibayo_handle_slots() {
    r_od         = V2_HDL_OD / 2;
    groove_depth = IBAYO_TAB_H + BAYO_SLOT_CLR;
    groove_z     = IBAYO_GROOVE_Z;
    slot_t       = BAYO_TAB_T + BAYO_SLOT_CLR * 2;
    groove_end   = groove_z + slot_t;
    ang_key      = (BAYO_TAB_W + BAYO_SLOT_CLR*2) / r_od * (180/PI);

    for (i = [0:BAYO_TABS-1]) {
        rotate([0, 0, i * (360/BAYO_TABS)]) {
            // Axial keyway
            rotate([0,0, -ang_key/2])
                rotate_extrude(angle=ang_key)
                    translate([r_od - groove_depth, -0.01])
                        square([groove_depth + 0.1, groove_end + 0.02]);
            // Circumferential lock arc
            rotate([0,0, -ang_key/2])
                rotate_extrude(angle=BAYO_TWIST + ang_key)
                    translate([r_od - groove_depth, groove_z])
                        square([groove_depth + 0.1, slot_t]);
        }
    }
}


// ================================================================
// TRACE HELPERS — surface-extruded geometry (outside difference = protrusion)
// ================================================================

// Straight trace between two (angle, z) endpoints
module _bt(r, a0, z0, a1, z1, w=TRACE_W) {
    hull() {
        rotate([0,0,a0]) translate([r,0,z0]) rotate([0,90,0])
            cylinder(h=TRACE_D, r=w/2, $fn=8);
        rotate([0,0,a1]) translate([r,0,z1]) rotate([0,90,0])
            cylinder(h=TRACE_D, r=w/2, $fn=8);
    }
}
// Diagonal trace (stepped hull) between two (angle, z) endpoints
module _bdiag(r, a0, z0, a1, z1, w=TRACE_W, steps=10) {
    for (i=[0:steps-1]) {
        fa = i/steps;  fb = (i+1)/steps;
        hull() {
            rotate([0,0, a0 + (a1-a0)*fa]) translate([r, 0, z0 + (z1-z0)*fa])
                rotate([0,90,0]) cylinder(h=TRACE_D, r=w/2, $fn=8);
            rotate([0,0, a0 + (a1-a0)*fb]) translate([r, 0, z0 + (z1-z0)*fb])
                rotate([0,90,0]) cylinder(h=TRACE_D, r=w/2, $fn=8);
        }
    }
}

// Via pad disc on cylinder surface
module _bpad(r, a, z, pr=TRACE_PAD) {
    rotate([0,0,a]) translate([r,0,z]) rotate([0,90,0])
        cylinder(h=TRACE_D+0.1, r=pr, $fn=16);
}

// Bus ring arc (rotate_extrude). Full ring: a0=0, a1=360.
module _bring(r, z, a0=0, a1=360, w=TRACE_WW) {
    ch = 0.4;
    translate([0,0, z - w/2])
    rotate([0,0,a0])
        rotate_extrude(angle=a1-a0, $fn=128)
            translate([r,0])
                square([TRACE_D+0.1, w]);
}
// ================================================================
// MODULE: pcb_grip_traces
//   PCB trace pattern extruded onto handle surface.
//   r     = handle outer radius
//   z_bot = bottom of trace zone (above bottom thread zone)
//   z_top = top of trace zone (below top thread zone)
// ================================================================
module pcb_grip_traces(r, z_bot, z_top) {
    h = z_top - z_bot;

    // ── POWER BUS RINGS ──────────────────────────────────────────
    _bring(r, z_bot);
    _bring(r, z_top);
    _bring(r, z_bot + 0.48*h, 0, 180, TRACE_W+0.3);  // mid signal bus 0°→180°

    // ── PADS at major junctions ──────────────────────────────────
    for (a=[0,90,180,270]) _bpad(r, a, z_bot);
    for (a=[0,135,180,315]) _bpad(r, a, z_top);
    _bpad(r,   0, z_bot+0.48*h);
    _bpad(r,   0, z_bot+0.73*h);
    _bpad(r, 180, z_bot+0.48*h);
    _bpad(r,  90, z_bot+0.48*h);
    _bpad(r, 135, z_bot+0.48*h);
    _bpad(r, 270, z_bot+0.55*h);
    _bpad(r, 315, z_bot+0.70*h);

    // ── MAIN AXIAL RAILS ─────────────────────────────────────────
    _bt(r,   0, z_bot,   0, z_top);
    _bt(r, 180, z_bot, 180, z_top);

    // ── 90°→135° BRANCH ──────────────────────────────────────────
    _bt(r,   90, z_bot,        90, z_bot+0.48*h);
    _bt(r,  135, z_bot+0.48*h, 135, z_top);

    // ── 270°→315° BRANCH ─────────────────────────────────────────
    _bt(r, 270, z_bot,         270, z_bot+0.55*h);
    _bdiag(r, 270, z_bot+0.55*h, 315, z_bot+0.70*h);
    _bt(r, 315, z_bot+0.70*h,  315, z_top);

    // ── 45° UPPER STUB ───────────────────────────────────────────
    //_bring(r, z_bot+0.73*h, 0, 36, TRACE_W);
    //_bt(r, 45, 280.0, 45, z_top);
}
// ================================================================
// MODULE: v2_handle
//   Staff/saber handle — 44mm OD / 38mm ID / 320mm height.
//   External thread both ends (staff config). PCB trace extrusions
//   on outer surface within the grip zone (gz_bot..gz_top).
//   Mic pocket at z=120, button hole at z=280, both at -45°.
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
                        translate([0, 13.0, MIC_H])
                            rotate([-90, 0, 0])
                                cylinder(h = 8.4, d = 11.0);
                        translate([0, 13.0, MIC_H + 5.5])
                            rotate([-90, 0, 0])
                                cylinder(h = 8.4, d = 0.1);
                    }
                    // ── BUTTON HOLE — teardrop ────────────────────────
                    // y_start = ID/2 - d/2 - 1 = 14.5mm; h = OD/2 + 0.2 - 14.5 = 7.7mm.
                    hull() {
                        translate([0, 14.5, BUTTON_H])
                            rotate([-90, 0, 0])
                                cylinder(h = 7.7, d = 8.0);
                        translate([0, 14.5, BUTTON_H + 4])
                            rotate([-90, 0, 0])
                                cylinder(h = 7.7, d = 0.1);
                    }
                }
                // ── PCB TRACE GRIP TEXTURE (indented) ─────────────────
                //pcb_grip_traces(r=V2_HDL_OD/2, z_bot=gz_bot, z_top=gz_top);
            }
        }
        // ── PCB TRACE GRIP TEXTURE (indented) ─────────────────
                //pcb_grip_traces(r=V2_HDL_ID/2+.05, z_bot=gz_bot, z_top=gz_top);

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
//   Through-hole passes blade Seg 1 body (BLADE_OD).
//
//   Z stack (open/bottom face down):
//     0..entry_collar          : plain socket, clears handle thread tips
//     entry_collar..+thread_h  : threaded bore, mates with handle ext thread
//     +ring_pocket_h           : smooth pocket for LED ring + blade retention ring
//     +collar_h                : blade through-hole (blade_pass_d)
//
//   PRINT ORIENTATION: open end down (z=0 on build plate)
// ================================================================
module v2_end_cap_open() {
    bore_d        = V2_HDL_OD + 2 * V2_H_CLR;               // 44.9mm — fit bore over handle OD
    ec_od         = bore_d + 2 * (V2_H_DEPTH + WALL) + 0.6; // 51.9mm — endcap OD
    thread_h      = V2_H_TURNS * V2_H_PITCH;                 // 18mm — threaded zone
    ring_pocket_h = BLADE_RING_H + V2_RING_H;                // 6mm — dual ring pocket
    collar_h      = 6.0;
    entry_collar  = 3.0;                                     // plain socket before threads
    blade_pass_d  = BLADE_OD + 0.8;                          // blade through-hole with clearance
    r_groove      = bore_d / 2 - 0.1;

    difference() {
        union() {
            cylinder(h=entry_collar + thread_h, d=ec_od);
            translate([0,0, entry_collar + thread_h])
                cylinder(h=ring_pocket_h + collar_h, d=ec_od);
        }
        // Entry collar: wider bore clears handle thread tips (no engagement)
        translate([0,0,-0.1])
            cylinder(h=entry_collar + 0.1, d=V2_HDL_OD + 2*V2_H_DEPTH + 1.0);
        // Threaded bore + ring pocket
        translate([0,0, entry_collar - 0.1])
            cylinder(h=thread_h + ring_pocket_h + 0.1, d=bore_d);
        // Blade through-hole above shelf
        translate([0,0, entry_collar + thread_h + ring_pocket_h - 0.1])
            cylinder(h=collar_h + 0.2, d=blade_pass_d);
        // Internal thread groove — clipped to thread zone
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
// MODULE: v2_blade_segment
//   Uniform hollow blade segment: BLADE_OD / BLADE_ID / BLADE_H.
//   seg1=false : female thread at bottom + spigot at top (mid segments)
//   seg1=true  : straight LED bore throughout + retention ring at base
//
//   PRINT ORIENTATION: bottom down (z=0 on build plate)
// ================================================================
module v2_blade_segment(seg1=false) {
    fem_h     = B_TURNS * B_PITCH;
    spigot_h  = fem_h;
    spigot_d  = FEM_D - 2 * B_CLR;
    r_spigot  = spigot_d / 2 - 0.1;
    r_bore    = FEM_D / 2 - 0.1;
    clip_d    = spigot_d + 2 * (B_DEPTH + 1);

    union() {
        difference() {
            cylinder(h=BLADE_H, d=BLADE_OD);

            if (seg1) {
                translate([0,0,-0.1])
                    cylinder(h=BLADE_H + 0.2, d=BLADE_ID);
            } else {
                // Female bore zone
                translate([0,0,-0.1])
                    cylinder(h=fem_h + 0.1, d=FEM_D);
                // Chamfer transition FEM_D → BLADE_ID
                translate([0,0, fem_h])
                    cylinder(h=CHAMFER_H + 0.1, r1=FEM_D/2, r2=BLADE_ID/2);
                // LED bore
                translate([0,0, fem_h + CHAMFER_H - 0.1])
                    cylinder(h=BLADE_H - fem_h - CHAMFER_H + 0.2, d=BLADE_ID);
                // Female thread groove
                intersection() {
                    translate([0,0,-B_PITCH])
                        thread_helix_int_cut(r_bore=r_bore, n_turns=B_TURNS+1,
                                             pitch=B_PITCH, depth=B_DEPTH,
                                             thread_h=B_THREAD_H, steps=B_STEPS);
                    cylinder(h=fem_h, r=100);
                }
                // Bore-to-chamfer fillet — reduces stress concentration at thread/chamfer junction
                rotate_extrude($fn=128)
                    translate([FEM_D/2 - 1.5, fem_h])
                        circle(r=1.5, $fn=32);
            }
        }

        // Spigot (non-seg1 only)
        if (!seg1) {
            translate([0,0, BLADE_H]) {
                difference() {
                    cylinder(h=spigot_h, d=spigot_d);
                    translate([0,0,-0.1])
                        cylinder(h=spigot_h + 0.2, d=BLADE_ID);
                }
                intersection() {
                    translate([0,0,-B_PITCH])
                        thread_helix_ext(r=r_spigot, n_turns=B_TURNS+1,
                                          pitch=B_PITCH, depth=B_DEPTH,
                                          thread_h=B_THREAD_H, steps=B_STEPS);
                    cylinder(h=spigot_h, d=clip_d);
                }
            }
        }

        // Seg1 retention ring + fillet
        if (seg1) {
            difference() {
                cylinder(h=BLADE_RING_H, d=BLADE_RING_OD);
                translate([0,0,-0.1])
                    cylinder(h=BLADE_RING_H + 0.2, d=BLADE_ID);
            }
            rotate_extrude($fn=128)
                translate([BLADE_OD/2 + SEG1_FILLET_R - 2.0, BLADE_RING_H + SEG1_FILLET_R - 2.0])
                    circle(r=SEG1_FILLET_R, $fn=64);
        }
    }
}


// ================================================================
// MODULE: v2_tip_segment
//   Same as v2_blade_segment (female bottom) but with flat cylinder
//   tip at top instead of spigot. No LED bore in tip zone.
//   PRINT ORIENTATION: bottom down
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
//   (|) profile LED channel insert for BLADE_ID (12mm) bore.
//   Cross-section: thin ring + center spine with diagonal access notches.
//   PRINT ORIENTATION: flat on back face, no supports needed.
// ================================================================
module v2_led_channel_insert(ins_h=BLADE_H) {
    r_out   = LED_INS_OD / 2;
    arc_t   = 0.6;
    r_inner = r_out - arc_t;
    spine_t = LED_WEB_T;

    notch_w = 2.0;
    notch_y = r_inner * 0.85;
    notch_h = r_out - notch_y + 0.1;

    linear_extrude(height=ins_h) {
        difference() {
            union() {
                difference() {
                    circle(r=r_out);
                    circle(r=r_inner);
                }
                translate([-spine_t/2, -r_inner])
                    square([spine_t, r_inner * 2]);
            }
            translate([ spine_t/2,              notch_y])              square([notch_w, notch_h]);
            translate([-(spine_t/2 + notch_w), -(notch_y + notch_h)]) square([notch_w, notch_h]);
        }
    }
}


// ================================================================
// MODULE: v2_led_channel_seg1
//   LED channel insert for Seg 1 position — same cross-section as
//   v2_led_channel_insert but with a retention ring at the bottom.
//   Ring is caught by endcap shelf to lock the LED stack axially.
//   PRINT ORIENTATION: ring-end down
// ================================================================
module v2_led_channel_seg1() {
    union() {
        v2_led_channel_insert(ins_h=V2_RING_H + BLADE_H);
        difference() {
            cylinder(h=V2_RING_H, d=LED_SEG1_RING_OD);
            translate([0,0,-0.1])
                cylinder(h=V2_RING_H + 0.2, d=LED_INS_OD);
        }
    }
}


// ── COUPLER PARAMETERS ──────────────────────────────────────────
COUPLER_H   = 180.0;  // mm — total coupler length
GRIP_N      = 3;      // number of grip rings
GRIP_WIDTH  = 10.0;   // mm — axial width of each ring
GRIP_DEPTH  = 1.2;    // mm — ring protrusion (radial)
GRIP_CHAMF  = 3.0;    // mm — chamfer on each side of ring


// ================================================================
// MODULE: coupler_grip_rings
//   Circumferential raised rings for coupler grip texture.
//   n evenly-spaced rings between z_bot and z_top, extruded outward.
// ================================================================
module coupler_grip_rings(r, z_bot, z_top, n=GRIP_N, depth=GRIP_DEPTH, width=GRIP_WIDTH) {
    chamf = GRIP_CHAMF;
    step = (z_top - z_bot) / (n + 1);
    for (i = [1:n]) {
        translate([0, 0, z_bot + i*step - width/2])
            rotate_extrude($fn=128)
                polygon([
                    [r,         0            ],
                    [r,         width        ],
                    [r + depth, width - chamf],
                    [r + depth, chamf        ],
                ]);
    }
}


// ================================================================
// MODULE: coupler_grip_rings_only
//   Rings-only body for dual-color export (second STL for AMS/dual nozzle).
// ================================================================
module coupler_grip_rings_only() {
    half   = COUPLER_H / 2;
    cap_od = 50.0;
    coupler_grip_rings(r=cap_od/2, z_bot=8.0, z_top=half - 3.0);
    translate([0,0, COUPLER_H]) mirror([0,0,1])
        coupler_grip_rings(r=cap_od/2, z_bot=8.0, z_top=half - 3.0);
}


// ================================================================
// MODULE: v2_double_blade_coupler
//   180mm tube connecting two handles for double-blade config.
//   Inverted bayonet: smooth bore + 4× inward tab lugs at each end.
//   Handles insert and twist 45° to lock.
//   PRINT ORIENTATION: either end down ✓
// ================================================================
module v2_double_blade_coupler() {
    cap_od        = 50.0;
    fit_clr       = 0.15;
    engage_bore   = V2_HDL_OD + 2*fit_clr + 1;
    half          = COUPLER_H / 2;
    entry_chamf   = 1.5;
    lug_z_coupler = half - IBAYO_GROOVE_Z - BAYO_TAB_T - BAYO_SLOT_CLR;
    lug_h         = engage_bore/2 - V2_HDL_OD/2 + IBAYO_TAB_H - fit_clr;

    union() {
        difference() {
            cylinder(h=COUPLER_H, d=cap_od);
            union() {
                translate([0,0,-0.1])
                    cylinder(h=COUPLER_H+0.2, d=engage_bore);
                translate([0,0,-0.1])
                    cylinder(h=entry_chamf+0.1,
                             r1=engage_bore/2+entry_chamf, r2=engage_bore/2);
                translate([0,0, COUPLER_H - entry_chamf])
                    cylinder(h=entry_chamf+0.1,
                             r1=engage_bore/2, r2=engage_bore/2+entry_chamf);
            }
        }

        coupler_grip_rings(r=cap_od/2, z_bot=8.0, z_top=half - 3.0);
        translate([0,0, COUPLER_H]) mirror([0,0,1])
            coupler_grip_rings(r=cap_od/2, z_bot=8.0, z_top=half - 3.0);

        ibayo_int_tabs_ring(engage_bore=engage_bore, lug_z=lug_z_coupler, lug_h=lug_h);
        translate([0,0, COUPLER_H]) mirror([0,0,1])
            ibayo_int_tabs_ring(engage_bore=engage_bore, lug_z=lug_z_coupler, lug_h=lug_h);
    }
}


// ================================================================
// MODULE: v2_end_cap_closed
//   Pommel cap — inverted bayonet receiver.
//   Smooth bore slides over handle OD. 4× inward tab lugs engage
//   the external L-slots on the handle. Insert → push → twist 45° to lock.
//   PRINT ORIENTATION: open end down (z=0 on build plate) ✓
// ================================================================
module v2_end_cap_closed() {
    cap_od      = 50.0;
    fit_clr     = 0.15;
    engage_bore = V2_HDL_OD + 2*fit_clr;
    bore_depth  = 15.0;
    solid_h     = 10.0;
    total_h     = bore_depth + solid_h;
    entry_chamf = 1.5;
    lug_z       = bore_depth - IBAYO_GROOVE_Z - BAYO_TAB_T - BAYO_SLOT_CLR;
    lug_h       = engage_bore/2 - V2_HDL_OD/2 + IBAYO_TAB_H - fit_clr;

    union() {
        difference() {
            cylinder(h=total_h, d=cap_od);
            union() {
                translate([0,0,-0.1])
                    cylinder(h=bore_depth + 0.1, d=engage_bore);
                translate([0,0,-0.1])
                    cylinder(h=entry_chamf + 0.1,
                             r1=engage_bore/2 + entry_chamf,
                             r2=engage_bore/2);
            }
        }
        ibayo_int_tabs_ring(engage_bore=engage_bore, lug_z=lug_z, lug_h=lug_h);
    }
}


// ================================================================
// RENDER —
// ================================================================
//v2_handle();

// Handle traces - Comment/Uncomment this line to add/remove trace pattern on handle
//pcb_grip_traces(r=V2_HDL_ID/2+.05, z_bot=20, z_top=140);

// v2_blade_segment(seg1=true); // Seg 1 only — adds retention ring at base
// v2_blade_segment();          // Seg 2 (used for smaller pritners - more segments)
v2_tip_segment();            // Seg 3 (top of blade — flat tip)
// v2_led_channel_insert();     // LED channel insert section (1 per blade segment, Seg 2–3)
// v2_led_channel_seg1();       // LED channel insert with retention ring (Seg 1 only)
// v2_double_blade_coupler();   // Coupler body (grey)
// coupler_grip_rings_only();   // Coupler rings only — second color STL
// v2_end_cap_closed();         // Pommel cap (bayonet receiver)
// v2_end_cap_open();              // Blade side cap for retention - threaded