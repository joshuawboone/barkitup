// ================================================================
// SABER HANGER — OpenSCAD Model
// ================================================================

$fn = 128;

// ── PARAMETERS ──────────────────────────────────────────────────

ARM_W     = 50.0;   // mm — arm width
ARM_L     = 175.0;  // mm — arm length
THICKNESS =  6.0;   // mm — thickness (Z)

FORK_W    = 12.0;   // mm — tine width in X
FORK_Y    =  8.0;   // mm — tine depth in Y
FORK_Z    = 50.0;   // mm — tine height in Z

HOOK_Y    =  6.0;   // mm — hook depth in Y
HOOK_Z    = 63.0;   // mm — hook drop in -Z

BRACE_Y   = 75.0;   // mm — how far back along arm brace reaches
BRACE_Z_TOP = 56.0; // mm — Z depth where brace tops out

R         =  3.0;   // mm — corner radius (all edges)

// ── HELPER: rounded flat slab (rounds vertical edges, flat top/bottom) ──
module rslab(x, y, z, r=R) {
    minkowski() {
        cube([x - 2*r, y - 2*r, z]);
        cylinder(r=r, h=0.01, $fn=128);
    }
}

// ── HELPER: tine with rounded tip ───────────────────────────────
module tine(x, y, z, r=R) {
    hull() {
        // Base (flat, at z=0)
        cube([x, y, 0.01]);
        // Tip (rounded cap at z=FORK_Z)
        translate([x/2, y/2, z])
            sphere(r=min(x,y)/2, $fn=128);
    }
}

// ================================================================
// GEOMETRY
// ================================================================

SCREW_D  =  4.0;   // mm — screw hole diameter (M3 clearance)
SCREW_Y  = 30.0;   // mm — screw hole inset from each end

// Arm (with screw holes)
difference() {
    cube([ARM_W, ARM_L, THICKNESS]);
    // Near-end hole
    //translate([ARM_W/2, SCREW_Y, -0.1])
      //  cylinder(d=SCREW_D, h=THICKNESS + 0.2, $fn=32);
    // Far-end hole (halfway to avoid hook zone)
    //translate([ARM_W/2, ARM_L/2, -0.1])
      //  cylinder(d=SCREW_D, h=THICKNESS + 0.2, $fn=32);
}

// Left tine
tine(FORK_W, FORK_Y, FORK_Z);

// Right tine
translate([ARM_W - FORK_W, 0, 0]) tine(FORK_W, FORK_Y, FORK_Z);

// Hook drop
translate([0, ARM_L - HOOK_Y, -HOOK_Z])
    cube([ARM_W, HOOK_Y, HOOK_Z]);

// Diagonal brace
hull() {
    translate([0, ARM_L - BRACE_Y, -BRACE_Z_TOP + THICKNESS/2])
        rotate([0, 90, 0])
            cylinder(h=ARM_W, r=THICKNESS/2, $fn=128);
    translate([0, ARM_L - HOOK_Y, -HOOK_Z])
        cube([ARM_W, HOOK_Y, THICKNESS]);
}
