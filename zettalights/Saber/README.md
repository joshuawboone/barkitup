// V2 REDESIGN:	version 2.2
//
//    Uniform blade segments: 24mm OD / 12mm ID / 6mm wall
//    Threaded blade segment-to-segment joints
//    Modular LED channel inserts (fit inside 12mm ID bore)
//    Handle: threads blade-end, bayonet pommel-end, circuit trace cutout pattern, inserts to fill circuit traces, 
//    Flat endcaps, double-blade coupler, coupler ring inserts, wall and hook hangers
// ================================================================
//
// PRINT NOTES:
//   Full Bambu Studio project file included
//   Full OpenSCAD included
//
// Blade segs:
//   Tough+ white at - 3 wall - 0 infill - .24 layers - overrides on thread sections and base segment to make those areas 4 wall 100 infill for stability
//     see bambu studio for reference - extend overrides into segment proper by 1-2mm for structure stability
//  OR
//   Transparent PETG at - 5 wall 40 gyroid - .24 layers
//     bump temps to 260 nozzle - 70 bed for a little extra layer adhesion and durability of this material
// 
// Tough+ is very light compared to the PETG more solid structure. PETG is a more brittle material and needs the higher settings to compare to tough+ durability
//   (Drop test - rated both blades can survive a drop onto concrete from a second story - PETG has more shatter potential)
//
// LED Inserts: 
//   PLA basic white is basically the same as the transparent PETG in terms of light diffusion through it. 
//   Non-structural - LED insert guide channels
//   LEDs - 2x strips of 36 60/m ws2812b (BTF Eco) - wired parallel to a JST connector for attaching to handle controller
// 
// Handle, Coupler, Caps, Hangers:
//   Tough+ at - 5 wall 30 gyroid - .24 layers (basically solid walls everywhere)
//     Core structure pieces - Tough+ limited color options. Add contrast color with handle/coupler inserts. 
//   Threaded cap - made inner wall speed 200/s to match outer wall speed due to some stringing in the threads (or set printer to 50% speed overall) 
//
// Inserts Handle/Coupler:
//   PLA Basic for multiple color options
//   Finicky pieces - Handle inserts particularly: Legs are longer than they need to be for trimming at assembly time 
//     PLA Basic at - 2 wall - 15 infill grid - .08 Layer height (default .08 profile)
//   ADD: 
//     support overrides - normal manual - coupler inserts: snug - Handle inserts: default
//     brim - 15mm - inner and outer
//   (same profile for both handle and coupler inserts. check bambu studio file for support overrides.) 
// ================================================================

// PRINT LIST: 
// 
// Full Double Bladed Saber: 
// 2x /Blade/blade base.stl
// 4x /Blade/blade mid.stl
// 2x /Blade/blade tip.stl
//
// 2x /Blade/LEDseg1.stl
// 6x /Blade/LEDseg234.stl
//
// 1x /Handle/coupler.stl
// 12x /Handle/coupler-insert.stl
// 2x /Handle/handle.stl
// 2x /Handle/handle-insert-1.stl
// 2x /Handle/handle-insert-2.stl
// 2x /Handle/handle-insert-3.stl
// 2x /Handle/open cap.stl
// (optional for conversion to two single blades)
// 2x /Handle/closed cap.stl
//
// (optional handle without insert traces)
// /Handle/handle-noTraces.stl
//
// (optional hangers for cubicle/doors/wall)
// /Hangers/
//
// (single blade modifier - drop coupler and coupler inserts - half everything else - closed cap becomes non-optional)