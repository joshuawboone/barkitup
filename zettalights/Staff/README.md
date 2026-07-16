// V2 Staff (Modified open SCAD from Sabed design. Flat tip vs saber rounded, thicker segments up to 33mm from 24, slightly longer segments)  
//  
//    Uniform staff segments: 33mm OD / 12mm ID  
//    Threaded staff segment-to-segment joints  
//    Modular LED channel inserts (fit inside 12mm ID bore)  
//    Handle: threads both ends to fit 2 sets of staff segments via threaded caps  
// ================================================================  
//  
// PRINT NOTES:  
//   Full Bambu Studio project file included - bedded for my a1 mini (override references)  
//     
//   Root directory is a1 mini compatible - Handle is a bit short on this config. I had to outsource to a larger printer to get a 256 printed.  
//   Included 256, 320, and 340 bed size variants (LED lengths need verification, might be a difference of an LED shorter or longer)  
//  
// staff segs: (.4 Nozzle - adjust wall numbers to match on .6 nozzle if used)  
//   Tough+ white at - 3 wall - 10 gyroid infill - .18 layers - overrides on thread sections and base segment to make those areas 4 wall 100 infill for stability  
//     see bambu studio for reference - extend overrides into segment proper by 1-2mm for structure stability  
//  OR  
//   Transparent PETG at - 5 wall 40 gyroid - .18 layers  
//     bump temps to 260 nozzle - 70 bed for a little extra layer adhesion and durability of this material  
//  
// Tough+ is very light compared to the PETG more solid structure. PETG is a more brittle material and needs the higher settings to compare to tough+ durability  
//   (Drop test - rated both blades can survive a drop onto concrete from a second story - PETG has more shatter potential)  
//  
// LED Inserts:  
//   PLA basic white is basically the same as the transparent PETG in terms of light diffusion through it.  
//   Non-structural - LED insert guide channels  
//   LEDs - 2x strips of 39 60/m ws2812b (BTF Eco) - wired parallel to a JST connector for attaching to handle controller  
//  
// Handle, Coupler, Caps, Hangers:  
//   Tough+ at - 5 wall 30 gyroid - .24 layers (basically solid walls everywhere)  
//     Core structure pieces - Tough+ limited color options.   
//   Threaded cap - made inner wall speed 200/s to match outer wall speed due to some stringing in the threads (or set printer to 50% speed overall)  
//  
// ================================================================  
// PRINT LIST: (adjust numbers based on bed/size selection)  
//  
// 1x Handle - Threaded both ends - Same for all beds (A1 mini size is rather short for a handle. get it at 256 minimum - Chop the 340 with some kind of connector or glue)  
// 2x Cap - interior threads - slower speeds for smoothest threads  
// 2x LEDseg1 - Base LED segment - retention ring at base for handle sandwich  
// 6x LEDseg2-3-4 - a1 mini (4x of 256 size or 2x of 320/340)  
// 2x seg1 - Staff base segments - retention ring for handle sandwich  
// 4x seg2-3 - a1 mini (2x seg2 of 256 or no mid seg needed on 320/240 - seg 2 on 320/340 is tip segment)  
// 2x seg4 - a1 mini (2x seg3 of 256 - 2x seg2 of 320/340)  
