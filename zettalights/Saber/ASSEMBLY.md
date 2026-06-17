# ZL Lightsaber — Assembly Guide
**Double-Blade Configuration** | V2.2

---

## Parts List

![All parts laid out](photos/photo_1_2026-06-17_00-31-32.jpg)

**Per blade (×2):**
- 1× Blade base segment (Tough+ white)
- 3× Blade mid segments (Tough+ white)
- 1× Blade tip segment (Tough+ white)
- 1× LED base insert (PLA white, has flange/disc at one end)
- 6× LED channel inserts (PLA white, grooved rod sections)
- 1× LED strip — 36 LEDs, 60/m WS2812B, pre-soldered JST connector

**Handle assembly (×2):**
- 1× Handle body (Tough+ grey, purple trace inserts installed)
- 1× Open cap (Tough+ grey) — bayonet end
- 1× Closed cap (Tough+ grey) — alternate single-blade config only

**Coupler (×1):**
- 1× Coupler body (Tough+ grey, purple ring inserts installed)

> **Note:** The ruler in the parts photo is for scale — blade segments are ~220mm each, full assembled blade is ~880mm tip to base flange.

---

## Phase 1 — LED Insert Assembly

### Step 1 — Start with the LED base insert

![LED base insert with strip](photos/photo_29_2026-06-17_00-31-32.jpg)

The LED base insert has a flat disc/flange at one end. Thread the LED strip through the slot in the disc so the strip runs along the flat side of the insert rod. The JST connector should be on the disc/flange end — this will be the handle-side.

### Step 2 — Seat strip in channel groove

![Strip seated in channel](photos/photo_26_2026-06-17_00-31-32.jpg)

Each channel insert has a groove running its length. Lay the LED strip into this groove, LEDs facing outward. The strip should sit flush — it holds in place via the groove geometry, no adhesive needed for assembly.

### Step 3 — Stack all channel inserts

![Two inserts on strip](photos/photo_24_2026-06-17_00-31-32.jpg)

Continue adding the remaining 6 channel inserts end-to-end along the strip length. The inserts butt up against each other. Keep the LED faces all oriented the same direction (outward, toward the blade wall).

### Step 4 — Full LED rod assembly

![Full LED rod alongside blade shells](photos/photo_20_2026-06-17_00-31-32.jpg)

You should now have a complete LED rod: base insert (with JST flange) → 6 channel inserts → LED strip running the full length. Lay it alongside the blade shell segments to confirm it spans the full blade length. The JST connector hangs off the flange end.

---

## Phase 2 — Blade Shell Assembly

### Step 5 — Thread blade segments together

![Blade shells assembled](photos/photo_15_2026-06-17_00-31-32.jpg)

Screw the blade shell segments together in order:
1. Blade base (has the wider threaded collar)
2. Blade mid × 3
3. Blade tip

Hand-tight is sufficient — these are threaded to snug up firmly without tools. Orient the base segment so its open end (where the LED rod will insert) faces down.

### Step 6 — Insert LED rod into blade shell

![Blade assembly with JST at base](photos/photo_13_2026-06-17_00-31-32.jpg)

Slide the LED rod assembly into the assembled blade shell from the base end. Feed it tip-first. The base insert flange will seat against the open end of the base segment. The JST connector trails out the bottom.

> The LED rod should fit snugly inside the 12mm ID bore. If it catches, check that the channel inserts are all aligned in the groove.

---

## Phase 3 — Handle Connection

### Step 7 — JST connector closeup

![JST connector at blade base](photos/photo_8_2026-06-17_00-31-32.jpg)

With the blade assembled, the JST connector (4-wire: red, white, green, black) extends from the base flange. This connects to the matching JST socket inside the handle.

### Step 8 — Connect blade to handle

![JST being plugged in](photos/photo_6_2026-06-17_00-31-32.jpg)

Hold the handle with the threaded blade-end opening facing up. Feed the JST connector into the handle bore and mate it with the socket inside. The connector is keyed — it only goes in one way. Once connected, thread the blade base segment onto the handle. Hand-tight.

> **Polarity note:** The JST connector is keyed but double-check that it seats fully before threading the blade on — a partial connection can cause one color channel to drop out.

### Step 9 — Power on test

![First blade lit — single segment test](photos/photo_5_2026-06-17_00-31-32.jpg)
![Controller visible inside handle](photos/photo_3_2026-06-17_00-31-32.jpg)

Before finishing assembly, power on and verify the blade lights. The controller is seated inside the handle bore (visible through the open cap end). You should see all LEDs respond. Cycle through colors to confirm full RGB — if a color channel is missing, reseat the JST.

### Step 10 — Cap the handle

![Single blade final assembly lit](photos/photo_2_2026-06-17_00-31-32.jpg)

Thread the open cap onto the pommel end of the handle (bayonet fit — insert and twist). The charge port is accessible through the cap opening.

---

## Phase 4 — Double Blade Config

Repeat Phase 1–3 for the second blade. Both blades connect to opposite ends of the coupler:

1. Thread Handle A into one end of the coupler
2. Thread Handle B into the other end
3. Both JST connectors are internal — the coupler passes through

> **Single blade config:** Use a closed cap on the non-blade end of whichever handle you're using. Drop the coupler entirely.

---

## Notes

- **Blade orientation:** The LED strip groove faces toward the blade wall — LEDs point radially outward for best diffusion through the Tough+ shell.
- **Torque:** Hand-tight only on all threads. The Tough+ material has good thread strength but doesn't need tool torque.
- **Disassembly:** Reverse this guide. The blade unthreads from the handle, JST unplugs, LED rod slides out. No permanent connections.
- **Firmware:** Controller runs WS2812B protocol. USB-C port on controller for updates/charging depending on config.

---

*ZL Lightsaber V2.2 — Open Source | github.com/ZL*
