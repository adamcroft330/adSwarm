# Hardware Platform Shortlist

Written 2026-06-30. Addresses tech doc §3 ("Hardware Platform — Re-Selection
Required" — Critical, blocks everything) and requirements doc §7 priority
action. Replaces the three-row sketch in tech doc §3.2 with researched
options, current pricing, and a dimension/budget reality check.

## Constraints this has to satisfy

- TOW ≤1.5 kg per drone (NFR-16)
- ≤250×250×250 mm including propeller guards (NFR-17)
- LiPo 4S maximum (NFR-18)
- Onboard compute only — ≥4 GB RAM, Jetson Orin Nano-class or QRB5165-class
  (NFR-08/09), co-resident with VIO + perception at ≥30 Hz (NFR-11)
- Mandatory propeller guards + hardware e-stop (FR-13/14)
- PX4 offboard mode or ArduPilot guided mode for the velocity-setpoint
  interface (RL pipeline §2.4)
- Budget: £10,000 covers the whole project, not just airframes — VIO
  (£190–400 for 4 units), UWB (£600–1,000 for 4, if adopted), batteries,
  spares, and contingency all come out of the same pot.

## Correction to the existing tech doc §3.2 shortlist

The first row of the existing shortlist — Holybro X500 V2 + Pixhawk 6C +
Jetson Orin Nano — is the standard PX4 development kit, but "X500" denotes
a ~500 mm motor-to-motor diagonal frame. That's roughly double the 250 mm
envelope and fails NFR-17 outright. It's a fine bench/integration
platform for early PX4 + companion-computer bring-up but should be dropped
as a competition-hardware candidate. The frame size bracket that actually
fits the rule is what FPV vendors call "250 class" — 5" prop frames,
~210–250 mm wheelbase — which is where all three options below sit.

## Options

### Option 1 — Custom 5" frame + Pixhawk FC + Jetson Orin Nano Super Dev Kit

Bespoke build: 5"-class ("250 class") frame, a lighter Pixhawk-family FC
(e.g. Pixhawk 6C Mini, ~$166 / £131), and an NVIDIA Jetson Orin Nano Super
Developer Kit (67 TOPS, 8 GB RAM, $249 / £196 — recently price-cut from
$499).

- Est. cost/unit: airframe + motors + ESCs + battery (~£150–250) + FC
  (~£131) + compute (~£196) ≈ **£500–650**, before VIO. ×4 ≈ £2,000–2,600.
- Pros: cheapest by a wide margin; leaves the most budget headroom for
  VIO/UWB/contingency; 5" is the natural fit for the 250 mm envelope;
  mature PX4 offboard-mode ecosystem.
- Cons: highest integration risk — frame selection, prop-guard fitment,
  and wiring FC + companion + VIO + ESCs together is fully DIY with no
  single vendor support. Matches the original tech doc's own risk note for
  this class of build. The dev kit is a full carrier board, bulkier than a
  production Orin Nano module — fine for bench/Stage 4–5 single-drone work,
  but mounting it cleanly inside a 250 mm envelope alongside everything
  else needs to be checked physically before committing.

### Option 2 — Holybro Pixhawk Jetson Baseboard + 5" frame

Holybro's integrated board puts a Pixhawk-class autopilot and a Jetson
Orin NX/Nano on one PCB, connected over UART/CAN/ethernet — removes the
FC↔companion wiring/integration problem Option 1 has to solve from
scratch. Pricing ranges from $396 / £312 (baseboard only, BYO Pixhawk) up
to ~$1,700 / £1,340 for a full bundle (Orin NX 16 GB + camera + SSD +
UBEC); a mid configuration (baseboard + Pixhawk 6X + Orin Nano 4 GB or
Orin NX 16 GB) likely lands somewhere in between — confirm the exact SKU
on Holybro's site before costing this precisely.

- Est. cost/unit: ~£450–850 for FC+compute, + frame/motors/ESCs/battery
  (~£150–250) ≈ **£600–1,100**, before VIO. ×4 ≈ £2,400–4,400 depending on
  RAM variant chosen.
- Pros: best balance of integration risk vs. budget — a vendor-supported,
  documented PX4 + ROS2 + Jetson combo, while still cheap enough to leave
  real budget headroom. Fits the 250 mm envelope on a 5" frame (to confirm
  once mounted).
- Cons: the 4 GB Orin Nano variant sits right at the NFR-08 floor
  (≥4 GB) with no margin for VIO + RL inference + perception running
  co-resident — the 16 GB Orin NX variant is the safer choice if budget
  allows, but costs more. No public footprint spec was found in this pass;
  physically verify it clears 250 mm with guards fitted on the chosen
  frame before buying in volume.

### Option 3 — ModalAI Starling 2 (purpose-built)

220 mm diagonal, 280 g, Qualcomm QRB5165 (15 TOPS), PX4 native, up to 5
onboard cameras for VIO/SLAM out of the box, IMU/barometer/GPS/WiFi
included, NDAA-compliant, assembled in the USA, available now. $2,949.99
/ unit (~£2,325 at current FX).

- Est. cost: $2,949.99 × 4 ≈ $11,800 (**~£9,300**) — this alone consumes
  essentially the entire £10,000 team budget, before VIO, UWB, batteries,
  spares, or contingency.
- Pros: by far the lowest integration risk — flight-ready, vendor-
  supported, PX4 and onboard VIO already working. Could materially de-risk
  Stage 5 hardware bring-up, which the RL pipeline doc already flags as
  the project's biggest schedule risk. Worth checking whether its native
  camera suite meets NFR-04 (≥30 Hz effective VIO) and could replace the
  separate Mighty Camera/OAK-D Lite purchase.
- Cons: budget-incompatible for all 4 units as-is. Only viable if the team
  either revisits the £10k figure or buys 1–2 units for early Stage 5
  single/pair-drone bring-up while running cheaper custom builds (Option 1
  or 2) for the rest — a real scope change from tech doc §1, needs
  explicit team sign-off, not just a build choice.

### Disqualified on dimensions

- **Holybro X500 V2** (~500 mm class) — fails NFR-17, see correction above.
- **ModalAI Starling 2 Max** (322 mm diagonal, 500 g, 500 g payload
  capacity, $2,999.99) — exceeds the 250 mm envelope despite being
  otherwise well-specified (55+ min flight, more payload margin for UWB
  etc.). Don't carry forward as a candidate.

## Comparison

| | Option 1: Custom 5" | Option 2: Holybro Baseboard | Option 3: Starling 2 |
|---|---|---|---|
| Est. cost (4 units, pre-VIO) | £2,000–2,600 | £2,400–4,400 | ~£9,300 |
| Integration risk | High | Medium | Low |
| Vendor support | None | Yes (Holybro) | Yes (ModalAI) |
| RAM headroom vs NFR-08 | Depends on module chosen | Tight (4GB) to safe (16GB) | TBC — spec not found |
| Native VIO | No — bolt-on Mighty/OAK-D | No — bolt-on Mighty/OAK-D | Yes — 5 onboard cameras |
| Budget fit (of £10k total) | Comfortable | Comfortable | Consumes nearly all of it |

## Recommendation framing (not a decision — for team sign-off)

Option 2 looks like the best default: meaningfully lower integration risk
than Option 1 for a modest cost premium, still leaves real budget room for
VIO/UWB/contingency, and stays inside the 250 mm envelope on a 5" frame.
Option 1 is the fallback if Option 2's measured footprint or RAM headroom
doesn't work out. Option 3 is worth keeping on the table only as a
1–2-unit Stage 5 bring-up aid, not as the 4-unit production choice, unless
the team explicitly revisits the budget split in tech doc §1.

## Sourcing risk (added 2026-06-30)

Holybro filed a "Declaration of Suspension of Overseas Export Shipment" in
June 2025; later reports (Aug–Sep 2025) narrow it to shipments specifically
to the USA (SpeedyBee hit the same restriction around the same time), cause
unconfirmed by either company. Separately, and more concretely: the
**Pixhawk Jetson Baseboard Bundle** that Option 2 depends on is currently
showing **Sold Out** directly on holybro.com. Don't treat Option 2 as a
guaranteed buy without checking live stock first.

Mitigations:
- Buy through a regional reseller rather than direct from Holybro/China —
  see holybro.com/pages/dealer. UK stockists found: Flying Tech, 3DXR.
- The FC half of Option 2 isn't actually single-vendor: Pixhawk 6C/6X is an
  open FMUv6X reference design. CUAV (Pixhawk V6X, X7+ Pro) and ARK
  Electronics (ARKV6X) make firmware-compatible alternative boards. The
  integrated Jetson baseboard itself is the one Holybro-exclusive part of
  Option 2 and the one currently out of stock — worth a fallback plan (e.g.
  separate Pixhawk + separate Jetson Orin Nano/NX carrier, closer to
  Option 1's wiring) if it doesn't come back in stock in time.

## Open items before this can close out tech doc §3

- Physically verify footprint (with prop guards mounted) for Option 2 on
  a candidate 5" frame — no public spec found for the baseboard's own
  dimensions in this pass.
- Decide 4 GB Orin Nano vs 16 GB Orin NX for Option 2 against NFR-08/11
  headroom.
- Get UK landed pricing (import duty/shipping) — all figures above are US
  list prices.
- Decide whether Option 3 gets budget as a Stage 5 de-risking aid.
- Once a platform is picked, this unblocks Stage 2 (dronelib.h BASE_*
  constants) — see `stage3_plan.md` for how that interacts with Stage 3
  sequencing.

## Sources

- [PX4 Development Kit - X500 v2 – Holybro Store](https://holybro.com/products/px4-development-kit-x500-v2)
- [Holybro X500 V2 + Pixhawk 6C frame reference – PX4 Guide](https://docs.px4.io/main/en/frames_multicopter/holybro_x500v2_pixhawk6c)
- [Pixhawk 6C – Holybro Store](https://holybro.com/products/pixhawk-6c)
- [NVIDIA Jetson Orin Nano Super Developer Kit](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/nano-super-developer-kit/)
- [NVIDIA Jetson Orin Nano Developer Kit Gets a "Super" Boost — NVIDIA Technical Blog](https://developer.nvidia.com/blog/nvidia-jetson-orin-nano-developer-kit-gets-a-super-boost/)
- [Pixhawk Jetson Baseboard Bundle – Holybro Store](https://holybro.com/products/pixhawk-jetson-baseboard)
- [Holybro Pixhawk Jetson Baseboard – PX4 Guide](https://docs.px4.io/main/en/companion_computer/holybro_pixhawk_jetson_baseboard)
- [VOXL 2 – ModalAI](https://www.modalai.com/products/voxl-2)
- [Starling 2 Indoor SLAM Development Drone – ModalAI](https://www.modalai.com/products/starling-2)
- [Starling 2 Datasheet – ModalAI Technical Docs](https://docs.modalai.com/starling-2-datasheet/)
- [Starling 2 Max GPS-denied Development Drone – ModalAI](https://www.modalai.com/products/starling-2-max)
- [Starling 2 Max Datasheet – ModalAI Technical Docs](https://docs.modalai.com/starling-2-max-datasheet/)
- [ModalAI Launches Next Generation Starling 2 and Starling 2 Max — Commercial UAV News](https://www.commercialuavnews.com/modalai-launches-next-generation-starling-2-and-starling-2-max-ndaa-compliant-development-drones)
- [5" Universal Ducted Propeller Guards – Stan FPV](https://stanfpv.com/products/5-injected-universal-ducted-propeller-guards-removable-top-optional)
- [Propeller Guards for the 250 Class Racer (5inch) – HobbyKing](https://hobbyking.com/en_us/propeller-guards-for-the-250-class-racer-5inch-set-of-4.html?___store=en_us)
