# Building our RC car, step by step

This is the official RoboRacer build (f1tenth.readthedocs.io, "Building the RoboRacer Car")
written out for the exact parts we have, in order, with a check at the end of each step. Where
our car differs from the guide there are only three places, and they are marked **OURS**:
the motor swap, the RPLidar S2 on our own deck, and the Orin Nano in place of the Xavier NX.
Everything else is the guide's step, and the guide's photos still apply.

Nothing here is irreversible. The worst case at any step is undoing a screw.

## What you have

| Part | Where it goes |
|---|---|
| Traxxas Ford Fiesta ST Rally BL-2s (74154-4) | the chassis; its ESC, receiver and motor come out |
| Traxxas Velineon 3500 motor (3351R) | replaces the BL-2s motor |
| VESC 6 MkVI with heat sink | motor controller, on the deck |
| Jetson Orin Nano Super dev kit + NVMe SSD | the computer, on the deck |
| Ambimat RoboRacer power board | 19 V for the Jetson, 5 V and 12 V for the rest |
| RPLidar S2 + its USB adapter | on the deck, front |
| Intel RealSense D435if | front, later (3D-printed mount from the RoboRacer Drive folder) |
| Zeee 3S 6000 mAh batteries, B6AC charger, LiPo bag | under the deck |
| 8BitDo Ultimate 2C gamepad | your dead-man switch |
| Platform deck, our V3.1 with S2 holes, 0.25 in acrylic | `hardware/deck/` |
| Standoffs: 2 × 14 mm, 4 × 25 mm, 6 × 45 mm (M3); 4 × M2.5 10 mm | deck and Jetson |
| Screw assortment M2 to M5; antennas; DP dummy plug; cables and adapters | see the steps |

Tools: the hex keys from the Traxxas box, a small Phillips, a T3 Torx for the Jetson, callipers
or a ruler, zip ties, blue thread-locker (optional but worth it).

Three rules for the whole build: battery unplugged unless a step says otherwise; red to red and
black to black every single time; nothing near the driveshaft that runs down the middle.

---

## Session 1: the chassis (about an hour)

### 1.1 Body off
Pull the four body clips, lift the body off.
*Check:* you see the chassis tub with a blue box (steering servo) on one side, the ESC (speed
control) and receiver box, and the motor at the back.

### 1.2 Stock electronics out
Guide: "Removing Traxxas Stock Components". Unplug and unscrew the BL-2s ESC; open the receiver
box, unplug the servo lead and the ESC lead from the receiver, unscrew the box; pull the antenna
tube; remove the plastic battery hold-down. **Keep the servo and its lead**: it is the only
electrical part that stays.
*Check:* nothing electrical on the chassis but the servo, with its three-wire lead loose.

### 1.3 Motor swap  **OURS**
The BL-2s motor has a keyed Traxxas plug the VESC cannot take and is rated for 2S; the Velineon
3500 has three bullet leads and takes 3S. Same size can, same 3 mm shaft, so the small gear moves
across.

1. Take the gear cover off (one middle screw) so you can see the small gear on the motor shaft (the
   **pinion**) meshing with the big gear (the **spur**).
2. Loosen the pinion's set screw (tiny hex) and slide the pinion off the BL-2s shaft.
3. Undo the motor's mounting screws on the blue motor plate; lift the BL-2s motor out. Note which
   way its wires faced.
4. Fit the Velineon in its place, wires towards where the BL-2s wires were, screws loosely.
5. Slide the pinion onto the Velineon's shaft, set screw onto the shaft's flat, tighten.
6. **Gear mesh:** slide the motor so the pinion meets the spur with a strip of paper between the
   teeth; tighten the motor screws; pull the paper out. Turn a wheel by hand: smooth, no clicking,
   no binding.
7. Gear cover back on. Box the BL-2s motor and ESC as spares.

*Check:* pushing the car along the bench, the motor turns freely and quietly.

### 1.4 Standoffs on the chassis
Guide: "Attaching the Standoffs". Remove the nerf bars (four screws each, from underneath). Fit
**three 45 mm standoffs** into the chassis with M3 screws from below: two on the motor side, one on
the battery side. A drop of thread-locker on each.
*Check:* three posts standing up, about the deck's footprint apart.

### 1.5 Battery, dry fit only
Place a battery in the compartment opposite the motor, see that the deck will clear it. Do not plug
it in. Read the guide's LiPo warning once, properly: charge only in the bag, never leave one
plugged in, never short the leads, never plug in backwards.

---

## Session 2: the deck (about an hour)

Lay the deck on the bench: lidar end (the rounded lobe with the many small holes) **away** from you.
The deck is not symmetric; the two big M5 holes for the VESC sit off-centre, on the side nearer you.

### 2.1 VESC
Guide: "Mounting the VESC". VESC on top, its battery lead facing away, its three motor leads facing
you. Two **M5 × 20** screws from underneath, diagonal corners, into the case. Fit the heat sink on
top if its kit allows with the deck screws; otherwise leave it off until the VESC has run and you
know its temperature.
*Check:* VESC solid, no wobble, battery lead pointing to the lidar end.

### 2.2 Antenna mount
Guide: "Mounting the Antenna". The 3D-printed antenna mount on two **45 mm standoffs** behind the
VESC, with its two cables. The antennas themselves screw on last.

### 2.3 Jetson Orin Nano  **OURS, but the same as the guide**
The guide's NX photos apply: your Orin Nano dev kit has the same board size and the same four
mounting holes. What differs: it has a fan on top (leave it), and it needs its SSD.
1. Before mounting: fit the NVMe SSD into the M.2 slot underneath (one screw).
2. Clip the two antenna cables onto the Wi-Fi card's tiny connectors (press until they click).
3. Four **25 mm standoffs** on the deck's Jetson holes (M3 screw from underneath into each).
4. Jetson onto the standoffs, four **M2.5 × 10** screws through its corner holes. (10 mm, not the
   sheet's 6 mm, because our deck is 0.25 in.)
*Check:* Jetson level, fan clear, ports facing where cables can reach, DC jack reachable.

### 2.4 Power board
Guide: "Mounting the Powerboard". Three holes: M3 × 20 screws from underneath with 14 mm standoffs
so the board sits off the deck, green terminal blocks facing the lidar end. There is a gap between
it and the VESC.
*Check:* the board's two switches are reachable with the deck on the car.

### 2.5 RPLidar S2  **OURS**
Our deck has four holes for it (3.4 mm, in a 41.7 mm square turned 30 degrees). S2 on top,
centred on the lobe; four **M3 × 10** screws from underneath. Not longer: the S2's holes are 4 mm
deep. Its cable will exit 30 degrees off the rear line; that is expected.
*Check:* the S2 sits flat, can be spun by hand without touching anything, and nothing is in its
scan plane (about 30 mm above the deck) except the antennas, which are behind it.

---

## Session 3: deck onto chassis and wiring (about an hour)

### 3.1 Deck onto the chassis
Guide: "Mounting the Upper Level Chassis". VESC towards the back of the car. Pass the servo lead up
through a deck slot. Three **M3 × 10** screws into the three chassis standoffs.
*Check:* deck level, battery can still slide in and out, nothing touches the driveshaft.

### 3.2 Motor to VESC
Guide: "Connecting the Brushless Motor to the VESC". The three 4 mm-to-3.5 mm bullet adapters
onto the motor's three leads, then onto the VESC's leads: **A to white, B to yellow, C to blue**.
If the car later runs backwards, swap white and blue. Push the bullets fully home.

### 3.3 Servo to VESC
Guide: "Attaching the PPM Cable". Three header pins into the servo lead's plug; the Adafruit
JST-PH cable onto the pins, **brown/black to the servo's black (ground)**; JST end into the
VESC's PPM port.
*Check:* polarity. Ground to ground, twice.

### 3.4 Battery to VESC  **OURS**
The battery's big EC5 plug into the **EC5-male-to-XT90-female** adapter, adapter onto the VESC's
XT90. Not yet: this is the step where things start to move. Just see that it reaches.

### 3.5 Power board inputs and outputs  **OURS**
Per the board's own manual (v2024.1):
- Battery in: the battery's small white **balance plug**, through the balance extender, to the
  board's battery socket (J9). The board's switch SW1 set to battery.
- Jetson: the board's **19 V DC jack output (J7)** to the Jetson's DC jack with the male-to-male
  barrel cable you have. Centre pin is positive on both. (The guide's 12 V pigtail route is the old
  way and too weak for an Orin Nano; use J7.)
- S2: its USB adapter plugs into a Jetson USB port; it powers from USB. Nothing to the board.
- SW2 is the board's power switch. Off for now.
*Check:* read the two switch labels; the manual warns they are printed swapped on some boards.

### 3.6 USB
VESC micro-USB to a Jetson USB port. S2 adapter to a Jetson USB port. Gamepad dongle to a third.
Antennas screwed onto the mount. Zip-tie every cable away from the driveshaft and the wheels.
*Check:* turn each wheel and the driveshaft by hand with the cables tied: nothing catches.

---

## Session 4: software (an evening, mostly waiting)

Guide: "Configure Jetson and Peripherals", then "Install RoboRacer Driver Stack", then
"Configuring the VESC". Follow those pages; the only substitution is below.

1. **Flash the Jetson** with NVIDIA SDK Manager from a laptop (JetPack 6 for the Orin Nano). Boot
   it on a monitor once (the DP dummy plug is for running it headless later), connect Wi-Fi, note
   its IP, then SSH from your laptop.
2. **Driver stack:** the guide's ROS 2 Humble + `f1tenth_system` steps; its udev rules so the
   VESC and lidar get fixed device names. **OURS:** where the lidar section offers Hokuyo or
   SICK, we use `rplidar_ros` for the S2 instead (its README is two commands).
3. **VESC Tool**, on your laptop, VESC on USB, car on a stand with the wheels in the air, battery
   plugged in (this is the first time): update firmware, enable servo output and write it, load the
   guide's motor XML, run motor detection, then the guide's hysteresis and PID steps. The wheels
   will spin during detection; hold the car.
4. **Gamepad** paired to the Jetson. In the guide's "Driving the RoboRacer Car": one button held is
   the dead-man for manual, another for autonomous. Release and it stops.

*Check:* on the stand, holding the dead-man, the left stick turns the front wheels, the trigger
spins the rear wheels, letting go stops them. Then, and only then, the floor.

---

## First drive
A big empty floor, 1 m/s, the gamepad in your hand the whole time. Drive a few laps by hand. If it
runs backwards: swap the white and blue motor leads. If it steers the wrong way: the servo
direction flag in the driver config. Then `calibrate_camera` and sysid are the next chapters, in
their own documents.

## If you get stuck
Send a photo of the step and say which number you are on. There is nothing in this list that a
photo and five minutes cannot sort out.
