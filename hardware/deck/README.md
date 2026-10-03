# Platform deck for the RC car

The RoboRacer platform deck (`Platform_Deck_V3.1 (New Powerboard)` from the official parts
sheet's Google Drive folder) with four holes added for the RPLidar S2, so the lidar mounts
straight on the deck with no adapter plate.

| File | What |
|---|---|
| `Platform_Deck_V3.1_original.dxf` | the official file, untouched, for reference |
| `Platform_Deck_V3.1_S2.dxf` | the same plus the S2 holes: the file to cut |
| `Platform_Deck_V3.1_S2_laser_32x18in.pdf` | the cut file ready for the Jacobs Hall Universal lasers: a 32 x 18 in artboard, 1:1, red hairlines (cut), part 20 mm from the top-left corner |
| `Platform_Deck_V3.1_S2_check.png` | dimensioned check drawing |

## The S2 holes
Four 3.4 mm clearance holes (M3) on the S2's 41.7 x 41.7 mm pattern, centred on the deck's lidar
position (97, 0), the square turned 30 degrees so the holes clear the Hokuyo UST-10LX holes
(square to the deck they would overlap them by 1.2 mm). The Hokuyo UST and UTM patterns are kept
for a later lidar. The S2 body therefore sits turned 30 degrees; its cable exits 30 degrees off the
rear axis, and the lidar's yaw offset in the car's TF is 30 degrees.

Screws: 4 x M3 x 10 mm. The S2's holes are blind, 4 mm deep: through a 0.25 in (6.35 mm) deck a
10 mm screw engages 3.6 mm; do not use longer. The S2's scan plane is about 30 mm above the deck.

## Material and cutting (Jacobs Hall, Berkeley)
- 0.25 in cast acrylic, as the original drawing specifies (Jacobs store stock is laser safe).
  The parts sheet's Jetson screw length assumes a 0.12 in deck; at 0.25 in use M2.5 x 10 mm.
- Maker Pass laser training (module, video, quiz), reserve at reserve.jacobshall.org, buddy
  present, never leave the laser running. Acrylic is the flammable one: watch it.
- Print the PDF from Illustrator or Acrobat to the laser; in the Universal Control Panel pick
  the Jacobs store acrylic in the material database and enter the measured thickness. Test-cut
  a small circle in a corner first.
- The VLS 6.60 bed (room 110C) is 32 x 18 in; the PDF matches it. The part is 310 x 145 mm.
