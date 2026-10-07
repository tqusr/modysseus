# Photo drop folder

Drop files here with the **exact names** below. The deck (`../index.html`) picks them up automatically on reload.
A missing file shows a dashed placeholder that names the file and the shot wanted, so you can see what is still open.

All photos get a cool, slightly desaturated treatment in CSS, so mixed phone shots sit together. You do not need to edit them.

## Style of shot (Einride look)

Cool and desaturated, dark or night, flash-lit against a dark floor. Symmetric, centred, square-on or low three-quarter angles.
Nothing cluttered: clear the floor, no cables, no posed smiling people. Landscape orientation. About 2000 px on the long edge, under 1 MB each.

## Expected files

| File | Slide | Frame (aspect) | Shot |
| --- | --- | --- | --- |
| `robot-hero.jpg` | 3 Meet the mini-truck | 1184 x 800 (about 3:2) | The mini-truck, three-quarter front view on a dark floor, one flash, truck centred. Lidar, camera and wheels visible. Then drag the numbered pins onto the parts (see below). |
| `apriltag-dock.jpg` | 10 Perception | 928 x 832 (about 8:7) | Close-up of a dock's AprilTag on the arena wall, flash-lit, truck nose just in frame. |
| `docking.jpg` | 13 Docking | 896 x 832 (about 1:1) | The truck mid-dock: nose to the dock, tag centred, low angle. A photo from the side or front works. |
| `gazebo.jpg` | 14 Sim to real (left) | 912 x 600 (3:2) | Screenshot of the arena in Gazebo with the truck and the dock tags. Crop out window chrome. |
| `robot-arena.jpg` | 14 Sim to real (right) | 912 x 600 (3:2) | The real 6 x 6 m arena: truck on the floor, docks on the walls, flash-lit. Match the camera angle of the Gazebo shot if you can. |
| `demo.mp4` | 16 Demo | 1384 x 778 (16:9) | Screen recording or phone video of one full delivery loop (route in, dock, load, deliver). Autoplays muted on the slide, with controls. |

Photos are cropped to fill their frame (`object-fit: cover`), so keep the subject near the middle.
The video is letterboxed to fit (`object-fit: contain`).

## Other formats

The deck looks for `.jpg` (and `.mp4` for the video). To use `.png`, `.webp` or `.mov` instead, either rename your file,
or open `../index.html` and replace the extension in the `src="photos/<name>.jpg"` line for that slot.
You can also use edit mode (press `E`) for text, but file extensions live in the HTML source.

## Aligning the callout pins (slide 3)

The numbered circles on `robot-hero.jpg` match the spec list on the left (1 Lidar, 2 Camera, 3 IMU, 4 Wheel encoders, 5 Compute).
Open the deck, press `E`, drag each pin onto the right part of your photo, then press `Ctrl+S`.
That downloads a new `index.html`: replace `presentation/index.html` with it.
(Pin positions are also autosaved in your browser as a draft, but only the saved file carries them to other machines.)
