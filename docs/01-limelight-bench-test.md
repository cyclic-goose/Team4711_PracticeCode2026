# 01: Fixing Limelight tag detection (on a desk, no robot)

## What's going on

A **Limelight 4** has a 1280×800 global-shutter camera and should detect a full-size, properly
printed FRC tag reliably at **15–20+ feet**. Losing the tag about half the time at 6–8 ft,
**with the camera sitting still**, is not normal. It also isn't something PID tuning or code can
fix. Something is making the tag hard to read:

| Likely cause | Why it fits what you saw | How to check |
| --- | --- | --- |
| **The tag itself.** Wrong size, wrong family, no white border, glossy or wrinkled paper | "Randomly printed by someone on standard paper." Drops even when still. | Print a proper tag (below) and compare at the same distance |
| **Room lighting flicker** | Classroom LED/fluorescent lights pulse 120×/s. With a short exposure, some frames come out darker. Gain "helping a bit" fits this. | Compare lights on vs off/daylight; look for brightness pulsing in the video |
| **Settings working against each other** | High detector downscale, very short exposure, high gain (noisy) | Use the baseline settings below and change one thing at a time |
| **Lens** | Protective film, smudges, or something in front of it | Look at it closely; clean it with a lens cloth |
| **Old firmware** | Detection has improved a lot over the years | Update with the Limelight Hardware Manager |

The fix is to **measure, change one thing, measure again**. The script in `tools/` does the
measuring for you.

## What you need

- The Limelight 4, an Ethernet cable, and power: a 12 V supply on its power input, **or** a PoE
  injector.
- A laptop.
- **New printed tags** (next section).
- A tape measure.

## Step 1: Print proper tags

The tag that gave trouble was "tag 1, printed on standard paper." Print new ones like this:

1. Get the official image. WPILib and FIRST both publish the **36h11** family as a PDF with one
   tag per page (search "FRC AprilTag 36h11 PDF" or see the AprilTag page in the WPILib docs).
   Print **ID 10 and ID 9**, plus **ID 26 and ID 25** if you can. Those are the hub-face tags; the
   robot section explains why.
2. Print at **100% / Actual size**. Turn **off** "fit to page" / "scale to fit".
3. **Measure the black square.** It must be **6.5 in (165.1 mm)** edge to edge. If it isn't, the
   printer scaled it. FRC used 6.5 in 36h11 tags in 2024–2026; check the field section of the
   game manual if in doubt.
4. Use **matte** paper or cardstock. No lamination, no glossy photo paper: glare off the ceiling
   lights wipes out parts of the tag.
5. Glue it **flat** onto cardboard or foam board. Don't tape it to a wall where it can curl or
   ripple.
6. **Keep the white border** around the black square. At least 1 inch, more is better. Don't
   trim it off, and don't mount it on a dark background with no border. The detector needs white
   all the way around the black edge.

Save the old paper tag and compare it with the new one in Step 5. That comparison will probably
be the answer.

## Step 2: Update and connect

1. Install the **Limelight Hardware Manager** and update the Limelight to the newest firmware
   (LLOS 2026.x or newer).
2. Plug it into your laptop. Open `http://limelight.local:5801`. If that doesn't load, use its IP.
   On the robot it should be `10.47.11.11`; you set that under Settings later.
3. Make sure you're editing the pipeline that is **active** (shown at the top of the UI).

## Step 3: Baseline settings

Start from these, then change **one thing at a time**:

| Setting | Baseline | Notes |
| --- | --- | --- |
| Pipeline type | AprilTag / Fiducial | |
| Family | **36h11** | FRC 2024–2026. The old 16h5 family won't detect 36h11 at all. |
| Tag size | **165.1 mm** | Only affects distance/3D math, not detection, but keep it right |
| Resolution | **1280×800** (full sensor) | Lower resolution = fewer pixels on the tag = shorter range |
| Detector downscale | **1** for these tests | Each step up roughly halves the pixels the detector uses. Try 1.5–2 later if FPS is too low. |
| LEDs | Off | AprilTags don't need them |
| Exposure | As **low** as possible while the tag still looks crisp black-on-white | Low exposure = less motion blur when the robot turns |
| Gain | Raise until the image is bright enough at that exposure | Very high gain adds noise (speckle), which also hurts |
| Black level | Start at 0 | Raising it darkens the background; try small values later |
| ID filters | **None** | An ID filter silently ignores other tags |

If your firmware has a quality / decision-margin threshold, leave it at the default for now.

## Step 4: Measure with the script

Put the tag on a wall at about the same height as the camera, facing it straight on. Put the
camera on a box. **Don't touch either during a run.**

From PowerShell in the project folder:

```powershell
cd tools
.\Test-LimelightDetection.ps1 -Label "6ft new tag baseline"
```

It watches for 30 seconds and prints the **detection rate** (what % of camera frames saw a tag),
the longest dropout, the tag area, and the FPS. Each run is saved to `tools/bench-results.csv`, so
you can compare runs in Excel.

> Windows may block the script the first time. If so, run
> `powershell -ExecutionPolicy Bypass -File .\Test-LimelightDetection.ps1 -Label "..."`.
> Use `-Address 10.47.11.11` if `limelight.local` doesn't resolve.

**What good looks like:** close to **100%** at 3, 6, 10 and 15 ft with a still camera and tag.

## Step 5: Find the culprit (one change per run)

Run the script for each line and label it clearly:

1. **New tag vs old tag**, same spot, 6 ft. A big difference means the old tag was the problem.
2. **Distance sweep** with the new tag: 3, 6, 10, 15, 20 ft. Note where the rate starts falling.
3. **Lighting:** room lights on vs off (daylight or a desk lamp). If the rate jumps with the lights
   off, it's **flicker**. Fixes: an exposure near **8.3 ms** (one flicker cycle) if the image
   allows it, steadier light, or angling the tag so ceiling lights don't reflect off it.
4. **Exposure / gain:** a couple of combinations at your worst working distance.
5. **Downscale:** 1 vs 2 at your worst working distance. Note the FPS change too.
6. **Motion:** walk the tag slowly side to side at 8 ft while running the script. If the rate drops
   only while moving, lower the exposure (blur).

Write the winners into a note. Those are your settings.

## Step 6: On the robot

1. **Power:** your notes say the Limelight runs off the VRM's 12 V / 500 mA output. Follow the
   Limelight 4 wiring docs instead: a PDH channel with a small breaker, or PoE. It looked stable,
   but a camera that's short on power can reset or behave oddly under load.
2. **Static IP** `10.47.11.11`, team number 4711, name `limelight`.
3. **Field map:** make sure the Limelight is using the **2026 field map** (welded). MegaTag
   needs it to know where the tags are.
4. **Robot-space camera pose** (the Limelight settings for where the camera is on the robot):
   measure from the **center of the robot, at floor level**: forward, sideways and up distance,
   plus the tilt (about 20° up for you). Enter it in the Limelight UI and check it in the
   Limelight's 3D view, which draws the camera on a robot. **Check the sign conventions on
   Limelight's docs page.** WPILib calls "tilted up" a *negative* pitch, and the Limelight UI may
   not. Then put the same numbers in `VisionConstants.robotToCamera0` (WPILib convention) for
   the simulator.
5. **Verify with the robot:** park the robot at a measured spot in front of the tags and compare
   `Odometry/Robot` and `Vision/Camera0/RobotPosesAccepted` in AdvantageScope with the tape
   measure. Being off by a few inches is fine; a foot or more means the camera pose or tag
   placement is wrong.

## Tools to use during robot sessions

- **Back button** on the controller saves a **snapshot** and the **last 20 seconds of video**
  (Limelight 4 "Rewind"). The moment the tag drops, press Back. Later, open the captures in the
  Limelight web UI and *see* what the camera saw (blur? glare? tag out of frame?).
- **`Vision/Camera0/DetectionRatePercent`** in AdvantageScope: the same measurement as the bench
  script, recorded for the whole session.
- **`Vision/Camera0/MeasuredFPS`**: frames actually reaching the robot code.

## Testing the field-position features in a classroom

The robot's field position needs tags at **known field positions**. You don't need a field. Tape
up a hub face and the robot will believe it's standing in front of that hub:

**Red hub face (tags 10 and 9):**

1. Mount **tag 10** flat on a wall with its **center 44.25 in** off the floor.
2. Mount **tag 9** at the same height, with its center **14.0 in to the LEFT** of tag 10's center
   (left as you face the wall).
3. Set the Driver Station alliance to **Red**. The DS defaults to Red 1, which is convenient.

**Blue hub face (tags 26 and 25):** same layout with **26** in place of 10 and **25** in place
of 9, and the Driver Station set to **Blue**.

The hub center is about 0.6 m (24 in) *behind* the wall, directly behind tag 10 (or 26). Level 2
aiming points at that spot. The field ends 4.0 m (13 ft) in front of the hub face, so pose
estimates from farther back than that are thrown out as "off the field". Stay within about
12 ft of the wall.
