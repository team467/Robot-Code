# Stereo fuel table test

This runs only the stereo test on your Windows laptop, without a roboRIO. Robot camera mounting constants remain unchanged.

## Before connecting hardware

Open PowerShell in this project. Use the WPILib JDK if your normal Java version cannot run Gradle:

```powershell
$env:JAVA_HOME = 'C:\Users\Public\wpilib\2026\jdk'
.\gradlew.bat stereoTableTest --args="--baseline 0.30 --seconds 3"
```

This should start, show disconnected cameras, and exit after three seconds. The first run may download dependencies.
PhotonLib may also print warnings about missing coprocessors/interface UUIDs when no Pi is connected; those are expected for this startup check. NetworkTables may warn about the empty persistence filename; this test intentionally does not persist settings.

## Hardware and PhotonVision

1. Connect both cameras to the Orange Pi. Power the Pi and radio correctly and connect the laptop and Pi to the same network.
2. Fix both cameras to a rigid mount, level and parallel, facing forward. Camera `stereo_camera_0` must be LEFT and `stereo_camera_1` RIGHT, as viewed from behind the cameras looking forward. Measure optical-center spacing in meters. Table height is not needed.
3. Open `http://<Pi-IP>:5800`. Configure both camera nicknames as above. For each, configure pipeline index 0 for object detection with the FUEL model, and verify detection boxes in both feeds. Use an Orange Pi supported for object detection.
4. Calibrate both cameras individually in PhotonVision at the detection resolution. Disable any custom target offset so angles refer to the detection center.
5. Run `ipconfig` on the laptop. Find the IPv4 address of the adapter connected to the radio network (not an unrelated Wi-Fi adapter).
6. In PhotonVision Settings / Networking, put that LAPTOP address in the **Team Number / NetworkTables Server Address** field. PhotonVision must act as a client; disable its own NetworkTables server if that toggle is present. Save/apply. Do not use localhost: the Pi and laptop are different devices. Do not replace the Pi's own IP with the laptop's IP.
7. Allow the WPILib Java test process through Windows Firewall on this network if prompted. NetworkTables uses TCP 5810 (also listens on 1735); PhotonLib time synchronization uses UDP 5810. Avoid running another robot simulator/NT server at the same time.
8. Match PhotonVision to the actual PhotonLib dependency version. This repository's Java and native dependencies currently use **v2026.1.1-rc-2**, despite the stale top-level v2025.3.1 label in `vendordeps/photonlib.json`. If installing a different 2026 release, update the vendor dependency through WPILib's vendor manager and rebuild; don't just edit its display label.

References: [PhotonVision networking](https://docs.photonvision.org/en/latest/docs/quick-start/networking.html), [object detection](https://docs.photonvision.org/en/latest/docs/objectDetection/about-object-detection.html), [calibration](https://docs.photonvision.org/en/latest/docs/calibration/calibration.html).

## Run and read coordinates

For measured 30 cm spacing, run:

```powershell
.\gradlew.bat stereoTableTest --args="--baseline 0.30" --console=plain
```

Replace `0.30` with your measured spacing. No source edit is needed. Stop with Ctrl+C. `--seconds 10` optionally limits the run.

The terminal prints camera connection state, fuel count, frame age, and either coordinates or an explanation every half second. PhotonVision's browser shows the images/detection boxes; computed stereo coordinates appear in the laptop terminal. This runner does not start AdvantageKit logging.

Origin is halfway between the optical centers. X is forward, Y left, Z up, all in meters. A ball below the cameras has negative Z. Example: `X=1.500 m Y=0.100 m Z=-0.600 m`. Coordinates estimate the detected ball center, not its floor contact point. Add measured camera-center height to Z if you need height above the floor.

Begin with one stationary ball 1–2 meters away, visible in BOTH feeds. Compare with tape measurements from the rig midpoint to the approximate ball center. Move it forward/back and left/right to check axis signs. Remove it and confirm positions disappear. Multiple balls can be mismatched by the current greedy ray matching; evaluate those only after the single-ball test works.

## If there are no coordinates

| Terminal message | What to check |
| --- | --- |
| Disconnected | Laptop server IP, camera nicknames, firewall, Pi networking, version compatibility |
| Missing/stale frames or clocks | PhotonLib time sync, FPS/load, UDP 5810, and matching software versions |
| No fuel | Pipeline 0/model, visibility, lighting, class ID 0, and detection boxes |
| Frames too far apart | Camera frame rates/load; use a stationary target first |
| Rays do not match | Camera order, measured baseline, level/parallel mount, both cameras seeing the same ball |

Frames older than 0.25 seconds and pairs more than 0.10 seconds apart are rejected. Cameras physically tilted or rolled require measured mounting rotations; this simple runner assumes zero rotation and equal camera heights. Don't loosen these checks just to force coordinates.
