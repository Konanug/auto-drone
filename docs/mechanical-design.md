# Mechanical design

Three custom parts were designed in SolidWorks and printed to carry the autonomy hardware
on a frame that was not built for it: a **Raspberry Pi mount**, a **camera mount**, and a
**GPS mount**.

> CAD source files and renders are not yet in the repository. When added they belong in
> `assets/cad/` (renders) with source geometry alongside; this document is where the design
> intent and measurements are recorded.

<!-- Add SolidWorks render of the camera mount here: assets/cad/camera-mount-cad.png -->
<!-- Add Raspberry Pi mount render here: assets/cad/pi-mount-cad.png -->
<!-- Add GPS mount render here: assets/cad/gps-mount-cad.png -->
<!-- Add exploded assembly view here: assets/cad/exploded-assembly.png -->
<!-- Add printed-vs-CAD comparison photo here: assets/cad/printed-vs-cad.jpg -->

## Constraints

A 7-inch airframe sized for FPV has no accommodation for a companion computer, a camera at
a fixed known pose, and a GPS mast. The mounts had to satisfy several constraints at once:

- **Known, rigid camera geometry.** Pose estimation converts a camera measurement into a
  vehicle-relative command, so the camera's position and orientation relative to the flight
  controller are part of the control maths. If the mount flexes, the transform is wrong.
- **Mass and its distribution.** Total mass is about 816 g. Anything added forward of the
  centre of gravity has to be balanced against the rest of the stack.
- **Thermal and cable routing.** The Pi needs airflow and a short, direct UART run to the
  flight controller.
- **Serviceability.** The camera and Pi have to come off without disassembling the stack.

## Measured camera offset

The camera is mounted **inverted**, 90 mm forward of and 30 mm above the flight controller.
Those measurements are not documentation — they are constants in
[`hover_on_tag.py`](../hover_on_tag.py) that shift the camera's measurement of the tag onto
the vehicle centre:

```python
CAM_OFFSET_FWD_M   =  0.090   # camera ahead of the FC
CAM_OFFSET_RIGHT_M =  0.0
CAM_OFFSET_DOWN_M  = -0.030   # camera is above the FC, so negative in FRD
```

The inversion is handled in the camera's ISP as a hardware 180° rotation rather than in
software, so no consumer pays for a frame flip — and calibration is captured through the
same rotation, keeping the intrinsics valid for how they are used.

Re-measure and update these constants if the camera mount changes.

## The vibration path

The mechanical design is now the project's limiting factor, so the camera mount is where
the next iteration goes.

Motors vibrate at roughly 65–100 Hz. That energy travels through the arms and frame plate
into the camera mount, and from there into a rolling-shutter sensor whose readout takes
about 8.3 ms — long enough for the vibration to shear the image row by row. The tag's
corner geometry is the input to pose estimation, so mechanical vibration becomes control
error more or less directly.

The current mount is rigid, which is correct for holding a known camera pose and wrong for
isolating vibration. Those two requirements are in tension, and resolving that tension is
the design problem:

- **Damped isolation** — soft mounts or gel between the frame and the camera, which trades
  a little geometric certainty for a large reduction in transmitted energy.
- **Source reduction** — propeller and motor balancing, which lowers the input rather than
  filtering it, and helps the flight controller's own gyro filtering at the same time.
- **Sensor change** — a global-shutter camera removes the shear mechanism entirely, at the
  cost of resolution and light sensitivity.

Related: [Vision and camera](vision-and-camera.md) covers what the software side already
does about this, and [Safety and limitations](safety-and-limitations.md) covers where the
boundary between the two now sits.
