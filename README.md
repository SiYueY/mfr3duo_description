# mfr3duo_description

ROS 2 URDF/Xacro description for the Mobile FR3 Duo. The public entry point is
`urdf/mfr3duo.urdf.xacro`; it is self-contained at runtime and includes a
vendored, traceable Franka 2.8.1 model closure.

See `docs/urdf.md` for the flat Xacro layout and official-model source/version
record.

The self-contained MuJoCo model is available at `mjcf/mfr3duo.xml`; see
`docs/mjcf.md` for its usage and asset provenance.

```bash
xacro $(ros2 pkg prefix mfr3duo_description)/share/mfr3duo_description/urdf/mfr3duo.urdf.xacro
```

For the fixed default assembly (`connected_to=base`), use the committed URDF
directly without Xacro:

```bash
$(ros2 pkg prefix mfr3duo_description)/share/mfr3duo_description/urdf/mfr3duo.urdf
```

To visualize the default model with RViz and GUI joint controls:

```bash
ros2 run mfr3duo_description visualize_urdf
```

Build the package and source `install/setup.bash` first. Closing RViz stops the
publisher processes and removes the script's temporary parameter file.

The default robot contains the IMU, front/rear RealSense D455 cameras,
front/rear NanoScan3 lidars, a head-mounted ZED Mini, and a D435 plus separate
wrist mount on each hand. The two wrist-camera mounting transforms are
provisional figure-fit values and must be replaced with CAD or measured
calibration before deployment. All runtime visual and collision meshes are in
`urdf/meshes/`, directly sourced from frozen official vendor releases. No other
robot-description package is required.
