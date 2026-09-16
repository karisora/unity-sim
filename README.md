# unity-sim / ERC2026

ERC rover simulation project for Unity, including scenes, rover models, sensors,
terrain data, and ROS 2 integration.

This branch is a complete **Unity project**. Open the repository folder in Unity
Hub; it is not a Unity Package Manager (UPM) package.

## Requirements

- Unity **2022.3.62f3** (see `ProjectSettings/ProjectVersion.txt`).
- Git and Git LFS for the large terrain files.
- Linux x86_64 and **ROS 2 Jazzy** for the bundled ROS2ForUnity native plugins.
  Native plugins for other platforms are not included.

## Get the project

```bash
git lfs install
git clone --branch ERC2026 git@github.com:karisora/unity-sim.git
cd unity-sim
git lfs pull
```

Use a GitHub account with access to the repository. Downloading a ZIP is not the
documented setup path, because the terrain files are stored in Git LFS.

## Open in Unity

Add the cloned folder in Unity Hub and select Unity 2022.3.62f3. Unity restores
packages from `Packages/manifest.json` and regenerates `Library/` on first import.

For ROS 2 simulation, launch the editor from a terminal with the ROS environment
loaded. The included script loads `/opt/ros/jazzy/setup.bash`:

```bash
./launch_unity_ros2_jazzy.sh
```

If Unity or ROS is installed elsewhere, override their paths:

```bash
UNITY_EDITOR=/path/to/Editor/Unity ROS_SETUP=/path/to/setup.bash \
  ./launch_unity_ros2_jazzy.sh
```

Open the desired scene in the Project window, such as
`Assets/ARES8/ARES8.unity` or a scene under `Assets/Simulation/Scenes/`, then enter
Play Mode. The branch name is ERC2026; the existing scene filenames are preserved.
The build settings currently have no scenes registered, so select the required
scenes in Build Settings before making a standalone player build.

## Versioned files

- `Assets/` and its `.meta` files preserve Unity assets, scripts, plugins, and GUIDs.
- `Packages/` and `ProjectSettings/` preserve package dependencies and project settings.
- `SourceAssets/` and `Tools/` preserve terrain source data and preparation scripts.
- Large OBJ terrain models and the TIFF orthophoto are stored in Git LFS.

Unity caches, temporary files, editor user settings, generated IDE projects, and
crash dumps are excluded by `.gitignore`.
