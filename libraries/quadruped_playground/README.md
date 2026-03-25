# quadruped_playground

Gazebo Classic simulation assets for the Lite3 workspace.

## Layout

- `models/`: Gazebo Classic models used by simulation worlds.
- `worlds/`: Gazebo Classic world files.
- `config/`: Optional visualization and SLAM configs kept with the simulation assets.

## Included assets

- `models/test_world/`: Building Editor model migrated from the external `Untitled` asset.
- `worlds/test_world.world`: Gazebo Classic world that loads `model://test_world` with `ground_plane` and `sun`.

## Usage

From the workspace root after building:

```bash
source install/setup.bash
./src/lite3_gazebo_classic/run_gazebo_world.sh
```

Or specify a custom world path:

```bash
./src/lite3_gazebo_classic/run_gazebo_world.sh /absolute/path/to/world.world
```
