# SoMaSLAM

SoMaSLAM extends [Efficient 2D Graph SLAM for Sparse Sensing](https://github.com/shiftlab-nanodrone/sparse-gslam) (sparse-gslam, IROS 2022) with **landmark-landmark constraints**. On top of the original pose-pose and pose-landmark edges, SoMaSLAM adds edges between line landmarks that are nearly parallel or orthogonal to each other, which pulls the line map toward a structurally consistent layout.

**Paper:** IEEE Robotics and Automation Letters (RA-L), 2025 · **Project page:** https://somaslam.github.io/

## Landmark-landmark parameters

These go in a dataset's `slam*.yaml`:

| Parameter | Meaning |
|---|---|
| `landmark_min_length_threshold` | Minimum segment length (m) for both landmarks before a constraint is considered |
| `max_past_landmark_ID` | Only landmarks at most this many IDs older than the current one are considered |
| `max_past_landmark_dist` | Two segments count as "close" if they are within this distance (m) |
| `min_pl_edge_count` | Minimum number of pose-landmark observations the past landmark must have |
| `parallel_ll_info_matrix` | Information scale for parallel constraints |
| `orthogonal_ll_info_matrix` | Information scale for orthogonal constraints |
| `maximum_constraints` | Read from the config but not used yet |

All of these parameters are required. A config that doesn't set them will fail at startup. These configs already set them:

- `aces/slam-4`, `aces/slam-11`
- `intel-lab/slam-4`, `intel-lab/slam-11`
- `mit-killian/slam`, `mit-killian/slam-4`
- `dagstuhl/slam-4`, `dagstuhl/slam-11`
- `freiburg-079/slam-4`, `freiburg-079/slam-11`
- `mit-csail-custom/slam-0`, `-4`, `-11`
- `nsh_level_a/slam`, `stanford-gates/slam`, `usc-sal/slam`, `custom_straight/slam`

## Installation

The setup is the same as upstream sparse-gslam. We recommend Ubuntu 20.04 with ROS Noetic. Ubuntu 18.04 with ROS Melodic should also work. You need a C++14-capable compiler.

1. Install dependencies:

    ```bash
    sudo apt install ros-noetic-jsk-rviz-plugins ros-noetic-navigation ros-noetic-joy
    ```

2. Build and install [Google Cartographer](https://google-cartographer.readthedocs.io/en/latest/).

3. Clone [libg2o-release](https://github.com/ros-gbp/libg2o-release). It is used instead of upstream g2o, whose bundled ceres-solver can conflict with Cartographer's.
    - Check out the `release/{your-ros-distro}/libg2o` branch.
    - In `config.h.in`, make sure `#define G2O_DELETE_IMPLICITLY_OWNED_OBJECTS 0` is enabled. This code manages edge and vertex memory itself, so you will get a segfault without it.
    - In `CMakeLists.txt`, make sure `BUILD_WITH_MARCH_NATIVE` is `ON`. Without it you may get a segfault.
    - Build and install it, then run `sudo ldconfig`.

4. Clone this repository and build:

    ```bash
    git clone https://github.com/ugrm/somaslam.git
    cd somaslam
    catkin_make -DCMAKE_BUILD_TYPE=Release
    ```

5. Download the datasets and evaluation scripts:

    ```bash
    cd src/sparse_gslam/datasets
    ./download.sh
    ```

## Running

```bash
source devel/setup.bash
roslaunch sparse_gslam log_runner.launch dataset:=intel-lab postfix:=-4
```

`dataset` is a directory under `src/sparse_gslam/datasets`. `postfix` picks `slam{postfix}.yaml` in that directory. Use a config that has the landmark-landmark parameters (see the list above).

Results are written to the dataset directory as `{dataset}.result`, `.result2`, `.ftime`, `.btime` and `.dtime`.

## Computing SLAM metrics

Ground truth is available for aces, intel-lab and mit-killian. After a run finishes:

```bash
cd src/sparse_gslam/datasets
./eval.sh {dataset_name}
```

## Citation

If you use this code, please cite:

```bibtex
@article{han2025somaslam,
  title={SoMaSLAM: 2D Graph SLAM for sparse range sensing with soft Manhattan world constraints},
  author={Han, Jeahn and Hu, Zichao and Yang, Seonmo and Kim, Minji and Kim, Pyojin},
  journal={IEEE Robotics and Automation Letters},
  volume={10},
  number={9},
  pages={9280--9287},
  year={2025},
  publisher={IEEE}
}
```

## Acknowledgements

This project is built on [sparse-gslam](https://github.com/shiftlab-nanodrone/sparse-gslam) by shiftlab. Its paper is included at [paper/iros2022.pdf](paper/iros2022.pdf). Please cite their work as well if you use this code.

## License

MIT, see [LICENSE](LICENSE). The original copyright (c) 2024 shiftlab is kept.
