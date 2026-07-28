# Third-party frontier search core

The files under `third_party/frontier_exploration_ros2/` are a narrowly adapted subset of
[Mert Güler's frontier-exploration-ros2](https://github.com/mertgulerx/frontier-exploration-ros2),
Apache-2.0, inspected at local version 1.6.0.

Upstream source files used as the basis:

- `include/frontier_exploration_ros2/frontier_types.hpp`
- `include/frontier_exploration_ros2/frontier_search.hpp`
- `src/frontier_search.cpp`

Project modifications are prominent in the adapted files: the data model was reduced to the
candidate-generator use case, complete boundary geometry is retained for stable IDs, occupancy-grid
origin yaw is applied in both map/world directions, and extraction is transport-independent.
The complete upstream explorer, dispatcher, MRTSP solver, and suppression implementation are not copied.
See `LICENSES/Apache-2.0.txt`.
