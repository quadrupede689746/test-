# robodog_sim

The supplied archive contained only empty placeholder directories for this
package, not a separate simulation model, world, meshes, or RViz configuration.
The actual robot description and its meshes are in `open_quadruped`.

This package provides a simple launch alias:

```bash
ros2 launch robodog_sim sim.launch.py
```

It delegates to `open_quadruped/launch/sim.launch.py`.
