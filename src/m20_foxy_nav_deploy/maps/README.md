# Maps

Put the final Nav2 2D occupancy grid map files here before deploying.

Expected pair:

```text
maps/m20_site_map.yaml
maps/m20_site_map.pgm
```

The YAML file should look like:

```yaml
image: m20_site_map.pgm
mode: trinary
resolution: 0.05
origin: [0.0, 0.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

You can also pass an absolute map path at launch time:

```bash
ros2 launch m20_foxy_nav_deploy m20_nav_bringup.launch.py \
  map:=/absolute/path/to/your_map.yaml
```
