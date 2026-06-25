#!/usr/bin/env python3
"""Convert a Nav2 occupancy map (PGM + YAML) into a Gazebo SDF world.

Occupied cells become wall boxes, positioned in the SAME coordinates as the map
(using its origin + resolution) so the saved map and the simulated world line up
1:1 -- meaning AMCL can localize and you can navigate by real-map coordinates.
Occupied cells are merged along each row (run-length) to cut the box count.
"""
import sys, yaml, os

def load_pgm(path):
    with open(path, 'rb') as f:
        data = f.read()
    assert data[:2] == b'P5', "expected binary PGM (P5)"
    idx, vals = 2, []
    while len(vals) < 3:
        while data[idx] in b' \t\n\r': idx += 1
        if data[idx:idx+1] == b'#':
            while data[idx] not in b'\n': idx += 1
            continue
        s = idx
        while data[idx] not in b' \t\n\r': idx += 1
        vals.append(int(data[s:idx]))
    w, h, _ = vals
    idx += 1
    px = data[idx:idx+w*h]
    return w, h, px

def main(yaml_path, out_path, wall_h=2.0):
    with open(yaml_path) as f:
        m = yaml.safe_load(f)
    res = m['resolution']; ox, oy, _ = m['origin']
    pgm = os.path.join(os.path.dirname(yaml_path), m['image'])
    w, h, px = load_pgm(pgm)
    occ_thresh = 255 * (1.0 - m.get('occupied_thresh', 0.65))  # dark = occupied

    boxes = []  # (cx, cy, sx, sy)
    for row in range(h):
        col = 0
        while col < w:
            if px[row*w + col] < occ_thresh:
                start = col
                while col < w and px[row*w + col] < occ_thresh:
                    col += 1
                run = col - start
                # map-frame coords (PGM row 0 = top = max y)
                cx = ox + (start + run/2.0) * res
                cy = oy + (h - 1 - row) * res
                boxes.append((cx, cy, run*res, res))
            else:
                col += 1

    links = []
    for i, (cx, cy, sx, sy) in enumerate(boxes):
        links.append(f"""      <link name="w{i}">
        <pose>{cx:.3f} {cy:.3f} {wall_h/2:.3f} 0 0 0</pose>
        <collision name="c"><geometry><box><size>{sx:.3f} {sy:.3f} {wall_h:.3f}</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>{sx:.3f} {sy:.3f} {wall_h:.3f}</size></box></geometry>
          <material><ambient>0.7 0.7 0.7 1</ambient><diffuse>0.7 0.7 0.7 1</diffuse></material></visual>
      </link>""")
    world = f"""<?xml version="1.0"?>
<sdf version="1.8">
  <world name="real_map">
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows><pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse><specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <model name="ground"><static>true</static><link name="l">
      <collision name="c"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry></collision>
      <visual name="v"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        <material><ambient>0.3 0.3 0.3 1</ambient><diffuse>0.4 0.4 0.4 1</diffuse></material></visual>
    </link></model>
    <model name="walls"><static>true</static>
{chr(10).join(links)}
    </model>
  </world>
</sdf>
"""
    with open(out_path, 'w') as f:
        f.write(world)
    print(f"wrote {out_path}: {len(boxes)} wall boxes from {w}x{h} map")

if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
