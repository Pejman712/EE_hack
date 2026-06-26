#!/usr/bin/env python3
"""Generate an industrial warehouse Gazebo world + a MATCHING occupancy map.

Both are built from the same 2D obstacle footprints, so the saved map lines up 1:1
with the simulated world -> AMCL localizes and Nav2 plans correctly. Robot spawns
at (0,0), which is kept clear (central aisle).
"""
import os, sys

RES = 0.05
# (cx, cy, sx, sy, height)  metres, centred footprints
WALL_H, SHELF_H, CRATE_H = 3.0, 2.2, 1.0
BX, BY = 12.0, 8.0   # interior half-extents -> 24 x 16 m building

boxes = []
def add(cx, cy, sx, sy, h): boxes.append((cx, cy, sx, sy, h))

# Outer walls
add(0, -BY, 2*BX+0.3, 0.3, WALL_H)
add(0,  BY, 2*BX+0.3, 0.3, WALL_H)
add(-BX, 0, 0.3, 2*BY, WALL_H)
add( BX, 0, 0.3, 2*BY, WALL_H)
# Shelving racks (rows along y), leaving a wide central aisle at x=0
for x in (-8.0, -4.0, 4.0, 8.0):
    add(x, 0.0, 1.0, 11.0, SHELF_H)
# A few crates / pillars in free space (kept off (0,0))
for (cx, cy, s) in [(10.0,-6.0,1.0),(10.0,6.0,1.0),(-10.0,6.0,1.2),(-10.0,-6.0,1.2),(0.0,6.5,1.5),(0.0,-6.5,1.5)]:
    add(cx, cy, s, s, CRATE_H)

# ---------- SDF world ----------
def box_link(i, cx, cy, sx, sy, h, rgb):
    return f"""      <link name="b{i}">
        <pose>{cx:.3f} {cy:.3f} {h/2:.3f} 0 0 0</pose>
        <collision name="c"><geometry><box><size>{sx:.3f} {sy:.3f} {h:.3f}</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>{sx:.3f} {sy:.3f} {h:.3f}</size></box></geometry>
          <material><ambient>{rgb} 1</ambient><diffuse>{rgb} 1</diffuse></material></visual>
      </link>"""

links = []
for i,(cx,cy,sx,sy,h) in enumerate(boxes):
    rgb = "0.55 0.55 0.58" if h >= WALL_H-0.01 else ("0.30 0.45 0.65" if h==SHELF_H else "0.6 0.45 0.25")
    links.append(box_link(i, cx, cy, sx, sy, h, rgb))

world = f"""<?xml version="1.0"?>
<sdf version="1.8">
  <world name="industrial">
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <scene><ambient>0.5 0.5 0.5 1</ambient><background>0.7 0.7 0.75 1</background></scene>
    <light type="directional" name="sun"><cast_shadows>true</cast_shadows><pose>0 0 12 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse><specular>0.2 0.2 0.2 1</specular><direction>-0.4 0.3 -0.9</direction></light>
    <model name="ground"><static>true</static><link name="l">
      <collision name="c"><geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry></collision>
      <visual name="v"><geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry>
        <material><ambient>0.35 0.35 0.35 1</ambient><diffuse>0.4 0.4 0.4 1</diffuse></material></visual>
    </link></model>
    <model name="warehouse"><static>true</static>
{chr(10).join(links)}
    </model>
  </world>
</sdf>
"""

# ---------- occupancy map (rasterise footprints) ----------
ox, oy = -(BX+0.6), -(BY+0.6)
W = int((2*(BX+0.6))/RES); H = int((2*(BY+0.6))/RES)
px = bytearray([254])*(W*H)   # free=light
def occ(x, y):
    for (cx,cy,sx,sy,h) in boxes:
        if abs(x-cx) <= sx/2 and abs(y-cy) <= sy/2:
            return True
    return False
for row in range(H):
    y = oy + (H-1-row)*RES + RES/2
    for col in range(W):
        x = ox + col*RES + RES/2
        if occ(x, y):
            px[row*W+col] = 0   # occupied=dark

outdir = sys.argv[1] if len(sys.argv) > 1 else "."
open(os.path.join(outdir, "industrial.world"), "w").write(world)
with open(os.path.join(outdir, "industrial.pgm"), "wb") as f:
    f.write(f"P5\n{W} {H}\n255\n".encode()); f.write(bytes(px))
open(os.path.join(outdir, "industrial.yaml"), "w").write(
    f"image: industrial.pgm\nmode: trinary\nresolution: {RES}\n"
    f"origin: [{ox:.4f}, {oy:.4f}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
print(f"world: {len(boxes)} boxes | map: {W}x{H} px ({W*RES:.0f}x{H*RES:.0f} m), origin ({ox:.2f},{oy:.2f})")
