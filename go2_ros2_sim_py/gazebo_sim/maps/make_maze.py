#!/usr/bin/env python3
"""Generate a maze Gazebo world + a MATCHING occupancy map (same footprints).

Recursive-backtracker maze; wide corridors so the Go2 (with costmap inflation) can
pass. Robot spawns at (0,0) = the start cell (kept open).
"""
import os, sys, random
random.seed(7)

NCOL, NROW = 7, 5
C   = 2.6     # corridor width / cell spacing (m)  -> ~0.9 m clearance after inflation
T   = 0.2     # wall thickness
H   = 1.5     # wall height
RES = 0.05

# --- carve the maze (remove walls between cells) ---
walls = set()
for c in range(NCOL):
    for r in range(NROW):
        if c+1 < NCOL: walls.add(frozenset({(c, r), (c+1, r)}))
        if r+1 < NROW: walls.add(frozenset({(c, r), (c, r+1)}))
visited = {(0, 0)}; stack = [(0, 0)]
while stack:
    c, r = stack[-1]
    nbrs = [(c+dc, r+dr) for dc, dr in ((1,0),(-1,0),(0,1),(0,-1))
            if 0 <= c+dc < NCOL and 0 <= r+dr < NROW and (c+dc, r+dr) not in visited]
    if nbrs:
        n = random.choice(nbrs)
        walls.discard(frozenset({(c, r), n})); visited.add(n); stack.append(n)
    else:
        stack.pop()

boxes = []  # (cx, cy, sx, sy)
def add(cx, cy, sx, sy): boxes.append((cx, cy, sx, sy))

# internal walls that survived
for w in walls:
    (ax, ay), (bx, by) = sorted(w)
    if bx == ax + 1:                      # vertical wall between horizontal neighbours
        add((ax+0.5)*C, ay*C, T, C+T)
    else:                                 # horizontal wall between vertical neighbours
        add(ax*C, (ay+0.5)*C, C+T, T)
# outer boundary
x0, x1 = -C/2, (NCOL-1)*C + C/2
y0, y1 = -C/2, (NROW-1)*C + C/2
add((x0+x1)/2, y0, x1-x0+T, T)
add((x0+x1)/2, y1, x1-x0+T, T)
add(x0, (y0+y1)/2, T, y1-y0+T)
add(x1, (y0+y1)/2, T, y1-y0+T)

# --- SDF world ---
def link(i, cx, cy, sx, sy):
    return f"""      <link name="w{i}">
        <pose>{cx:.3f} {cy:.3f} {H/2:.3f} 0 0 0</pose>
        <collision name="c"><geometry><box><size>{sx:.3f} {sy:.3f} {H:.3f}</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>{sx:.3f} {sy:.3f} {H:.3f}</size></box></geometry>
          <material><ambient>0.45 0.5 0.55 1</ambient><diffuse>0.5 0.55 0.6 1</diffuse></material></visual>
      </link>"""
links = "\n".join(link(i, *b) for i, b in enumerate(boxes))
world = f"""<?xml version="1.0"?>
<sdf version="1.8">
  <world name="maze">
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <scene><ambient>0.5 0.5 0.5 1</ambient><background>0.7 0.75 0.8 1</background></scene>
    <light type="directional" name="sun"><cast_shadows>true</cast_shadows><pose>0 0 12 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse><direction>-0.4 0.3 -0.9</direction></light>
    <model name="ground"><static>true</static><link name="l">
      <collision name="c"><geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry></collision>
      <visual name="v"><geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry>
        <material><ambient>0.35 0.35 0.35 1</ambient><diffuse>0.4 0.4 0.4 1</diffuse></material></visual>
    </link></model>
    <model name="maze"><static>true</static>
{links}
    </model>
  </world>
</sdf>
"""

# --- occupancy map (rasterise the same boxes) ---
M = 0.6
ox, oy = x0 - M, y0 - M
W = int((x1 - x0 + 2*M)/RES); Hh = int((y1 - y0 + 2*M)/RES)
px = bytearray([254])*(W*Hh)
for row in range(Hh):
    y = oy + (Hh-1-row)*RES + RES/2
    for col in range(W):
        x = ox + col*RES + RES/2
        for (cx, cy, sx, sy) in boxes:
            if abs(x-cx) <= sx/2 and abs(y-cy) <= sy/2:
                px[row*W+col] = 0; break

outdir = sys.argv[1] if len(sys.argv) > 1 else "."
open(os.path.join(outdir, "maze.world"), "w").write(world)
with open(os.path.join(outdir, "maze.pgm"), "wb") as f:
    f.write(f"P5\n{W} {Hh}\n255\n".encode()); f.write(bytes(px))
open(os.path.join(outdir, "maze.yaml"), "w").write(
    f"image: maze.pgm\nmode: trinary\nresolution: {RES}\n"
    f"origin: [{ox:.4f}, {oy:.4f}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
print(f"maze {NCOL}x{NROW} cells, corridor {C} m | {len(boxes)} wall boxes | "
      f"map {W}x{Hh}px ({W*RES:.0f}x{Hh*RES:.0f} m), origin ({ox:.2f},{oy:.2f})")
