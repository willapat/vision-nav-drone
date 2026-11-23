#!/usr/bin/env python3
"""Turn a Forest3D world into a PX4-ready Gazebo Harmonic world.

Forest3D (https://github.com/unitsSpaceLab/Forest3D) places terrain, trees, bushes,
rocks and ground cover. This script keeps those placements and rebuilds everything
else the way PX4 SITL expects:

  * PX4-style physics (4 ms step), sky, sun with shadows and spherical coordinates
  * the world shifted vertically so the terrain at the origin (the drone's spawn
    point) is at z = 0
  * a clear spawn area around the origin with a flat landing pad
  * model URIs prefixed with model://forest3d/, resolved through a symlink in
    PX4's models folder, so no extra environment variables are needed
  * ground truth: every placed object's name, model, pose and scale written to a
    CSV next to the world, for planners and for scoring runs later

Usage:
  python3 forest3d_to_px4.py --forest3d ~/Forest3D --px4 ~/PX4-Autopilot [--name forest3d]
"""
import argparse
import csv
import math
import os
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

# Clear radius (m) around the spawn point per category.
SPAWN_CLEAR = {'tree': 9.0, 'bush': 5.0, 'rock': 4.0, 'grass': 1.5}

HEADER = '''<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <world name="{name}">
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene>
      <grid>false</grid>
      <ambient>0.45 0.45 0.45 1</ambient>
      <background>0.62 0.75 0.9 1</background>
      <shadows>true</shadows>
      <sky/>
    </scene>
    <light name="sun" type="directional">
      <pose>0 0 200 0 0 0</pose>
      <cast_shadows>true</cast_shadows>
      <intensity>1.2</intensity>
      <diffuse>1 0.96 0.88 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.45 0.25 -0.85</direction>
    </light>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>47.397971057728974</latitude_deg>
      <longitude_deg>8.546163739800146</longitude_deg>
      <elevation>488</elevation>
    </spherical_coordinates>
'''
# Flat take-off/landing pad at the spawn point, top surface at PAD_TOP so it clears the
# sloping terrain. Spawn the drone on it with PX4_GZ_MODEL_POSE="0,0,PAD_TOP"
# (sim/run_forest.sh does this).
PAD_TOP = 0.15
PAD = '''    <model name="landing_pad">
      <static>true</static>
      <pose>0 0 {z:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>2.5 2.5 0.3</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>2.5 2.5 0.3</size></box></geometry>
          <material><ambient>0.25 0.18 0.12 1</ambient><diffuse>0.38 0.28 0.18 1</diffuse>
            <specular>0.05 0.05 0.05 1</specular></material></visual>
      </link>
    </model>
'''
FOOTER = '''  </world>
</sdf>
'''


def read_stl(path):
    """Triangles of an ASCII or binary STL as an (n, 3, 3) array."""
    data = path.read_bytes()
    if data[:5] == b'solid' and b'facet' in data[:512]:
        v = [float(x) for x in re.findall(rb'vertex\s+(\S+)\s+(\S+)\s+(\S+)', data) for x in x]
        return np.array(v).reshape(-1, 3, 3)
    n = int.from_bytes(data[80:84], 'little')
    rec = np.frombuffer(data[84:84 + n * 50], dtype=np.dtype([
        ('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]))
    return rec['v'].astype(float)


def height_at(tris, x, y):
    """Terrain height under (x, y), by barycentric interpolation."""
    a, b, c = tris[:, 0], tris[:, 1], tris[:, 2]
    d = (b[:, 1] - c[:, 1]) * (a[:, 0] - c[:, 0]) + (c[:, 0] - b[:, 0]) * (a[:, 1] - c[:, 1])
    ok = np.abs(d) > 1e-12
    d = np.where(ok, d, 1)
    l1 = ((b[:, 1] - c[:, 1]) * (x - c[:, 0]) + (c[:, 0] - b[:, 0]) * (y - c[:, 1])) / d
    l2 = ((c[:, 1] - a[:, 1]) * (x - c[:, 0]) + (a[:, 0] - c[:, 0]) * (y - c[:, 1])) / d
    l3 = 1 - l1 - l2
    inside = ok & (l1 >= -1e-9) & (l2 >= -1e-9) & (l3 >= -1e-9)
    if not inside.any():
        raise SystemExit(f'({x}, {y}) is outside the terrain')
    i = np.argmax(inside)
    return float(l1[i] * a[i, 2] + l2[i] * b[i, 2] + l3[i] * c[i, 2])


def prefix_model_uris(models_dir, prefix):
    """Rewrite model://X inside every model.sdf to model://<prefix>/X (idempotent)."""
    for sdf in models_dir.rglob('model.sdf'):
        text = sdf.read_text()
        new = re.sub(r'model://(?!%s/)' % re.escape(prefix), f'model://{prefix}/', text)
        if new != text:
            sdf.write_text(new)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--forest3d', type=Path, default=Path('~/Forest3D').expanduser())
    ap.add_argument('--px4', type=Path, default=Path('~/PX4-Autopilot').expanduser())
    ap.add_argument('--world', type=Path, help='Forest3D world (default: <forest3d>/worlds/forest_world.world)')
    ap.add_argument('--name', default='forest3d', help='world name, used as PX4_GZ_WORLD')
    args = ap.parse_args()

    f3d = args.forest3d.expanduser()
    models = f3d / 'models'
    src = args.world or f3d / 'worlds' / 'forest_world.world'
    px4_models = args.px4.expanduser() / 'Tools/simulation/gz/models'
    px4_worlds = args.px4.expanduser() / 'Tools/simulation/gz/worlds'
    prefix = 'forest3d'

    # 1. Make the models reachable as model://forest3d/... from PX4's model path.
    link = px4_models / prefix
    if link.is_symlink() or not link.exists():
        if link.is_symlink():
            link.unlink()
        link.symlink_to(models.resolve(), target_is_directory=True)
    prefix_model_uris(models, prefix)

    # 2. Read Forest3D's placements.
    world = ET.parse(src).getroot().find('world')
    includes = world.findall('include')
    dz = -height_at(read_stl(models / 'ground/mesh/terrain.stl'), 0.0, 0.0)

    body, rows, removed = [], [], {}
    for inc in includes:
        uri = inc.findtext('uri')
        rel = uri.split('model://', 1)[1]
        category = rel.split('/')[0]
        name = inc.findtext('name') or rel.replace('/', '_')
        pose = [float(v) for v in (inc.findtext('pose') or '0 0 0 0 0 0').split()]
        scale = inc.findtext('scale')
        if category in SPAWN_CLEAR and math.hypot(pose[0], pose[1]) < SPAWN_CLEAR[category]:
            removed[category] = removed.get(category, 0) + 1
            continue
        pose[2] += dz
        p = ' '.join(f'{v:.4f}' for v in pose)
        # SDF <include> has no <scale>; Gazebo ignores it (with a warning per object),
        # so it is dropped here and every object is placed at its model's native size.
        body.append(f'    <include>\n      <uri>model://{prefix}/{rel}</uri>\n      <name>{name}</name>\n'
                    f'      <pose>{p}</pose>\n    </include>\n')
        if category != 'ground':
            rows.append([name, category, rel.split('/')[-1], *(f'{v:.4f}' for v in pose), '1'])

    out = px4_worlds / f'{args.name}.sdf'
    out.write_text(HEADER.format(name=args.name) + PAD.format(z=PAD_TOP - 0.15) + ''.join(body) + FOOTER)
    gt = px4_worlds / f'{args.name}_objects.csv'
    with gt.open('w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['name', 'category', 'model', 'x', 'y', 'z', 'roll', 'pitch', 'yaw', 'scale'])
        w.writerows(rows)

    counts = {}
    for r in rows:
        counts[r[1]] = counts.get(r[1], 0) + 1
    print(f'World:  {out}')
    print(f'Ground truth: {gt}')
    print(f'Placed: {counts}; cleared around spawn: {removed}; vertical shift {dz:+.2f} m')
    print(f'Run:    cd {args.px4} && PX4_GZ_WORLD={args.name} make px4_sitl gz_x500')


if __name__ == '__main__':
    main()
