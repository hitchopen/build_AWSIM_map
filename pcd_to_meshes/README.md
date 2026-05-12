# pcd → AWSIM ground + wall meshes

Builds the **Unity-side** geometry an AWSIM scene needs to let a virtual ego
vehicle drive on the track, straight from the PCD + trajectory that
[`mcap_to_pcd/build_map.py`](../mcap_to_pcd/) produces:

```
ground.obj                drivable surface              (Unity layer: Ground)
wall_left.obj             inner cliff face / wall       (Unity layer: Wall)
wall_right.obj            outer parapet / cliff drop    (Unity layer: Wall)
meshes_manifest.yaml      file → Unity layer + RGL material category mapping
meshes_summary.txt        per-mesh point/triangle counts (debug aid)
```

The PCD that `mcap_to_pcd` writes is the **Autoware-side** input (NDT
localization). It does not participate in Unity physics or sensor
simulation. The meshes this stage produces are the missing piece: they give
the wheel raycasts something to roll on, contain the car against falling
off the cliff, and (just as importantly) give AWSIM's RGL lidar simulation
real surfaces to bounce rays off so NDT inside Autoware actually has
features to lock onto.

## Pipeline at a glance

```
data/<track>/cleaned_tf.mcap
    │
    │  mcap_to_pcd/build_map.py
    ▼
map/<track>/{pointcloud_map.pcd,         ← Autoware NDT input
             trajectory_enu.csv,          ← used here as the road centreline
             lanelet2_map.osm,
             map_origin.yaml}
    │
    │  pcd_to_meshes/build_meshes.py     ← THIS DIRECTORY
    ▼
map/<track>/meshes/{ground.obj,
                    wall_left.obj,
                    wall_right.obj,
                    meshes_manifest.yaml}
```

## What the build does

Five passes, all in pure numpy + scipy:

1. **PCD load** — read the binary `pointcloud_map.pcd` produced upstream.
2. **Corridor filter** — keep only points whose XY distance to the trajectory
   polyline is ≤ `--corridor-half-width` (default 12 m). For each kept point
   we also record the signed lateral offset (left of travel = +, right = −)
   and the arclength along the trajectory — used downstream to split walls
   left/right and to embed wall cells back into ENU.
3. **Verticality classification** — for each corridor point, fit a local PCA
   over its k nearest neighbours and read off the surface-normal estimate.
   Points whose normal `|n_z|` is below `--vertical-thresh` (default 0.4,
   ~surfaces tilted within 66° of vertical) are flagged as **walls**;
   the rest are **ground**.
4. **Ground heightmap mesh** — bin ground points into an XY grid
   (`--ground-cell`, default 0.5 m). Per cell, take the **low-percentile
   Z** (`--ground-low-percentile`, default 10th — robust to vegetation /
   parked-car returns). Then in order:

   - drop islands smaller than `--ground-min-component-cells` (default 200) —
     removes scattered dust before the smoother and hole-filler operate,
   - mask-aware box smoothing of the height field (radius 1 cell) —
     averages each populated cell with its populated 3×3 neighbours,
   - **enclosed-hole fill**: `--ground-hole-fill-passes` (default 4) of
     mean-of-neighbours imputation, restricted to empty cells that lie
     inside ENCLOSED holes (cells reachable from the grid boundary
     through empty space are NOT filled — this prevents the road from
     dilating into off-road areas),
   - **permissive triangulation**: each grid quad emits two CCW triangles
     when all 4 corners are populated, and one CCW triangle when exactly
     3 corners are populated. The 3-corner rule covers residual gaps the
     hole-fill couldn't reach (those with only 2 populated 8-neighbours).

5. **Wall meshes (left + right)** — split wall points by sign of lateral
   offset. For each side, bin into an `(arclength, height-above-road)`
   grid (`--wall-arclength-cell` × `--wall-height-cell`, default 1.0 m
   each) and take the per-cell **median lateral offset** as the wall's
   actual transverse position. Then in order:

   - **chain-thickening fill**: pass 1 uses `min_neighbors=2` to reach
     into 1-cell-wide cliff-return chains (the natural shape of striped
     lidar returns on tilted surfaces), subsequent passes use
     `min_neighbors=3`. Total: `--wall-hole-fill-passes` (default 2).
     Unlike the ground variant this is intentional dilation — the (s, h)
     grid's boundaries ARE the wall edges and we want thin curtains to
     thicken into something triangulable,
   - drop islands smaller than `--wall-min-component-cells` (default 50)
     **after** thickening,
   - re-embed each cell into ENU using the trajectory's tangent / normal
     at that arclength,
   - same permissive (3-of-4) triangulation as ground, with winding
     flipped per side so each wall's outward normal faces away from
     the road,
   - **bridge-triangle filter**: any triangle whose longest 3D edge in
     ENU exceeds `--wall-max-edge-m` (default 5 m) is dropped. The
     (s, h) grid is monotonic in arclength, but if the trajectory has
     a GPS dropout or a sharp switchback compresses several cells of
     arclength into a tiny ENU step, two adjacent (s, h) cells can map
     to physically distant world positions and form a long bridge
     triangle that would otherwise act as an unwanted collider /
     lidar-return surface.

The `(arclength, height)` parametrisation is the trick that lets a 2.5D
heightmap mesher handle a curving cliff curtain without 3D surface
reconstruction — the mesh is a heightmap in the road's intrinsic frame,
then warped back into world coordinates.

If a wall side has no surviving cells (rare — straight road sections with
no inboard structure on the inner side), `build_meshes.py` logs a skip
and **deletes any pre-existing `wall_<side>.obj`** in the output dir so
you never import stale geometry from a previous run.

## Install dependencies

```bash
pip install --user -r requirements.txt
```

Python ≥ 3.10. No PCL, no Open3D, no ROS.

## Run

```bash
cd pcd_to_meshes                    # from the repo root

python3 build_meshes.py \
    --pcd ../map/TM99_uphill/pointcloud_map.pcd \
    --trajectory ../map/TM99_uphill/trajectory_enu.csv \
    --out-dir ../map/TM99_uphill/meshes
```

Expected runtime: a couple of minutes for the current TM99 uphill map
(~120 M-point PCD, then a much smaller corridor subset) on a modern laptop.
The verticality PCA is by far the dominant cost (O(N · k log N)); raise
`--vertical-knn` only if you need smoother normals.

## Useful flags

| flag | purpose |
| --- | --- |
| `--corridor-half-width`        | XY half-width of the corridor (m). Default 12. Bigger captures more cliff but adds noise. |
| `--corridor-max-height`        | Max metres above local road Z to keep. Default 30. Raise on sites with tall cliffs. |
| `--corridor-max-depth`         | Max metres below local road Z to keep. Default 10. The downhill side's wall lives here. |
| `--vertical-knn`               | k for local PCA. Default 16. |
| `--vertical-thresh`            | `|n_z|` below this is "vertical". Default 0.4 ≈ 66° off horizontal. Lower = stricter (fewer walls). |
| `--ground-cell`                | Ground heightmap cell (m). Default 0.5. |
| `--ground-low-percentile`      | Per-cell Z percentile to take as ground. Default 10. Raise if input is already ground-only. |
| `--ground-smooth`              | Box-smoothing radius in ground cells. Default 1. 0 disables. |
| `--ground-min-component-cells` | Drop ground islands smaller than this many cells. Default 200. |
| `--ground-hole-fill-passes`    | Hole-fill passes on the ground heightmap mask before triangulation. Default 4 — bridges holes up to ~4 cells (~2 m at 0.5 m cells). 0 disables. |
| `--wall-arclength-cell`        | Arclength bin (m) for walls. Default 1.0. |
| `--wall-height-cell`           | Vertical bin (m) for walls. Default 1.0 — anything smaller makes the (s, h) heightmap too sparse for the triangulator to bridge gaps. |
| `--wall-max-height`            | Clip wall above this height (m). Default 30. |
| `--wall-min-component-cells`   | Drop wall islands smaller than this many cells. Default 50. |
| `--wall-hole-fill-passes`      | Hole-fill passes on the (s, h) wall mask before triangulation. Default 2. 0 disables. |
| `--wall-max-edge-m`            | Drop any wall triangle whose longest 3D edge exceeds this (m). Catches "bridge" triangles spanning trajectory discontinuities. Default 5. Set <= 0 to disable. |
| `--no-normals`                 | Skip per-vertex normals (smaller files; Unity will compute). |
| `--max-points`                 | Hard cap on PCD points loaded (debug aid). |

## Loading in AWSIM

The deliverables drop directly into a Unity scene; no asset bundling needed.

```
my_track_scene/
├── ground.obj            → MeshCollider, layer "Ground"
├── wall_left.obj         → MeshCollider, layer "Wall"
├── wall_right.obj        → MeshCollider, layer "Wall"
└── meshes_manifest.yaml  → reference for layer assignment
```

In the Unity editor:

1. Drop each `.obj` into `Assets/`. Unity creates a prefab automatically.
2. For each prefab: select it, set the **Layer** to `Ground` (for the
   road) or `Wall` (for the cliff/parapet) per `meshes_manifest.yaml`.
   AWSIM's vehicle controller wheel raycast mask is set to the `Ground`
   layer by default — meshes on any other layer will be invisible to the
   wheels.
3. Add a `MeshCollider` component to each prefab. Convex = off (these are
   environment meshes, not physics props).
4. (Optional) Open AWSIM's RGL Mesh Material Properties asset and assign
   the `rgl_material` from the manifest (`asphalt`, `rock`, `metal`).
   This shapes the simulated lidar intensity returns so they better match
   the real-bag intensity that NDT was tuned against.

The ENU origin of these meshes matches the origin in `map_origin.yaml`,
so they slot in alongside `pointcloud_map.pcd` with no transform.

Check `meshes_summary.txt` after generation. For a usable AWSIM scene,
`ground.obj` should have dense triangle coverage along the route, and each
`wall_*.obj` should have enough triangles to represent continuous barriers
or cliff faces where the source PCD actually contains vertical structure.

## Caveats

- **Smoothness vs. fidelity.** Heightmap meshes are watertight along
  the road but step-wise on tight switchbacks (each cell has one Z).
  Drop `--ground-cell` to 0.25 m if you need finer geometry; runtime
  and OBJ size grow ~4×.
- **Walls assume the road has structure on both sides.** On open
  sections (a parking lot, the start/finish straight) the wall mesher
  may produce a thin, sparse curtain or skip a side entirely.
  Component filtering (`--wall-min-component-cells`) helps prune dust.
- **Verticality classifier ignores semantics.** Tree trunks, lamp
  posts, parked cars, large boulders — anything with a locally vertical
  surface — will end up classified as wall. For a sterile hillclimb
  road this is mostly fine; on urban tracks you'll want to add a
  semantic pre-filter.
- **No textures / materials in the OBJ.** These meshes are colliders +
  RGL hit surfaces, not visual hero assets. If you want a textured road
  for the visual scene, lay a separate decal mesh on top in Unity.

## Files

```
build_meshes.py          orchestrator + CLI
lib_pcd_io.py            binary-PCD reader + trajectory CSV + corridor filter
lib_ground.py            XY heightmap → ground triangle mesh
lib_walls.py             PCA verticality + per-side (arclength, height) wall mesh
lib_obj.py               minimal OBJ writer + per-vertex normal estimator
requirements.txt
```
