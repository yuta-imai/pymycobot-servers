# MyCobot Pose Visualizer (`webui/`)

A self-contained 3D web app that shows the live robot pose, the URDF kinematic
model, the firmware-reported end-effector pose, and a pick-target marker. It
fetches everything from the Pi's REST API over plain HTTP GET.

No build step and no framework: a single `index.html` + `app.js` using
[three.js](https://threejs.org/) loaded from a CDN via an ES-module import map.

## Running

Served same-origin by the REST API server (recommended):

```
python mycobot_api_server.py        # on the Pi
# then open  http://<pi>:8080/ui/
```

The server mounts this folder at `/ui` (see the `StaticFiles` mount in
`mycobot_api_server.py`). Same-origin means no CORS concerns.

Or host the folder anywhere static (it is fully portable):

```
cd webui && python3 -m http.server 8745
# open http://localhost:8745/  and set "API base" to http://<pi>:8080
```

CORS is already `*` on the API server, so a remote static host works too.

## What it shows

- **Kinematic skeleton** — bones + per-link coordinate frames, driven by URDF
  forward kinematics from `GET /joints/angles` (degrees). No mesh files ship
  with the URDF, so links are drawn as frames/bones rather than solid meshes.
- **URDF FK end-effector** (blue marker) vs **firmware `GET /robot/coords`**
  (orange marker). Showing both side by side makes the known URDF-wrist vs
  physical-wrist discrepancy visible at a glance.
- **Gripper** state from `GET /gripper/status`.
- **Pick target** (yellow marker) — manual `x/y/z` in mm (base frame), or
  fetched from an optional targets URL. Designed to be auto-filled by a future
  `/vision/targets` endpoint. Expected JSON: `{x, y, z}` /  `[x, y, z]` /
  `{targets: [{position: {x, y, z}}]}`.
- **Camera** — point the snapshot/MJPEG URL field at any image stream.

Connection settings, target, and camera URL persist in `localStorage`.

## Data contracts (from `mycobot_api_spec.yaml`)

| Endpoint              | Used for                          | Units      |
|-----------------------|-----------------------------------|------------|
| `GET /joints/angles`  | `{angles: [6]}` → joint FK        | degrees    |
| `GET /robot/coords`   | `{coords: [x,y,z,rx,ry,rz]}`      | mm / deg   |
| `GET /gripper/status` | gripper readout                   | —          |

## Tuning

If a joint visually rotates the wrong way relative to the hardware, adjust
`ARM_SIGN` / `ARM_OFFSET` in `app.js`. The `angles[]` → URDF-joint mapping is
`ARM_JOINTS` (J1..J6). The visualized end-effector link is `TCP_LINK`.

The bundled `mycobot.urdf` is a cleaned copy of
`urdf/mycobot_280m5_with_gripper_parallel.urdf` (a stray merge-conflict marker
removed) so the app stays portable. `app.js` also defensively truncates URDF
text at `</robot>`.

## Debug hook

The page exposes `window.viz` for driving the model without the API:

```js
viz.setPose([0, 30, -60, 0, 40, 0])   // degrees J1..J6 → returns TCP mm
```
