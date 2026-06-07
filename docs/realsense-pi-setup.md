# Intel RealSense D435 on the Raspberry Pi — build & setup (reproducible)

Records how `pyrealsense2` (librealsense) was built on the robot host so a new site
can reproduce it. Done 2026-06-07 for the gripper-challenge vision stack.

## Target / why source build
- Host: Raspberry Pi 4 Model B, 4 GB, **Debian 12 bookworm, aarch64**, kernel 6.12.
- Python: project venv `~/arms/.venv` (Python 3.11).
- There is **no prebuilt `pyrealsense2` wheel for ARM/aarch64**, so build librealsense
  from source with the Python bindings. On the Pi use the **RSUSB (libuvc) backend**
  (`FORCE_RSUSB_BACKEND=ON`) — userspace USB, **no kernel patching**.
- Depth is computed on the camera's D4 ASIC; the Pi only receives RGB+Depth frames,
  so a Pi 4 is enough for depth-based vision (no heavy ML).

## Prerequisites
- `sudo` (passwordless on this host), ~2 GB free disk, USB3 port (blue) for the camera.
- D435 enumerates as USB id `8086:0b07` (`lsusb | grep 8086`).
  - **Use a blue USB3 port + USB3 cable.** On USB2 it shows up at 480M and is
    bandwidth-limited (lower res/fps still works, e.g. 640x480@6).

## Build (the script we ran: `~/build_rs.sh`, launched with nohup)
```bash
#!/bin/bash
set -e
VENV=/home/factory/arms/.venv
sudo DEBIAN_FRONTEND=noninteractive apt-get update -y
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y \
  cmake libusb-1.0-0-dev libssl-dev libudev-dev pkg-config python3-dev
cd ~
[ -d librealsense ] || git clone --depth 1 --branch v2.55.1 \
  https://github.com/IntelRealSense/librealsense.git
cd librealsense && mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DFORCE_RSUSB_BACKEND=ON \
  -DBUILD_PYTHON_BINDINGS=ON -DPYTHON_EXECUTABLE=$VENV/bin/python \
  -DBUILD_EXAMPLES=OFF -DBUILD_GRAPHICAL_EXAMPLES=OFF -DBUILD_TOOLS=ON \
  -DCMAKE_INSTALL_PREFIX=/usr/local
make -j3                         # ~30 min on a Pi4; -j3 keeps <2GB RAM (4GB is fine)
sudo make install && sudo ldconfig
# non-root USB access:
sudo cp ../config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
# put the binding into the venv (built as cpython-311 .so):
SITE=$($VENV/bin/python -c "import site; print(site.getsitepackages()[0])")
find ~/librealsense/build -name "pyrealsense2*.so" -exec cp {} "$SITE/" \;
```
Notes:
- `-j3` (not `-j4`) so the C++ build stays under ~2 GB RAM on the 4 GB Pi.
- Binding `.so` is `wrappers/.../Release/pyrealsense2.cpython-311-aarch64-linux-gnu.so`;
  it's compiled against the venv's Python 3.11 (same ABI as system 3.11), and copied
  into the venv `site-packages`. `librealsense2.so` goes to `/usr/local/lib` (ldconfig).
- If the camera was plugged in before the udev rules, replug it (or rerun
  `udevadm trigger`) so non-root access takes effect; otherwise run as root once.

## Verify
```bash
~/arms/.venv/bin/python -c "import pyrealsense2 as rs; print(rs.__version__)"   # -> 2.55.1
rs-enumerate-devices            # lists the D435 (installed to /usr/local/bin)
```
Capture smoke-test (RGB + Depth, USB2-safe profile):
```python
import pyrealsense2 as rs, numpy as np
pipe, cfg = rs.pipeline(), rs.config()
cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 6)
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 6)
prof = pipe.start(cfg)
for _ in range(10):
    fs = pipe.wait_for_frames(5000)          # let auto-exposure settle
depth = np.asanyarray(fs.get_depth_frame().get_data())   # uint16, millimetres-ish
color = np.asanyarray(fs.get_color_frame().get_data())   # HxWx3 BGR
scale = prof.get_device().first_depth_sensor().get_depth_scale()  # 0.001 m/unit
pipe.stop()
```
Confirmed working: device `Intel RealSense D435`, depth+color 640x480, depth_scale
0.001 m, plausible center depth. (RGB looked dark indoors — tune exposure later;
depth is IR-based and unaffected.)

## Next (vision stack — keep on the Pi, no heavy ML)
- Mount the D435 **fixed**, angled top-down over the work surface, rigid w.r.t. the
  robot base (~40-70 cm). A moving camera/robot invalidates calibration.
- Calibration: ArUco marker on the gripper, drive `solve_topdown_ik` to a grid of
  known command coords, observe the marker in camera 3D, fit **camera-3D -> command
  coords** (FK-independent — sidesteps the unverified absolute position; see
  docs/topdown-wrist-investigation/README.md).
- `locate`: RANSAC the support plane in the point cloud, take points above it as
  objects, cluster -> centroid (x,y) / principal axis (yaw) / top height (z).
- Orchestrate locate -> pick in a single Pi process (owns the serial like the
  wrist_calib scripts), or via the REST API.
