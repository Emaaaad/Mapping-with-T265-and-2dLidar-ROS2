# Mapping‑with‑T265‑and‑2D‑Lidar (ROS 2)

> **What is this?** A minimal end‑to‑end SLAM stack that fuses **Intel® RealSense T265** visual‑inertial odometry with any **2‑D lidar** (we ship examples for a Hokuyo URG‑04LX, but any laser scanner that publishes a standard `sensor_msgs/LaserScan` will work).  Everything runs inside a self‑contained **Docker** image based on ROS 2 Humble, so you can deploy on a NUC, Jetson or plain laptop without touching the host filesystem.
>
> **Why docker?** • identical dev/runtime environment • quick rollback • easy to copy to other robots • avoids host‑level driver headaches.  If you prefer running natively, just install the same ROS 2 packages and copy the `mapping_docker` workspace – the launch files don’t care whether they run in a container or on bare metal.
>
> **Hardware tested**
>
> * Intel RealSense T265 (USB‑C)
> * Hokuyo URG‑04LX (USB serial)
> * Intel NUC 11 running Ubuntu 20.04 host, container uses Ubuntu 22.04 base
>
> **Works with other lidars** – just adapt the static TF (`base_link → laser`) and the `/scan` topic; no changes to ekf / slam\_toolbox required.

---

## 1 ▪ Build the Docker image (first time only)

```bash
cd ~/ros2_ws/mapping_docker
xhost +local:root              # allow the container to talk to your X server

# one‑time build (~15 min on first run)
docker build -f ros2_slam.dockerfile -t ros2_slam .
```

## 2 ▪ Run the container

```bash
docker run --rm -it \
  --name ros2_slam_gui \
  --privileged \
  --net host \
  -e ROS_DOMAIN_ID=17 \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/ros2_ws/config:/ros2_ws/config:ro \
  -v ~/ros2_ws/mapping_docker:/ros2_ws/src/mapping_docker \
  --device /dev/bus/usb \  # or /dev/ttyACM0 only
  ros2_slam
```

*Adjust `--device` for your laser (e.g. `/dev/ttyACM0`) and camera if needed.*

## 3 ▪ Build the workspace inside the container (first run only)

```bash
source /opt/ros/humble/setup.bash
cd /ros2_ws
colcon build --packages-select mapping_docker
source install/setup.bash
```

## 4 ▪ Launch the full stack

```bash
ros2 launch mapping_docker slam.launch.py
```

RViz opens automatically with a pre‑configured view.  Drive the robot – the occupancy grid (`/map`) will update in real time.

---

### Troubleshooting

| Symptom                                                             | Quick fix                                                                                                                                                                              |
| ------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **`Message Filter dropping message: frame 'odom' … queue is full`** | Restart `slam_toolbox` (or just relaunch step 4). Happens if TF tree is incomplete at startup.                                                                                         |
| **Map does not grow / stays blank**                                 | Check `/scan` and `/odometry/filtered` are publishing (`ros2 topic hz …`). Verify `t265_pose → base_link` and `base_link → laser` static TFs exist (`ros2 run tf2_tools view_frames`). |
| **Authentication error when pushing to GitHub**                     | Use a personal access token instead of a password (see GitHub docs).                                                                                                                   |

---

### Optional ▪ Run nodes manually (debugging)

Open **six terminals** inside the container (`docker exec -it ros2_slam_gui bash`) and in each:

```bash
# 1 T265 driver
ros2 run realsense2_camera realsense2_camera_node \
   --ros-args -p enable_pose:=true -p publish_odom_tf:=true \
   -p base_frame_id:=base_link -p odom_frame_id:=odom

# 2 Lidar driver
ros2 launch urg_node2 urg_node2.launch.py serial_port:=/dev/ttyACM0

# 3 Static TFs
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 base_link laser

# 4 EKF
ros2 launch robot_localization ekf.launch.py params_file:=/ros2_ws/config/ekf.yaml

# 5 SLAM Toolbox
ros2 launch slam_toolbox online_async_launch.py \
   slam_params_file:=/ros2_ws/config/slam_toolbox_params.yaml

# 6 RViz
rviz2 -d /ros2_ws/config/slam_setup.rviz
```

### Troubleshooting

|  Symptom                                            |  Likely cause & quick fix                                                                             |
| --------------------------------------------------- | ----------------------------------------------------------------------------------------------------- |
|  `Message Filter dropping message … queue is full`  | SLAM Toolbox is waiting for TFs – just restart `slam.launch.py`.                                      |
|  No GUI appears                                     | Ensure `xhost +local:root` ran **before** `docker run`, and `DISPLAY` & X11 volume flags are present. |
|  "No RealSense devices were found"                  | Add `--privileged` **and** include the librealsense udev rule step in Dockerfile.                     |
|  Device permission error                            | Mount the specific device: `--device /dev/ttyACM0` (Hokuyo) or keep `--device /dev/bus/usb`.          |

---

## Remote‑mapping NUC ↔ Laptop – SSH & ROS 2 Network Cheat‑Sheet

---

## 0. Prerequisites

| Machine               | Needs                                             | Notes                                                                             |
| --------------------- | ------------------------------------------------- | --------------------------------------------------------------------------------- |
| **NUC (robot‑side)**  | Ubuntu ≥ 20.04, `openssh‑server`, Wi‑Fi adapter   | The NUC will host the Wi‑Fi hotspot **and** run your ROS 2 stack (inside Docker). |
| **Laptop (operator)** | Any OS with `ssh`, ROS 2 Humble (for rviz2 & CLI) | Connects to the hotspot, visualises the map, sends cmd\_vel.                      |

```bash
# install sshd on the NUC if missing
sudo apt update && sudo apt install openssh-server -y
```

---

## 1. Turn the NUC into a Wi‑Fi hotspot

1. **Settings ▸ Wi‑Fi ▸ ⋮  ▸ Turn On Hotspot**
   Ubuntu auto‑creates an SSID like **“<hostname> Hotspot”**; keep/modify password.
2. Keep that window—the hotspot stops if you disable Wi‑Fi or reboot (see § 6 to persist).

---

## 2. Connect the laptop

*Select the hotspot SSID* ➜ enter password ➜ wait until you get an IP (usually `10.42.0.x`).

Check connectivity:

```bash
ping 10.42.0.1        # NUCʼs default hotspot address
nc -vz 10.42.0.1 22   # confirm SSH port open
```

---

## 3. Key‑based SSH (one‑time)

```bash
# on laptop ------------------------------------------------------------------
ssh-keygen -t ed25519 -C "laptop@mapping"          # press <Enter> through prompts
ssh-copy-id nuc@10.42.0.1                          # upload pub‑key to the NUC
ssh nuc@10.42.0.1                                  # first password‑less login
```

*(Optional)* add a short‑hand to `~/.ssh/config`:

```sshconfig
Host nuclidar               # any alias you like
  HostName 10.42.0.1
  User nuc
  IdentityFile ~/.ssh/id_ed25519
```

Now you can simply `ssh nuclidar`.

---

## 4. ROS 2 environment

Append to **both** machinesʼ `~/.bashrc`:

```bash
export ROS_DOMAIN_ID=17          # pick a number 0‑101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp  # explicit but optional
```

Reload: `source ~/.bashrc`

No extra firewall rules are needed if `ufw` is disabled; otherwise allow UDP 7400‑7600.

---

## 5. Launching the stack

On the **NUC (SSH session or VS Code Remote‑SSH)**

```bash
cd ~/ros2_ws/mapping_docker
# run the docker image that talks to Hokuyo + T265
./run_slam.sh   # your long docker run … line here
```

On the **laptop**

```bash
rviz2 -d ~/ros2_ws/config/slam_setup.rviz
```

You should see live `/scan`, `/map`, `/tf`, and can send `/cmd_vel` from your control node.

---

## 6. (Option) make the hotspot persistent

Ubuntu **Settings ▸ Connections ▸ Wi‑Fi** → select the hotspot → ⚙️  → toggle **Connect automatically** & **All users**.
Alternatively enable `netplan` or a systemd‑networkd AP unit.

---

### Troubleshooting quick table

| Symptom                             | Fix                                                                                   |
| ----------------------------------- | ------------------------------------------------------------------------------------- |
| Laptop can’t SSH                    | verify `ping`, `nc -vz 22`; check `sshd` status.                                      |
| ROS 2 topics not seen               | confirm both machines share the same `ROS_DOMAIN_ID`; try `ros2 topic list`.          |
| TF tree split (no map → base\_link) | ensure static TFs for **t265\_pose↔base\_link** and **base\_link↔laser** are running. |

---


