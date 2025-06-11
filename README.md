# Mapping-with-T265-and-2dLidar-ROS2


first of all we need to build our docker(this is just for first time):

```
-- docker build -f ros2_slam.dockerfile -t ros2_slam .
```


then: 

```
-- xhost +local:root
```

and then we should run our builded docker: 


```
docker run --rm -it \
  --name ros2_slam_gui \
  --privileged \
  --net host \
  -e ROS_DOMAIN_ID=17 \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/ros2_ws/config:/ros2_ws/config:ro \
  -v ~/ros2_ws/mapping_docker:/ros2_ws/src/mapping_docker \
  --device=/dev/bus/usb/004/006 \
  ros2_slam
```




and then for launch the mapping:

```
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
```
```
cd /ros2_ws
colcon build --packages-select mapping_docker
source install/setup.bash
```
and for final step we need to run our launch file: 

```
ros2 launch mapping_docker slam.launch.py
```
------------------------------------------------------------------------------------

if you get this following error you have to launch the node again

** Message Filter dropping message: frame 'odom' at time  for reason 'discarding message because the queue is full' **

------------------------------------------------------------------------------------




or we should have extra 6 terminal and:


-- docker exec -it ros2_slam_gui bash


-- source /opt/ros/humble/setup.bash
 source /ros2_ws/install/setup.bash
 
 
 and then run this commands properly in each window: 


-- ros2 run realsense2_camera realsense2_camera_node    --ros-args      -p enable_pose:=true      -p publish_odom_tf:=true      -p base_frame_id:=base_link      -p odom_frame_id:=odom



-- ros2 launch urg_node2 urg_node2.launch.py   serial_port:=/dev/ttyACM0 


-- ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link


-- ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 base_link laser



-- ros2 launch robot_localization ekf.launch.py   params_file:=/ros2_ws/config/ekf.yaml


-- ros2 launch slam_toolbox online_async_launch.py   slam_params_file:=/ros2_ws/config/slam_toolbox_params.yaml



---

# Remote‑mapping NUC ↔ Laptop  – SSH & ROS 2 Network Cheat‑Sheet

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


