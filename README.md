
> Submarine Control with ROS 2, MAVROS, and ArduSub SITL

This project demonstrates how to simulate and control an underwater vehicle using ArduSub SITL, ROS 2 Humble, and MAVROS. It includes:

- A ROS 2 workspace with MAVROS packages
- A Python script to control the vehicle (arm, change mode, moves to a specific point, returns to origin)
- Setup instructions for running the simulation and integration

---

> Project Structure

```
edhitha_ws/
├── src/
│   └── amrutha/
│       └── submarine.py     
├── log/                     
├── install/                 
├── build/                   
├── .gitignore
└── README.md
```

---

> Setup Instructions

1. Clone this repository

```bash
git clone https://github.com/amruthaaraballi/submarine.git
cd submarine
```

2. Install dependencies

Make sure you have the following installed:

- ROS 2 Humble
- MAVROS and MAVROS extras
- ArduPilot with ArduSub SITL
- QGroundControl (optional)

3. Build the workspace

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

---

> How to Run the Simulation

Open **four terminals** and run the following commands:

> Terminal 1 – ArduSub SITL

```bash
cd ~/ardupilot
sim_vehicle.py -v ArduSub -f vectored --console --map --out=127.0.0.1:14550
output add 127.0.0.1:14540  ( used to forward MAVLink data from SITL to MAVROS, enabling communication between the simulator and ROS 2 nodes)

```

> Terminal 2 – MAVROS bridge

```bash
ros2 launch mavros apm.launch fcu_url:=udp://:14540@127.0.0.1:14557
```

> Terminal 3 – submarine.py (python script)

```bash
cd ~/edhitha_ws
nano src/amrutha/submarine.py  # Edit your script if needed

colcon build --packages-select amrutha
source install/setup.bash
chmod +x src/amrutha/submarine.py

ros2 run amrutha submarine
```

> Terminal 4 – Monitor MAVROS state

```bash
ros2 topic echo /mavros/state
```

---

> What the Script Does

- Arms the vehicle
- Switches to 'GUIDED' mode
- Commands it to dive to a specific depth or to a specific point 
- Moves the submarine back to origin
---

> Notes

- `build/` and `install/` directories are excluded from version control using `.gitignore`
- Files larger than 100MB are removed using `git-filter-repo`
- Make sure your PX4/ArduPilot SITL is properly sending data to MAVROS

---

