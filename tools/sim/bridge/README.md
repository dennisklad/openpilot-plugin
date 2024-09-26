# Master Thesis
## Concept and Evaluation of a Plugin</br>for enabling Autonomous Driving Simulation in Game Engines
Dionysios Kladis | 391539 | Technische Universität Berlin | Open Distributed Systems | Fakultät IV

--

## Native setup of Openpilot on Ubuntu 24.04

**1. Clone openpilot**

NOTE: This repository uses Git LFS for large files. Ensure you have [Git LFS](https://git-lfs.com/) installed and set up before cloning or working with it.

Either do a partial clone for faster download:
``` bash
git clone --filter=blob:none --recurse-submodules --also-filter-submodules https://github.com/dennisklad/openpilot-plugin.git
```

or do a full clone:
``` bash
git clone --recurse-submodules https://github.com/dennisklad/openpilot-plugin.git
```

**2. Run the setup script**

``` bash
cd openpilot-plugin
git lfs pull
tools/ubuntu_setup.sh
```

Activate a shell with the Python dependencies installed:
``` bash
poetry shell
```

**3. Build openpilot**

``` bash
scons -u -j$(nproc)
```

## Running the bridge

We are going to require two terminals to simulate Openpilot. The first one will launch the Openpilot UI and the second will run the bridge.

1. Open a new terminal and activate the shell:
``` bash
poetry shell
```

2. Navigate to this repository:
``` bash
cd tools/sim/bridge
```

3. Launch the Openpilot UI:
``` bash
./launch_openpilot.sh
```

4.  Repeat steps 1. and 2. and run the bridge:
``` bash
./run_bridge.py --dual_camera
```


## Unity Setup

Create a map and drag the [Bridge.cs](https://github.com/dennisklad/unity-bridge/blob/bf271c500198193c7f9bfbee58fe22642ad96019/Assets/Scripts/Bridge.cs) script to the vehicle you want Openpilot to drive.

Set the required variables as necessary. For a dual camera setup (recommended) two different camera objects are required as references. 
Additionally, we recommend a FOV of 40 degrees for the road camera and 120 degrees for the wide camera. 

After launching the game, a connection should be automatically build by the bridge terminal showing the exact values transmitted between the applications and the Openpilot UI should show the camera feed from Unity.

Finally, pressing the `E` key should engage Openpilot and the vehicle should now be driving automatically.
