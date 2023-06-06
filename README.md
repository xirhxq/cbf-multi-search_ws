# cbf-search-ral_ws

The ROS workspace for cbf-multi-uav-searching demo in RAL letter.

## Equipment

- DJI Matrice 210
- NVIDIA NX

## File Structure

- dji-osdk for core comm. to DJI M210
- dji-osdk-ros for ros interface
- cbf-search-ral_ws for controlling and strategy planning

## Installing zmq and pyzmq offline (on NX)

```bash
cd src/zmq_ros_bridge
pip3 install --no-index --find-links=pip3_zmq -r requirements.txt
```