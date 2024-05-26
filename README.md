# Kickelhack Challenge - Traversability analysis -

Hackaton solution for the following task:

## Traversability analysis for outdoor/offroad scenarios
### Your mission:
An outdoor robot created a 3D-map of its environment during
an exploration phase.
The environment contains paved but also overgrown
and uneven spaces.
Develop a tool that analyzes the traversability of the
3D-map and visualize the results.
Note here that the robot has a certain
ground clearance and can overcome smaller obstacles.
## Setup

> NOTE: requires python3.8 (or python3.10)!!
>
> AUR: [python38](https://aur.archlinux.org/packages/python38)


### Setup the development environment and install [open3d](https://www.open3d.org)
```bash
python3.8 -m venv python_venv       # create venv
source python_venv/bin/activate     # and use it
pip install open3d                  # install the required library
```

> make sure to always source the venv 
> 
> `source python_venv/bin/activate`


## Usage
TODO design interface
```bash
./test.py point_cloud.pcd 
```
