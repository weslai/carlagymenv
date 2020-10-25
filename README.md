# carlagymenv

This is a Carla Simulator and Gym Carla Environment 

## Setup 
### Carla Simulator 
Please go to [Carla Simulator](https://github.com/carla-simulator/carla) Website to install Carla 
For this repository, the version **Carla 0.9.9.4** is used. 

- Unpack the Download .tar.gz File 
- '''chmod +x CarlaUE4.sh''' 
- Get some try with the codes under '''/PythonAPI/examples''' 
- '''/PythonAPI/codes/egovehicle_radius.py''' is the code, which should work for our need. 

How to run Carla 
----------------

Launch a terminal in this folder and execute the simulator by running 

```sh
 ./CarlaUE4.sh
```

This will launch a window with a view over the city. This is the view, you can fly around the city using the mouse and WASD keys. 
This simulator acts now as a server, waiting for a client to connect and interact. 

Try an example to interact with the server 

```sh
 ./spawn_npc.py -80
```

This should launch 80 Vehicles into the simulator. 

Try also manual control the vehicle by running 

```sh
 ./manual_control.py
```

Now we can test '''egovehicle_radius.py''' with Town04 by running 

```sh
 ./egovehicle_radius.sh 
```

### Gym Environment 
Please go to [gym github website](https://github.com/openai/gym) Website to clone openai gym 
***Gym environment here clone to the folder /PythonAPI/codes/ inside the carla*** 

```
git clone https://github.com/openai/gym 
```

Now go inside the gym folder under /gym/envs/, create a new folder **carla** for the carla gym environment 
For the Carla Environment, they should two files unter the folder **carla**, **__init__.py** and **carla_env** 

the file /gym/envs/carla/__init__.py should have: 
```
from gym.envs.carla.carla_env import CarlaEnv
```

And also /gym/envs/__init__.py should have:  
```
register(
    id='carla-v0',
    entry_point='gym.envs.carla:CarlaEnv',
    kwargs = {'host':'127.0.0.1', 'port': 2000, 'tm_p': 8000, 'filtervehicle':'vehicle.*', 'coord_file': 'map04_coordination_1.csv',
              'map_name':'Town04', 'num_vehicle': 10, 'velocity': 80, 'autopilot': True},
)
register(
    id='carla-v1',
    entry_point='gym.envs.carla:CarlaEnv',
    kwargs = {'host':'127.0.0.1', 'port': 2000, 'tm_p': 8000, 'filtervehicle':'vehicle.*', 'coord_file': 'map04_coordination_1.csv',
              'map_name':'Town04', 'num_vehicle': 10, 'velocity': 90, 'autopilot': False},
)
```

Under /gym/ in terminal, by running:
```
pip install -e .
```
This should be run, each time if the environment is changed.
### Python Packages 
Create a Python Conda Environment, by using 
```
conda create --name carlagym 
conda activate carlagym
```

install python packages with /PythonAPI/codes/requirements.txt: 
```
pip install -r requirements.txt
```
## Hierarchy 
Under **/codes/simulate/**: 
egovehicle_radius.py: 
Basically, we transformed this file to the carla_env.py, it measured the 15 Cars surrounding around the ego vehicle. 

tm_audi.py: 
Try out with Traffic Manager and ego vehicle, played with parameters. 

view_spawn_points.py: 
It can save the coordination points of the vehicles in 4 lanes, and also to view the trajectory of the vehicles(visualization) 

view_way_points.py: 
Just plot out the way points 

Under **/gym/gym/envs/carla**
carla_env.py: 
This is the gym Environment for carla, Collision Sensor haven't be fixed yet.
