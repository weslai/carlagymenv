#!/usr/bin/env python

## Fraunhofer IIs
## Wei-Cheng Lai
## Mostly base on Spawn_npc.py from tutorial

### Carla Traffic Manager
### View the spawn points 
### Save the sparn points in .csv file in order to be loaded as 
### the start points(x,y,z) for the vehicles

### the default Map is Town04
import glob
import os
import sys
import time

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import argparse
import logging
import random
import pandas as pd

def main():
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '--save_coordinate',
        default=False,
        type=bool,
        help='savw the point coodinate for four lanes'
    )
    argparser.add_argument(
        '--draw_spawn_points',
        default=False,
        type=bool,
        help='draw the point coodinate on the map'
    )
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1280x720',
        help="window resolution (default: 1280x720)")
    argparser.add_argument(
        '--filterv',
        metavar='PATTERN',
        default='vehicle.*',
        help='vehicles filter (default: "vehicle.*")')
    argparser.add_argument(
        '-tm_p', '--tm_port',
        metavar='P',
        default=8000,
        type=int,
        help='port to communicate with TM (default: 8000)')
    argparser.add_argument(
        '--sync',
        action='store_true',
        help='Synchronous mode execution')
    argparser.add_argument(
        '-m', '--map_name',
        default='Town04',
        type=str,
        help='map name to load in the server (default: Town01)')

    args = argparser.parse_args()
    args.width, args.height = [int(x) for x in args.res.split('x')]

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    client = carla.Client(args.host, args.port)
    client.load_world(map_name=args.map_name)
    client.set_timeout(10.0)

    ## Datasets path
    path_dataset = os.getcwd() + "../Datasets/"
    dataset_name = "map04_coordination_1.csv"
    path_dataset = path_dataset + dataset_name

    ## To check whether the dataset exists 
    ## if not, new file should be created
    if os.path.isfile(path_dataset):
        newfile = False
    else:
        newfile = True

    try:
        traffic_manager = client.get_trafficmanager(args.tm_port)
        traffic_manager.set_global_distance_to_leading_vehicle(4.0)
        world = client.get_world()

        synchronous_master = False

        if args.sync:
            settings = world.get_settings()
            traffic_manager.set_synchronous_mode(True)
            if not settings.synchronous_mode:
                synchronous_master = True
                settings.synchronous_mode = True
                settings.fixed_delta_seconds = 0.05
                world.apply_settings(settings)
            else:
                synchronous_master = False

        blueprints = world.get_blueprint_library().filter(args.filterv)

        spawn_points = world.get_map().get_spawn_points()
        number_of_spawn_points = len(spawn_points)

        ## for 4 lanes that we can mark the points that they drive through
        blueprint_toyota = None  ## for toyota prius 4 Cars
        vehicle_actors = []      ## all vehicle actors

        ## set the starting point on the highway (transform)
        ## here we already gave the location (x=-5.9, y=150.15, z=0.2819)
        transform = carla.Transform(carla.Location(x=-5.9, y=150.15, z=0.2819), carla.Rotation(pitch=0, yaw=90, roll=0))
        if blueprint_toyota is None: 
            ## create new actors (four toyotas)
            blueprint_toyota = world.get_blueprint_library().filter('vehicle.toyota.prius')   ## actorblueprint, transform, rotation
            blueprint_toyota1 = world.get_blueprint_library().filter('vehicle.toyota.prius')
            blueprint_toyota2 = world.get_blueprint_library().filter('vehicle.toyota.prius')
            blueprint_toyota3 = world.get_blueprint_library().filter('vehicle.toyota.prius')
            bp_toyota_list = [blueprint_toyota, blueprint_toyota1, blueprint_toyota2, blueprint_toyota3]
            actor_toyota_list = [blueprint_toyota[0], blueprint_toyota1[0], blueprint_toyota2[0], blueprint_toyota3[0]]

            actor_toyota_list[0].set_attribute('role_name', 'autopilot')  # set the autopilot  ## hero?
            actor_toyota_list[1].set_attribute('role_name', 'autopilot')  # set the autopilot  ## hero?
            actor_toyota_list[2].set_attribute('role_name', 'autopilot')  # set the autopilot  ## hero?
            actor_toyota_list[3].set_attribute('role_name', 'autopilot')  # set the autopilot  ## hero?
            actor_toyota_list[0].set_attribute('color', '0,255,0')  # set the color of the toyota0
            actor_toyota_list[1].set_attribute('color', '0,255,255')  # set the color of the toyota1
            actor_toyota_list[2].set_attribute('color', '255,255,255')  # set the color of the toyota2
            actor_toyota_list[3].set_attribute('color', '100, 100, 100')  # set the color of the toyota3

            # four toyotas should be set parallel at the starting points
            print(transform)
            actor_toyota = world.spawn_actor(actor_toyota_list[0], transform)
            transform.location.x = -9.25  # to refer to the position of lane
            print(transform)
            actor_toyota1 = world.spawn_actor(actor_toyota_list[1], transform)
            transform.location.x = -13.03 # to refer to the position of lane
            print(transform)
            actor_toyota2 = world.spawn_actor(actor_toyota_list[2], transform)
            transform.location.x = -16.25 # to refer to the position of lane
            print(transform)
            actor_toyota3 = world.spawn_actor(actor_toyota_list[3], transform)
            vehicle_actors.append(actor_toyota)
            vehicle_actors.append(actor_toyota1)
            vehicle_actors.append(actor_toyota2)
            vehicle_actors.append(actor_toyota3)

            traffic_manager.distance_to_leading_vehicle(actor_toyota,
                                                        2.0)  ## set the minimum distance in meters to keep with the others
            traffic_manager.distance_to_leading_vehicle(actor_toyota1,
                                                        4.0)  ## set the minimum distance in meters to keep with the others
            traffic_manager.distance_to_leading_vehicle(actor_toyota2,
                                                        6.0)  ## set the minimum distance in meters to keep with the others
            traffic_manager.distance_to_leading_vehicle(actor_toyota3,
                                                        8.0)  ## set the minimum distance in meters to keep with the others
            traffic_manager.vehicle_percentage_speed_difference(actor_toyota, 10.0)
            traffic_manager.vehicle_percentage_speed_difference(actor_toyota1, 10.0)
            traffic_manager.vehicle_percentage_speed_difference(actor_toyota2, 10.0)
            traffic_manager.vehicle_percentage_speed_difference(actor_toyota3, 10.0)
            traffic_manager.auto_lane_change(actor_toyota, False)  ### not change the lane (force to ride on highway)?
            traffic_manager.auto_lane_change(actor_toyota1, False)  ### not change the lane (force to ride on highway)?
            traffic_manager.auto_lane_change(actor_toyota2, False)  ### not change the lane (force to ride on highway)?
            traffic_manager.auto_lane_change(actor_toyota3, False)  ### not change the lane (force to ride on highway)?
            actor_toyota1.set_autopilot()
            actor_toyota.set_autopilot()
            actor_toyota2.set_autopilot()
            actor_toyota3.set_autopilot()
            traffic_manager.ignore_lights_percentage(actor_toyota, 100)
            traffic_manager.ignore_lights_percentage(actor_toyota1, 100)
            traffic_manager.ignore_lights_percentage(actor_toyota2, 100)
            traffic_manager.ignore_lights_percentage(actor_toyota3, 100)

            # for save the coordination of each lane in the csv. file 
            counter = 0
            frame = 0  ## for the csv file, record each frame
            last_time = time.time()
            data_columns = ['Frame', 'lane1 X', 'lane1 Y', 'lane1 Z', 'lane1 pitch', 'lane1 yaw', 'lane1 roll', 'lane2 X', 'lane2 Y', 'lane2 Z', 'lane2 pitch', 'lane2 yaw', 'lane2 roll', 'lane3 X', 'lane3 Y',
                            'lane3 Z', 'lane3 pitch', 'lane3 yaw', 'lane3 roll', 'lane4 X', 'lane4 Y', 'lane4 Z', 'lane4 pitch', 'lane4 yaw', 'lane4 roll']
            while True:
                if args.sync and synchronous_master:
                    now = time.time()
                    if counter > 1:
                        print('interval : {}'.format(now - last_time))
                        last_time = now
                    counter += 1

                    if args.save_coordinate: 
                        if counter % 30:
                            data_list = [frame, actor_toyota.get_transform().location.x, actor_toyota.get_transform().location.y, actor_toyota.get_transform().location.z,
                                         actor_toyota.get_transform().rotation.pitch, actor_toyota.get_transform().rotation.yaw, actor_toyota.get_transform().rotation.roll,
                                         actor_toyota1.get_transform().location.x, actor_toyota1.get_transform().location.y, actor_toyota1.get_transform().location.z,
                                         actor_toyota1.get_transform().rotation.pitch, actor_toyota1.get_transform().rotation.yaw, actor_toyota1.get_transform().rotation.roll,
                                         actor_toyota2.get_transform().location.x, actor_toyota2.get_transform().location.y, actor_toyota2.get_transform().location.z,
                                         actor_toyota2.get_transform().rotation.pitch, actor_toyota2.get_transform().rotation.yaw, actor_toyota2.get_transform().rotation.roll,
                                         actor_toyota3.get_transform().location.x, actor_toyota3.get_transform().location.y, actor_toyota3.get_transform().location.z,
                                         actor_toyota3.get_transform().rotation.pitch, actor_toyota3.get_transform().rotation.yaw, actor_toyota3.get_transform().rotation.roll]

                            toyota_locations_df = pd.DataFrame([data_list], columns=data_columns) # transform list to dataframe
                            frame += 1
                            # save the dataframe 
                            if newfile:
                                toyota_locations_df.to_csv(path_dataset, index=False)
                                newfile = False
                            else:
                                toyota_locations_df.to_csv(path_dataset, header=None, mode='a', index=False)

                    # to draw spawn points
                    # spawn points are not equal to the coordination we are saving
                    if args.draw_spawn_points:
                        for waypoint in spawn_points:
                            print(type(waypoint.location))
                            world.debug.draw_string(waypoint.location, 'x:{}, y:{}, z:{}'.format(waypoint.location.x, waypoint.location.y, waypoint.location.z),
                                                    draw_shadow=True,
                                                    color=carla.Color(r=0, g=255, b=0), life_time=100,
                                                    persistent_lines=True)

                            # life_time=0.2) ## to view the points for the start
                            time.sleep(2)
                    world.tick()

    finally:

        if args.sync and synchronous_master:
            settings = world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            world.apply_settings(settings)

        time.sleep(0.5)

if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
