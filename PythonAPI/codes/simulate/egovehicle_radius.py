#!/usr/bin/env python

## Fraunhofer IIS
## Wei-Cheng Lai
## Mostly base on Spawn_npc.py from tutorial

### Carla Traffic Manager
### Vehicles selfcontrol

import glob
import os
import sys
import time
import pandas as pd
import numpy as np


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
import copy

## utils
from utils.ego_to_exit import get_exit_waypoint

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
        '-n', '--number-of-vehicles',
        metavar='N',
        default=10,
        type=int,
        help='number of vehicles (default: 10), this muss be greater than 1')
    argparser.add_argument(
        '-w', '--number-of-walkers',
        metavar='W',
        default=50,
        type=int,
        help='number of walkers (default: 50)')
    argparser.add_argument(
        '--assigned',
        default=True,
        help='Ego Vehicle assigned')
    argparser.add_argument(
        '--safe',
        action='store_true',
        help='avoid spawning vehicles prone to accidents')
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
        '--filterw',
        metavar='PATTERN',
        default='walker.pedestrian.*',
        help='pedestrians filter (default: "walker.pedestrian.*")')
    argparser.add_argument(
        '-tm_p', '--tm_port',
        metavar='P',
        default=8000,
        type=int,
        help='port to communicate with TM (default: 8000)')
    argparser.add_argument(
        '--hybrid',
        action='store_true',
        help='Enanble')
    argparser.add_argument(
        '--sync',
        action='store_true',
        help='Synchronous mode execution')
    argparser.add_argument(
        '-m', '--map_name',
        default='Town01',
        type=str,
        help='map name to load in the server (default: Town01)')
    argparser.add_argument(
        '--file_name',
        default= 'vehicles_info_car50_velo80_autopilot.csv',
        type=str,
        help='dataset name to be saved')
    argparser.add_argument(
        '--collision_file',
        default='collision_info_car50_velo80_autopilot.csv',
        type=str,
        help='collision dataset name to be saved')
    argparser.add_argument(
        '-r', '--coordination_read',
        dest='coordination_read',
        action='store_true',
        help='read the coordination file')
    argparser.add_argument(
        '-wayr', '--waypoints_read',
        dest='coordination_read',
        action='store_false',
        help='read waypoints')
    argparser.add_argument(
        '--coord_file',
        default='map04_coordination_1.csv',
        type=str,
        help='dataset to be loaded')
    argparser.add_argument(
        '--ego_auto',
        dest='autopilot',
        action='store_true',
        help='autopilot for the ego-vehicle')
    argparser.add_argument(
        '--non_ego_auto',
        dest='autopilot',
        action='store_false',
        help='random lane change for the ego-vehicle')
    argparser.add_argument(
        '--velocity',
        default= 80,
        type=float,
        help= 'velocity (default 80)')
    argparser.set_defaults(autopilot=False)
    argparser.set_defaults(coordination_read=True)

    args = argparser.parse_args()
    args.width, args.height = [int(x) for x in args.res.split('x')]

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    vehicles_list = []
    walkers_list = []
    all_id = []
    client = carla.Client(args.host, args.port)
    client.load_world(map_name=args.map_name)
    client.set_timeout(10.0)

    ## Datasets path
    path_dataset = os.getcwd() + "/Datasets/"
    coord_file = path_dataset + args.coord_file
    path_collision_dataset = path_dataset + args.collision_file
    path_dataset = path_dataset + args.file_name


    if os.path.isfile(path_dataset):
        newfile = False
    else:
        newfile = True

    try:
        traffic_manager = client.get_trafficmanager(args.tm_port)

        ## tm set the global distance to other vehicles
        traffic_manager.set_global_distance_to_leading_vehicle(4.0)

        world = client.get_world()
        map = world.get_map()  ## map
        synchronous_master = False

        if args.sync:   ## synchronous mode
            settings = world.get_settings()
            traffic_manager.set_synchronous_mode(True)
            if not settings.synchronous_mode:
                synchronous_master = True
                settings.synchronous_mode = True
                settings.fixed_delta_seconds = 0.05  ## get a fixed time-step in between frames (0.05 sec) = 20
                world.apply_settings(settings)
            else:
                synchronous_master = False

        blueprints = world.get_blueprint_library().filter(args.filterv)
        blueprintsWalkers = world.get_blueprint_library().filter(args.filterw)
        blueprints_sensors = world.get_blueprint_library().filter('sensor.*.*')

        ## Autobahn -- number of wheels == 4   ## to reject bikes on the highway
        blueprints = [x for x in blueprints if int(x.get_attribute("number_of_wheels")) == 4]

        if args.safe:
            blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
            blueprints = [x for x in blueprints if not x.id.endswith('isetta')]
            blueprints = [x for x in blueprints if not x.id.endswith('carlacola')]
            blueprints = [x for x in blueprints if not x.id.endswith('cybertruck')]
            blueprints = [x for x in blueprints if not x.id.endswith('t2')]

        spawn_points = world.get_map().get_spawn_points()
        number_of_spawn_points = len(spawn_points)

        if args.number_of_vehicles < number_of_spawn_points:
            random.shuffle(spawn_points)
        elif args.number_of_vehicles > number_of_spawn_points:
            msg = 'requested %d vehicles, but could only find %d spawn points'
            logging.warning(msg, args.number_of_vehicles, number_of_spawn_points)
            args.number_of_vehicles = number_of_spawn_points

        # @todo cannot import these directly.
        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        FutureActor = carla.command.FutureActor


        # --------------
        # Spawn vehicles
        # --------------
        batch = []
        blueprint_audi = None          ## for audi tt
        actorblueprint_audi = None
        actor_audi = None
        vehicle_actors = []
        rest_vehicleactors = []        ## to store vehicle actors
        num = 0
        i = 0

        ## velocity of the vehicles
        actor_velocity = -(args.velocity - 70)

        ## load the coordination
        if args.coordination_read:
            coordination = pd.read_csv(coord_file)
        elif not args.coordination_read:
            waypoints = map.generate_waypoints(distance=10.0)  ## the waypoints


        # if args.assigned:   ### assign special car, here is audi
        if blueprint_audi is None:
            blueprint_audi = world.get_blueprint_library().filter('vehicle.audi.tt')   ## actorblueprint, transform, rotation
            actorblueprint_audi = blueprint_audi[0]

        for n, transform in enumerate(spawn_points):
            if n >= args.number_of_vehicles - 1:   ### number of vehicles must be greater than 2
                break
            blueprint = random.choice(blueprints)
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            if blueprint.has_attribute('driver_id'):
                driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                blueprint.set_attribute('driver_id', driver_id)
            if n == 0 and blueprint_audi is not None:  ## the attribute setting for audi
                # actorblueprint_audi.set_attribute('role_name', 'autopilot')  # set the autopilot
                actorblueprint_audi.set_attribute('role_name', 'hero')  # set to the ego vehicles
                actorblueprint_audi.set_attribute('color', '255,0,0')  # set the color of the audi
                ## set the starting point on the highway
                if args.coordination_read:
                    transform.location.x = coordination['lane3 X'][151]
                    transform.location.y = coordination['lane3 Y'][151]
                    transform.location.z = coordination['lane3 Z'][151] + 2
                    transform.rotation.yaw = coordination['lane3 yaw'][151]
                    transform.rotation.pitch = coordination['lane3 pitch'][151]
                    transform.rotation.roll = coordination['lane3 roll'][151]
                    print(transform)
                elif not args.coordination_read:
                    print('in')
                    print('in')
                    print('in')
                    transform.location.x = waypoints[n].transform.location.x
                    transform.location.y = waypoints[n].transform.location.y
                    transform.location.z = waypoints[n].transform.location.z + 2
                    transform.rotation.yaw = waypoints[n].transform.rotation.yaw
                    transform.rotation.pitch = waypoints[n].transform.rotation.pitch
                    transform.rotation.roll = waypoints[n].transform.rotation.roll
                # actor_audi = SpawnActor(actorblueprint_audi, transform).then(SetAutopilot(FutureActor, True))
                actor_audi = world.spawn_actor(actorblueprint_audi, transform)
                audi_id = actor_audi.id
                print(actor_audi)
                vehicle_actors.append(actor_audi)
                traffic_manager.distance_to_leading_vehicle(actor_audi, 3.0) ## set the minimum distance in meters to keep with the others
                traffic_manager.vehicle_percentage_speed_difference(actor_audi, actor_velocity)
                actor_audi.set_autopilot()
                if args.autopilot:  ## autopilot for the ego-vehicle
                    traffic_manager.auto_lane_change(actor_audi, True)  ### change the lane
                elif not args.autopilot:
                    print('not auto')
                    traffic_manager.auto_lane_change(actor_audi, False) ### not to auto change the lane
                traffic_manager.ignore_vehicles_percentage(actor_audi, 100)
                traffic_manager.ignore_lights_percentage(actor_audi, 100)

                ## velocity set control (just for the start)
                actor_audi.apply_control(carla.VehicleControl(throttle=0, brake=0, manual_gear_shift=True,
                                                              gear=0))
                ## collision sensor    carla.CollisionEvent()
                collision_sensor_bp = blueprints_sensors.find('sensor.other.collision')
                collision_sensor = world.spawn_actor(collision_sensor_bp, transform, attach_to=actor_audi)
                collision_sensor.listen(lambda event: collision_detailed(event))

                def collision_detailed(event):
                    if os.path.isfile(path_collision_dataset):
                        collision_newfile = False
                    else:
                        collision_newfile = True
                    actor_ego_collide_against = event.other_actor
                    collision_transform = event.transform
                    collision_frame = event.frame
                    print('collision frame', collision_frame)
                    print('normal frame', frame)
                    print(actor_ego_collide_against)
                    print('type_id', actor_ego_collide_against.type_id)
                    collision_data = pd.DataFrame([[collision_frame, actor_ego_collide_against.id, actor_ego_collide_against.type_id,
                                                    collision_transform.location.x, collision_transform.location.y, collision_transform.location.z]],
                                                  columns=["frame", "actor id", "actor type", "location x",
                                                           "location y", "location z"])
                    if collision_newfile:
                        collision_data.to_csv(path_collision_dataset, index=False)
                    else:
                        collision_data.to_csv(path_collision_dataset, header=None, mode="a", index=False)

                # actor_audi.set_velocity(carla.Vector3D(0, actor_velocity,0))

            else:
                blueprint.set_attribute('role_name', 'autopilot')  ## set to autopilot
                ## get actor blueprint
                if args.coordination_read:        ## for the tranfsorm and location
                    if 71 + num != 151:
                        transform.location.x = coordination["lane{} X".format(i + 1)][51 + num]
                        transform.location.y = coordination["lane{} Y".format(i + 1)][51 + num]
                        transform.location.z = coordination["lane{} Z".format(i + 1)][51 + num] + 2
                        transform.rotation.yaw = coordination["lane{} yaw".format(i + 1)][51 + num]
                        transform.rotation.pitch = coordination["lane{} pitch".format(i + 1)][51 + num]
                        transform.rotation.roll = coordination["lane{} roll".format(i + 1)][51 + num]
                        print(i, transform)
                        if i >= 3:
                            i = 0
                        else:
                            i += 1
                    num += 20
                elif not args.coordination_read: ## use waypoints
                    transform.location.x = waypoints[n].transform.location.x
                    transform.location.y = waypoints[n].transform.location.y
                    transform.location.z = waypoints[n].transform.location.z + 2
                    transform.rotation.yaw = waypoints[n].transform.rotation.yaw
                    transform.rotation.pitch = waypoints[n].transform.rotation.pitch
                    transform.rotation.roll = waypoints[n].transform.rotation.roll

                actor = world.spawn_actor(blueprint, transform)
                rest_vehicleactors.append(actor.id)
                traffic_manager.distance_to_leading_vehicle(actor, 3.0)
                traffic_manager.vehicle_percentage_speed_difference(actor, actor_velocity)
                traffic_manager.auto_lane_change(actor, True)
                actor.set_autopilot()
                traffic_manager.ignore_lights_percentage(actor, 100)
                traffic_manager.ignore_vehicles_percentage(actor, 30)

                ## velocity set control (just for the start)
                actor.apply_control(carla.VehicleControl(throttle=0, brake=0, manual_gear_shift=True,
                                                         gear=0))
                # actor.set_velocity(carla.Vector3D(0, actor_velocity, 0))
                batch.append(SpawnActor(blueprint, transform).then(SetAutopilot(FutureActor, True)))

        # print(dir(traffic_manager))
        for response in client.apply_batch_sync(batch, synchronous_master):
            if response.error:
                logging.error(response.error)
            else:
                vehicles_list.append(response.actor_id)

        # -------------
        # Spawn Walkers
        # -------------
        # some settings
        percentagePedestriansRunning = 0.0  # how many pedestrians will run
        percentagePedestriansCrossing = 0.0  # how many pedestrians will walk through the road
        # 1. take all the random locations to spawn
        spawn_points = []
        for i in range(args.number_of_walkers):
            spawn_point = carla.Transform()
            loc = world.get_random_location_from_navigation()
            if (loc != None):
                spawn_point.location = loc
                spawn_points.append(spawn_point)
        # 2. we spawn the walker object
        batch = []
        walker_speed = []
        for spawn_point in spawn_points:
            walker_bp = random.choice(blueprintsWalkers)
            # set as not invincible
            if walker_bp.has_attribute('is_invincible'):
                walker_bp.set_attribute('is_invincible', 'false')
            # set the max speed
            if walker_bp.has_attribute('speed'):
                if (random.random() > percentagePedestriansRunning):
                    # walking
                    walker_speed.append(walker_bp.get_attribute('speed').recommended_values[1])
                else:
                    # running
                    walker_speed.append(walker_bp.get_attribute('speed').recommended_values[2])
            else:
                print("Walker has no speed")
                walker_speed.append(0.0)
            batch.append(SpawnActor(walker_bp, spawn_point))
        results = client.apply_batch_sync(batch, True)
        walker_speed2 = []
        for i in range(len(results)):
            if results[i].error:
                logging.error(results[i].error)
            else:
                walkers_list.append({"id": results[i].actor_id})
                walker_speed2.append(walker_speed[i])
        walker_speed = walker_speed2
        # 3. we spawn the walker controller
        batch = []
        walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
        for i in range(len(walkers_list)):
            batch.append(SpawnActor(walker_controller_bp, carla.Transform(), walkers_list[i]["id"]))
        results = client.apply_batch_sync(batch, True)
        for i in range(len(results)):
            if results[i].error:
                logging.error(results[i].error)
            else:
                walkers_list[i]["con"] = results[i].actor_id
        # 4. we put altogether the walkers and controllers id to get the objects from their id
        for i in range(len(walkers_list)):
            all_id.append(walkers_list[i]["con"])
            all_id.append(walkers_list[i]["id"])
        all_actors = world.get_actors(all_id)

        # wait for a tick to ensure client receives the last transform of the walkers we have just created
        if not args.sync or not synchronous_master:
            world.wait_for_tick()
        else:
            world.tick()

        # 5. initialize each controller and set target to walk to (list is [controler, actor, controller, actor ...])
        # set how many pedestrians can cross the road
        world.set_pedestrians_cross_factor(percentagePedestriansCrossing)
        for i in range(0, len(all_id), 2):
            # start walker
            all_actors[i].start()
            # set walk to random point
            all_actors[i].go_to_location(world.get_random_location_from_navigation())
            # max speed
            all_actors[i].set_max_speed(float(walker_speed[int(i / 2)]))

        print('spawned %d vehicles and %d walkers, press Ctrl+C to exit.' % (len(vehicles_list), len(walkers_list)))

        ## for dataframe save as csv
        ego_columns = ["Frame", "Ego Vehicle", "Ego ID", "Location X", "Location Y", "Velocity X", "Velocity Y", "Rotation Yaw",
                       "Rotation Pitch", "Rotation Roll", "Ego Lane ID"]
        for i in range(15):
            vehicle_column = ["Vehicle {}".format(1 + i), "Vehicle {} ID".format(1 + i), "Location X {}".format(1 + i), "Location Y {}".format(1 + i), "Velocity X {}".format(1 + i),
                              "Velocity Y {}".format(1 + i), "Rotation Yaw {}".format(1 + i), "Rotation Pitch {}".format(1 + i), "Rotation Roll {}".format(1 + i),
                              "Vehicle {} Lane ID".format(1 + i)]
            ego_columns += vehicle_column
        counter = 0
        frame = 0
        if args.hybrid:         ## tm set the hybrid mode
            traffic_manager.set_hybrid_physics_mode(True)
            traffic_manager.set_hybrid_physics_radius(30.0)
        while True:
            if args.sync and synchronous_master:
                counter += 1
                frame += 1
                # actor_audi.set_velocity(carla.Vector3D(np.sqrt(actor_velocity), np.sqrt(actor_velocity), 0.0))
                print('id and velocity x, y', actor_audi.id, actor_audi.get_velocity().x, actor_audi.get_velocity().y)
                ## draw the location of the ego vehicle
                world.debug.draw_point(actor_audi.get_location(), color=carla.Color(r=0, g=0, b=255),
                                       life_time=1)
                # for vehicle_id in rest_vehicleactors:
                #     vehicle = world.get_actor(vehicle_id)
                #     vehicle.set_velocity(carla.Vector3D(np.sqrt(actor_velocity), np.sqrt(actor_velocity), 0.0))
                #     print(vehicle.id, vehicle.get_velocity())
                # if counter % 70 == 0: ## exit waypoint
                    # get_exit_waypoint(map, actor_audi, waypoints, forward=1500)

                # if counter % 200 == 5 or counter % 520 == 5:
                if counter % 100 == 5 and not args.autopilot:
                    ## not crash on the cars infront of it
                    traffic_manager.ignore_vehicles_percentage(actor_audi, 0)
                    traffic_manager.distance_to_leading_vehicle(actor_audi, 2)
                if counter % 20 == 0:
                    ## The distance to audi ego vehicle
                    vehicles_data = []
                    distances = []
                    for vehicle_id in rest_vehicleactors:
                        vehicle = world.get_actor(vehicle_id)
                        distance = np.sum(np.square([vehicle.get_transform().location.x - actor_audi.get_transform().location.x,
                                              vehicle.get_transform().location.y - actor_audi.get_transform().location.y,
                                              vehicle.get_transform().location.z - actor_audi.get_transform().location.z]))

                        if distance <= 50 ** 2:  ## in 50 meters
                            vehicle_data = [vehicle.type_id, vehicle_id, vehicle.get_transform().location.x, vehicle.get_transform().location.y,
                                            vehicle.get_velocity().x, vehicle.get_velocity().y,
                                            vehicle.get_transform().rotation.yaw, vehicle.get_transform().rotation.pitch, vehicle.get_transform().rotation.roll,
                                            ## lane_id
                                            map.get_waypoint(carla.Location(x=vehicle.get_transform().location.x, y=vehicle.get_transform().location.y,
                                                                            z=vehicle.get_transform().location.z)).lane_id
                                            ]
                            distances.append(distance)
                            vehicles_data.append(vehicle_data)

                            ## circle the cars inside the circle from 40 radius
                            world.debug.draw_string(vehicle.get_transform().location, 'O', color=carla.Color(r=255, g=0, b=0),
                                                   life_time=1)
                    # save the dataframe as a list
                    data_list = [frame, actor_audi.type_id, audi_id, actor_audi.get_transform().location.x, actor_audi.get_transform().location.y,
                                 actor_audi.get_velocity().x, actor_audi.get_velocity().y,
                                 actor_audi.get_transform().rotation.yaw, actor_audi.get_transform().rotation.pitch, actor_audi.get_transform().rotation.roll,
                                 ## lane_id
                                 map.get_waypoint(carla.Location(x=actor_audi.get_transform().location.x,
                                                                 y=actor_audi.get_transform().location.y,
                                                                 z=actor_audi.get_transform().location.z)).lane_id
                                 ]
                    sort_idx = sorted(range(len(distances)), key=lambda k: distances[k])
                    if len(distances) >= 15:
                        for i in range(15):
                            data_list += vehicles_data[sort_idx[i]]
                    else:  ## < 15
                        for i in range(len(distances)):
                            data_list += vehicles_data[sort_idx[i]]
                        none_vehicle = ['None', 'None', 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 'None']
                        data_list += (15 - len(distances)) * none_vehicle

                    ego_neighbor_dt = pd.DataFrame([data_list], columns=ego_columns)
                    if newfile:
                        ego_neighbor_dt.to_csv(path_dataset, index=False)
                        newfile = False
                    else:
                        ego_neighbor_dt.to_csv(path_dataset, header=None, mode="a", index=False)

                if counter % 100 == 0 and not args.autopilot:
                    print('random change lane 20%')
                    traffic_manager.ignore_vehicles_percentage(actor_audi, 100) ## crash the car to the side
                    traffic_manager.distance_to_leading_vehicle(actor_audi, 0)
                    ## 20% for changing lane
                    r = np.random.randint(0, 10, size=1)
                    if r <= 1: ## ego to change lane
                        lr = np.random.uniform(0.0, 1.0, size=1)
                        if lr < 0.5:
                            traffic_manager.force_lane_change(actor_audi, False) ##turn left
                            print('turn left')
                        else:
                            traffic_manager.force_lane_change(actor_audi, True)  ##turn right
                            print('turn right')
                    else: ## ego not to change lane
                        traffic_manager.auto_lane_change(actor_audi, False)
                # elif counter % 502 == 0:
                #     traffic_manager.ignore_vehicles_percentage(actor_audi, 100) ## crash the car to the side
                #     traffic_manager.distance_to_leading_vehicle(actor_audi, 0)
                #     traffic_manager.force_lane_change(actor_audi, True)         ## turn right
                #     print('turn right')

                world.tick()        ## synchronous mode

            else:
                world.wait_for_tick()   ## asynchronous mode
                counter += 1
                if counter % 100 == 0:
                    print(counter)
                if counter % 2000 == 0:
                    traffic_manager.ignore_vehicles_percentage(actor_audi, 100) ##
                    traffic_manager.force_lane_change(actor_audi, False)
                    print('turn left')
                elif counter % 8100 == 0:
                    traffic_manager.ignore_vehicles_percentage(actor_audi, 100) ##
                    traffic_manager.force_lane_change(actor_audi, True)
                    print('turn right')
    finally:

        if args.sync and synchronous_master:
            settings = world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            world.apply_settings(settings)

        print('\ndestroying %d vehicles' % len(vehicles_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])

        # stop walker controllers (list is [controller, actor, controller, actor ...])
        for i in range(0, len(all_id), 2):
            all_actors[i].stop()

        print('\ndestroying %d walkers' % len(walkers_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in all_id])

        time.sleep(0.5)


if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')