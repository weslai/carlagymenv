#!/usr/bin/env python

## Fraunhofer IIs
## Wei-Cheng Lai
## Mostly base on Spawn_npc.py from tutorial

### Carla Traffic Manager
### Vehicles selfcontrol

import glob
import os
import sys
import time
import pandas as pd

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
        help='number of vehicles (default: 10)')
    argparser.add_argument(
        '-w', '--number-of-walkers',
        metavar='W',
        default=50,
        type=int,
        help='number of walkers (default: 50)')
    argparser.add_argument(
        '--assigned',
        default=True,
        type=bool,
        help='one Vehicle assigned, audi.tt'
    )
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
        '--sync',
        action='store_true',
        help='Synchronous mode execution')
    argparser.add_argument(
        '-m', '--map_name',
        default='Town01',
        type=str,
        help='map name to load in the server (default: Town01)')
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
    dataset_name = "audi_tt.csv"
    path_dataset = path_dataset + dataset_name

    if os.path.isfile(path_dataset):
        newfile = False
    else:
        newfile = True

    try:
        traffic_manager = client.get_trafficmanager(args.tm_port)

        ## tm set the global distance to other vehicles
        traffic_manager.set_global_distance_to_leading_vehicle(4.0)
        ## tm set the hybrid mode

        world = client.get_world()

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

        ## get traffic lights and set them to green
        # actor_trafficlights = world.get_actors().filter('traffic.traffic_light*')
        # for light in actor_trafficlights:
        #     light.set_state(carla.TrafficLightState.Green)
        # print(actor_trafficlights[0])

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
        blueprint_toyota = None        ## for toyota prius 3 Cars
        vehicle_actors = []
        if args.assigned:   ### assign special car, here is audi
            if blueprint_audi is None:
                blueprint_audi = world.get_blueprint_library().filter('vehicle.audi.tt')   ## actorblueprint, transform, rotation
                actorblueprint_audi = blueprint_audi[0]
            if blueprint_toyota is None:
                blueprint_toyota = world.get_blueprint_library().filter('vehicle.toyota.prius')   ## actorblueprint, transform, rotation
                blueprint_toyota1 = world.get_blueprint_library().filter('vehicle.toyota.prius')
                blueprint_toyota2 = world.get_blueprint_library().filter('vehicle.toyota.prius')
                bp_toyota_list = [blueprint_toyota, blueprint_toyota1, blueprint_toyota2]
                actor_toyota_list = [blueprint_toyota[0], blueprint_toyota1[0], blueprint_toyota2[0]]

        for n, transform in enumerate(spawn_points):
            if n >= args.number_of_vehicles - 2:
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
                actorblueprint_audi.set_attribute('role_name', 'hero')  # set the autopilot
                actorblueprint_audi.set_attribute('color', '255,0,0')  # set the color of the audi
                ## set the starting point on the highway
                transform.location.x = -16.6
                transform.location.y = -87.56    ## -146.11
                transform.location.z = 0.2819    ## 1.19
                transform.rotation.yaw = 90
                transform.rotation.pitch = 0
                transform.rotation.roll = 0
                print(transform)
                # print(type(blueprint_audi))
                # actor_audi = SpawnActor(actorblueprint_audi, transform).then(SetAutopilot(FutureActor, True))
                actor_audi = world.spawn_actor(actorblueprint_audi, transform)
                print(actor_audi)
                vehicle_actors.append(actor_audi)
                traffic_manager.distance_to_leading_vehicle(actor_audi, 2.0) ## set the minimum distance in meters to keep with the others

                traffic_manager.vehicle_percentage_speed_difference(actor_audi, 30.0)
                actor_audi.set_autopilot()
                traffic_manager.auto_lane_change(actor_audi, False)  ### not change the lane (force to ride on highway)?
                traffic_manager.ignore_vehicles_percentage(actor_audi, 0)
                traffic_manager.ignore_lights_percentage(actor_audi, 100)
            elif n == 1 and bp_toyota_list is not None: ## the attribute setting for toyota
                actor_toyota_list[0].set_attribute('role_name', 'autopilot')  # set the autopilot  ## hero?
                actor_toyota_list[1].set_attribute('role_name', 'autopilot')  # set the autopilot  ## hero?
                actor_toyota_list[2].set_attribute('role_name', 'autopilot')  # set the autopilot  ## hero?
                actor_toyota_list[0].set_attribute('color', '0,255,0')  # set the color of the toyota
                actor_toyota_list[1].set_attribute('color', '0,255,255')  # set the color of the toyota
                actor_toyota_list[2].set_attribute('color', '255,255,255')  # set the color of the toyota
                ## set the starting point on the highway
                transform.location.x = -5.9
                transform.location.y = -39.15  ## -146.11
                transform.rotation.yaw = 90
                transform.rotation.pitch = 0
                transform.rotation.roll = 0
                print(transform)
                actor_toyota = world.spawn_actor(actor_toyota_list[0], transform)
                transform.location.x = -9.25
                print(transform)
                actor_toyota1 = world.spawn_actor(actor_toyota_list[1], transform)
                transform.location.x = -13.03
                print(transform)
                actor_toyota2 = world.spawn_actor(actor_toyota_list[2], transform)
                vehicle_actors.append(actor_toyota)
                vehicle_actors.append(actor_toyota1)
                vehicle_actors.append(actor_toyota2)
                traffic_manager.distance_to_leading_vehicle(actor_toyota, 2.0)  ## set the minimum distance in meters to keep with the others
                traffic_manager.distance_to_leading_vehicle(actor_toyota1, 4.0)  ## set the minimum distance in meters to keep with the others
                traffic_manager.distance_to_leading_vehicle(actor_toyota2, 6.0)  ## set the minimum distance in meters to keep with the others
                traffic_manager.vehicle_percentage_speed_difference(actor_toyota, -10.0)
                traffic_manager.vehicle_percentage_speed_difference(actor_toyota1, -10.0)
                traffic_manager.vehicle_percentage_speed_difference(actor_toyota2, -10.0)
                traffic_manager.auto_lane_change(actor_toyota, False)  ### not change the lane (force to ride on highway)?
                traffic_manager.auto_lane_change(actor_toyota1, False)  ### not change the lane (force to ride on highway)?
                traffic_manager.auto_lane_change(actor_toyota2, False)  ### not change the lane (force to ride on highway)?
                actor_toyota1.set_autopilot()
                actor_toyota.set_autopilot()
                actor_toyota2.set_autopilot()
                traffic_manager.ignore_lights_percentage(actor_toyota, 100)
                traffic_manager.ignore_lights_percentage(actor_toyota1, 100)
                traffic_manager.ignore_lights_percentage(actor_toyota2, 100)
                traffic_manager.ignore_vehicles_percentage(actor_toyota, 70)
                traffic_manager.ignore_vehicles_percentage(actor_toyota1, 50)
                traffic_manager.ignore_vehicles_percentage(actor_toyota2, 30)
            else:
                # mul = 3.5
                blueprint.set_attribute('role_name', 'autopilot')
                # if n < 5:  ## for left lanes
                #     transform.location.x = 4.6115 + (n-1) * mul
                #     transform.location.y = -87.56
                #     transform.location.z = 0.2819
                #     transform.rotation.yaw = -90
                #     transform.rotation.pitch = 0
                #     transform.rotation.roll = 0
                #     print(transform)
                #     actor = world.spawn_actor(blueprint, transform)
                #     actor.set_autopilot()
                #     traffic_manager.auto_lane_change(actor, False)  ### not change the lane (force to ride on highway)?
                #
                # if n >= 5 and n < 8:  ## for right lanes
                #     transform.location.x = -5.90521 - (n - 5) * mul
                #     transform.location.y = -87.56
                #     transform.location.z = 0.2819
                #     transform.rotation.yaw = 90
                #     transform.rotation.pitch = 0
                #     transform.rotation.roll = 0
                #     print(transform)
                #     actor = world.spawn_actor(blueprint, transform)
                #     actor.set_autopilot()
                #     traffic_manager.auto_lane_change(actor, False)  ### not change the lane (force to ride on highway)?
                batch.append(SpawnActor(blueprint, transform).then(SetAutopilot(FutureActor, True)))
                # vehicle_actors.append(world.spawn_actor(blueprint, transform))
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


        counter = 0
        frame = 0
        # traffic_manager.set_hybrid_physics_mode(True)
        # traffic_manager.set_hybrid_physics_radius(30.0)
        while True:
            if args.sync and synchronous_master:
                counter += 1
                if counter % 200 == 5 or counter % 520 == 5:
                    ## not crash on the cars infront of it
                    traffic_manager.ignore_vehicles_percentage(actor_audi, 0)
                    traffic_manager.distance_to_leading_vehicle(actor_audi, 2)
                    # for vehicle in vehicle_actors[1:]:
                    #     traffic_manager.collision_detection(actor_audi, vehicle, True)
                if counter % 60 == 0:
                    # save the dataframe as a list
                    data_list = [frame, actor_audi.get_transform().location.x, actor_audi.get_transform().location.y, actor_audi.get_transform().location.z,
                                 actor_audi.get_transform().rotation.yaw, actor_audi.get_transform().rotation.pitch, actor_audi.get_transform().rotation.roll,
                                 actor_audi.get_velocity().x, actor_audi.get_velocity().y, actor_audi.get_velocity().z,
                                 actor_audi.get_acceleration().x, actor_audi.get_acceleration().y, actor_audi.get_acceleration().z,
                                 actor_audi.get_angular_velocity().x, actor_audi.get_angular_velocity().y, actor_audi.get_angular_velocity().z]
                    data_columns = ["Frame", "Location X", "Location Y", "Location Z", "Rotation Yaw", "Rotation Pitch", "Rotation Roll", "Velocity X", "Velocity Y", "Velocity Z",
                                    "Acceleration X", "Acceleration Y", "Acceleration Z", "Angular Velocity X", "Angular Velocity Y", "Angular Velocity Z"]
                    audi_tt_df = pd.DataFrame([data_list], columns=data_columns)
                    frame += 1
                    if newfile:
                        audi_tt_df.to_csv(path_dataset, index=False)
                        newfile = False
                    else:
                        audi_tt_df.to_csv(path_dataset, header=None, mode="a", index=False)

                if counter % 200 == 0:
                    # for vehicle in vehicle_actors[1:]:
                    #     traffic_manager.collision_detection(actor_audi, vehicle, False)
                    traffic_manager.ignore_vehicles_percentage(actor_audi, 100) ## crash the car to the side
                    traffic_manager.distance_to_leading_vehicle(actor_audi, 0)
                    traffic_manager.force_lane_change(actor_audi, False)        ## turn left
                    print('turn left')
                elif counter % 502 == 0:
                    # for vehicle in vehicle_actors[1:]:
                    #     traffic_manager.collision_detection(actor_audi, vehicle, False)
                    traffic_manager.ignore_vehicles_percentage(actor_audi, 100) ## crash the car to the side
                    traffic_manager.distance_to_leading_vehicle(actor_audi, 0)
                    traffic_manager.force_lane_change(actor_audi, True)         ## turn right
                    print('turn right')

                ## turn the light to green
                # for vehicle in vehicle_actors:
                #     if vehicle.is_at_traffic_light():
                #         traffic_light = vehicle.get_traffic_light()
                #         if traffic_light.get_state() == carla.TrafficLightState.Red:
                #             traffic_light.set_state(carla.TrafficLightState.Green)

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