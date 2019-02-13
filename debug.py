#!/usr/bin/env python

import carla
import time
import pprint

ipAddress = "172.20.1.75"
killCount = 0

def main():
    print("# This script is for debug only")

    print("Connecting to", ipAddress)
    client = carla.Client(ipAddress, 2000)
    client.set_timeout(2.0)

    print("Client version", client.get_client_version())
    print("Server version", client.get_server_version())

    print("Getting world")
    world = client.get_world()
    print(world)

    time.sleep(.1)

    print("Getting actors")
    actors = world.get_actors()
    print("Found", len(actors), "actors")

    while(True):
        print("--- --- ---")
        print("0 ... exit")
        print("1 ... reload actors")
        print("2 ... display actors")
        print("3 ... actor kill menu")
        print("4 ... actor pose menu")

        selection = input("Selection: ")
        print("--- --- ---")

        try:
            s = int(selection)
        except:
            s = -1

        if s == 0:
            exit()
        elif s == 1:
            print("Reloading actors")
            actors = world.get_actors()
            print("Found", len(actors), "actors")
        elif s == 2:
            print("Displaying actors")
            for actor in actors:
                print(actor)
        elif s == 3:
            killMenu(actors)
        elif s == 4:
            poseMenu(actors)
        else:
            print("")
            continue

def killMenu(actors):
    actorID = input("Provide ActorID to kill (0 to abort): ")
    try:
        actorID = int(actorID)
    except:
        actorID = 0
    
    if(actorID != 0):
        try:
            for actor in actors:
                if(actor.id == actorID):
                    actor.destroy()
        except Exception as e:
            print("Killing gone wrong:", e)

        global killCount
        killCount += 1
        if killCount == 5:
            print("killing spree")
    else:
        print("Killing aborted ... for now")

def poseMenu(actors):
    while(True):
        print("--- Pose Menu ---")
        print("0 ... exit to main menu")
        print("1 ... get pose from actor")
        print("2 ... set pose for actor (free)")
        print("3 ... set pose for actor (defaults)")

        selection = input("Selection: ")
        print("--- --- ---")

        try:
            s = int(selection)
        except:
            s = -1

        if s == 0:
            return
        elif s == 1:
            actorID = input("Provide ActorID to print pose (0 to abort): ")
            try:
                actorID = int(actorID)
            except:
                actorID = 0
            
            if(actorID != 0):
                try:
                    for actor in actors:
                        if(actor.id == actorID):
                            print(actor.get_transform())
                except Exception as e:
                    print("Error during pose retrieval:", e)
            else:
                print("Aborted pose retrieval")
        elif s == 2:
            print("Provide actor ID and pose, then confirm settings")
            actorID = input("Provide ActorID: ")
            x = input("x: ")
            y = input("y: ")
            z = input("z: ")
            roll = input("roll(°): ")
            pitch = input("pitch(°): ")
            yaw = input("yaw(°): ")

            try:
                actorID = int(actorID)
                x = int(x)
                y = int(y)
                z = int(z)
                roll = int(roll)
                pitch = int(pitch)
                yaw = int(yaw)
            except Exception as e:
                print("Input error:", e)
                continue

            try:
                for actor in actors:
                    if(actor.id == actorID):
                        transform = actor.get_transform()
                        print("old pose:", transform)
                        transform.location.x = x
                        transform.location.y = y
                        transform.location.z = z
                        transform.rotation.roll = roll
                        transform.rotation.pitch = pitch
                        transform.rotation.yaw = yaw
                        print("new pose:", transform)
                        
                        decision = input("confirm pose change (y/n)")
                        if(decision == "y"):
                            actor.set_transform(transform)
                    else:
                        print("actor ID", actorID, "not found")
                        continue
            except Exception as e:
                print("Error during pose retrieval/setting:", e)
        elif s == 3:
            actorID = input("Provide ActorID: ")
            actorForPose = None
            try:
                actorID = int(actorID)
                for actor in actors:
                    if(actor.id == actorID):
                        actorForPose = actor
                if(actorForPose == None):
                    print("Couldn't find actor")
                    continue
            except Exception as e:
                print("Input error:", e)
                continue

            transform1 = carla.Transform()
            transform1.location.x = 80.0
            transform1.location.y = 0.0
            transform1.location.z = 60.0
            transform1.rotation.roll = -88.0
            transform1.rotation.pitch = -88.0
            transform1.rotation.yaw = 0.0
            
            print("Select Transform:")
            print("0 ... abort")
            print("1 ...", transform1)
            

            option = input("Select Pose:")

            try:
                o = int(option)
            except:
                o = -1

            if o == 1:
                try:
                    actor.set_transform(transform1)
                except Exception as e:
                    print("Error during pose setting:", e)
            else:
                print("Aborted")
            
        else:
            print("")
            continue


if __name__ == '__main__':

    main()
