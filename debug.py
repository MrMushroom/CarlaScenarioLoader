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
        print("Aborted pose menu")


if __name__ == '__main__':

    main()
