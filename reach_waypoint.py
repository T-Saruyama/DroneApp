#!/usr/bin/env python
# -*- coding: utf-8 -*-

# フライトプラン（mission.waypoints）を読み込んで実行し、各WayPointに到着したときにその位置をPrintする
#     ・ただし、実行するフライトプランに含まれるコマンドはWAYPOINTのみ

from dronekit import connect, Command, VehicleMode, LocationGlobalRelative
import time
import math

vehicle = connect('127.0.0.1:14550', wait_ready=True, timeout=60)

wpNav_Radius = vehicle.parameters['WPNAV_RADIUS'] / 100.0   # WPNAV_RADIUSはcm単位なのでm単位に変換
print("['WPNAV_RADIUS']: %s" % vehicle.parameters['WPNAV_RADIUS'])

# ミッション（フライト・プラン）のファイル読み込み
def read_mission_file(aFileName):
    print("Reading mission from file: %s" % aFileName)
    cmds = vehicle.commands
    missionlist=[]
    with open(aFileName) as f:
        for i, line in enumerate(f):
            if i == 0:
                if not line.startswith('QGC WPL 110'):
                    raise Exception('File is not supported WP version')
            else:
                linearray       = line.split('\t')
                ln_index        = int(linearray[0])
                ln_currentwp    = int(linearray[1])
                ln_frame        = int(linearray[2])
                ln_command      = int(linearray[3])
                ln_param1       = float(linearray[4])
                ln_param2       = float(linearray[5])
                ln_param3       = float(linearray[6])
                ln_param4       = float(linearray[7])
                ln_param5       = float(linearray[8])
                ln_param6       = float(linearray[9])
                ln_param7       = float(linearray[10])
                ln_autocontinue = int(linearray[11].strip())
                cmd = Command( 0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
                missionlist.append(cmd)
    return missionlist


# ミッションのアップロード
def upload_mission(aFileName):
    # Read mission from file
    missionlist = read_mission_file(aFileName)
    
    print("Upload mission from a file: %s" % aFileName)
    # Clear existing mission from vehicle
    print(' Clear mission')
    cmds = vehicle.commands
    cmds.clear()
    # Add new mission to vehicle
    for command in missionlist:
        cmds.add(command)
    print(' Upload mission')
    vehicle.commands.upload()


# 2地点間のXY方向の距離（メートル）
def get_distance_metres_XY(aLocation1, aLocation2):
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


# WayPointまでのXY方向の距離（メートル）
def distance_to_current_waypoint_XY(wayPoint, myLocation):
    missionitem = vehicle.commands[wayPoint-1]  # commands are zero indexed
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    targetWaypointLocation = LocationGlobalRelative(lat, lon, alt)
    distancetopoint_XY = get_distance_metres_XY(targetWaypointLocation, myLocation)
    return distancetopoint_XY


# WayPointまでのZ方向の距離（メートル）
def distance_to_current_waypoint_Z(wayPoint, myLocation):
    missionitem = vehicle.commands[wayPoint-1]  # commands are zero indexed
    return abs(missionitem.z - myLocation.alt)


# WayPointの到達チェック（コールバック）
def reach_check_callback(self, attr_name, value):
    nextWayPoint = vehicle.commands.next
    myLocation = vehicle.location.global_relative_frame
    if (reach_check_callback.lastNext != nextWayPoint) and (reach_check_callback.lastNext != reach_check_callback.rchWayPoint):
        # 到達チェック(isReach)がTrueにならずにWayPointを通過してしまったとき（コールバックの呼び出しタイミングに依存する）
        if (reach_check_callback.lastNext > 0) and (not reach_check_callback.isReach):
            lstNext = reach_check_callback.lastNext
            distance_XY = distance_to_current_waypoint_XY(lstNext, myLocation)     # XY方向の距離
            distance_Z = distance_to_current_waypoint_Z(lstNext, myLocation)       # Z方向の距離
            missionitem = vehicle.commands[lstNext-1]  # commands are zero indexed
            print("* Reached No.",  lstNext,
                  ": Lat=",         missionitem.x,
                  ", Long=",        missionitem.y,
                  ", Alt=",         missionitem.z,
                  ", Distance:XY=", distance_XY,
                  ",Z=",            distance_Z)
            reach_check_callback.isReach = True
            reach_check_callback.rchWayPoint = lstNext
    else:
        # WayPointに到着するまでの距離をチェック
        distance_XY = distance_to_current_waypoint_XY(nextWayPoint, myLocation)     # XY方向の距離
        distance_Z = distance_to_current_waypoint_Z(nextWayPoint, myLocation)       # Z方向の距離
        if (distance_XY <= wpNav_Radius) and (distance_Z <= wpNav_Radius):
            if not reach_check_callback.isReach:
                missionitem = vehicle.commands[nextWayPoint-1]  # commands are zero indexed
                print("  Reached No.",  nextWayPoint,
                      ": Lat=",         missionitem.x,
                      ", Long=",        missionitem.y,
                      ", Alt=",         missionitem.z,
                      ", Distance:XY=", distance_XY,
                      ",Z=",            distance_Z)
                reach_check_callback.isReach = True
                reach_check_callback.rchWayPoint = nextWayPoint
        else:
            reach_check_callback.isReach = False
    reach_check_callback.lastNext = nextWayPoint


reach_check_callback.isReach = False    # 各WayPointに到達したときに1度だけTrueになるフラグ
reach_check_callback.rchWayPoint = 0    # 最後に到達したWayPoint
reach_check_callback.lastNext = -1      # 前回のコールバック時のvehicle.commands.next

while not vehicle.is_armable:
    print("..Waiting for vehicle to initialise...")
    time.sleep(1)

vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True
while not vehicle.armed:
    print("..Waiting for arming...")
    time.sleep(1)

print("Take off!")
targetAltitude = 10
vehicle.simple_takeoff(targetAltitude)
while True:
    print("  Altitude: ", vehicle.location.global_relative_frame.alt)
    if vehicle.location.global_relative_frame.alt >= targetAltitude * 0.95:
        print("..Reached target altitude")
        break
    time.sleep(1)

# Upload mission from file
mission_filename = 'mission.waypoints'
upload_mission(mission_filename)

# Download mission
cmds = vehicle.commands
cmds.download()
cmds.wait_ready()  # wait until download is complete.

if len(vehicle.commands) == 0:
    print("..Mission is empty..")
else:
    vehicle.mode = VehicleMode("AUTO")
    print("..Mission Start..")
    time.sleep(1)

    vehicle.add_attribute_listener('location.global_relative_frame', reach_check_callback)

    while True:
        if reach_check_callback.rchWayPoint == len(vehicle.commands):
            break
        if vehicle.mode != VehicleMode("AUTO"):
            break

    vehicle.remove_attribute_listener('location.global_relative_frame', reach_check_callback)
    print("..Mission End..")

print("Return to launch")
vehicle.mode = VehicleMode("RTL")
time.sleep(5)

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()
