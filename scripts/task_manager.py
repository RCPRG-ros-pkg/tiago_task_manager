#!/usr/bin/env python

# Copyright (c) 2019, Robot Control and Pattern Recognition Group,
# Institute of Control and Computation Engineering
# Warsaw University of Technology
#
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Warsaw University of Technology nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Author: Dawid Seredynski
#

import rospy
import tf

import sys
import rospy
import math
import copy

import tf
import tf2_ros
from tf.transformations import quaternion_from_euler
import actionlib
from geometry_msgs.msg import Pose

from tiago_msgs.msg import Command

from rosplan_tiago_scenarios_msgs.msg import GoAction, GoActionGoal, GoActionFeedback, GoActionResult, GoGoal

from tiago_behaviours_msgs.msg import WanderAction, WanderGoal

def deg2rad(angle_deg):
    return float(angle_deg)/180.0*math.pi

def makePose(x, y, theta):
    q = quaternion_from_euler(0, 0, theta)
    result = Pose()
    result.position.x = x
    result.position.y = y
    result.orientation.x = q[0]
    result.orientation.y = q[1]
    result.orientation.z = q[2]
    result.orientation.w = q[3]
    return result

def taskGoTo(place_name):
    print 'Poproszono mnie o przejscie do: ' + place_name
    # TODO: transform place_name to pose
    if place_name == 'kuchnia':
        pose = makePose(3, 0.2, -math.pi/2)
    elif place_name in ['warsztat', 'warsztatu']:
        pose = makePose(1.55, 8.65, math.pi/2)
    elif place_name in ['pokoj', 'pok\\303\\263j']:
        pose = makePose(-0.15, -0.3, math.pi/2)
    elif place_name == 'sypialnia':
        pose = makePose(3, 5, math.pi/2)
    else:
        print 'Nie wiem gdzie jest: ' + place_name
        return False

    # Creates the SimpleActionClient, passing the type of the action to the constructor.
    client = actionlib.SimpleActionClient('go', GoAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = GoGoal()

    goal.pose = pose

    #goal.header.stamp = rospy.get_rostime()
    #goal.header.frame_id = 'map'

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()

def taskWander():
    print 'Poproszono mnie, abym zaczal patrolowac'
    # Creates the SimpleActionClient, passing the type of the action to the constructor.
    client = actionlib.SimpleActionClient('wander', WanderAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = WanderGoal()

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()

def taskBring(object_name):
    pass

def callback(data):
    #print 'query_text', data.query_text
    #print 'intent_name', data.intent_name

    param_dict = {}
    for param_name, param_value in zip(data.param_names, data.param_values):
        param_dict[param_name] = param_value

    if data.intent_name == 'projects/incare-dialog-agent/agent/intents/176ab2ca-6250-4227-985b-cc82d5497d9f':
        object_name = param_dict['przedmiot']
        print 'poproszono mnie o przyniesienie: ' + object_name

    elif data.intent_name == 'projects/incare-dialog-agent/agent/intents/0165eceb-9621-4a7d-aecc-7a879951da18':
        taskGoTo( param_dict['miejsce'] )

    elif data.intent_name == 'projects/incare-dialog-agent/agent/intents/2f028022-05b6-467d-bcbe-e861ab449c17':
        print 'Niezrozumiale polecenie: "' + data.query_text + '"'

    elif data.intent_name == 'projects/incare-dialog-agent/agent/intents/d9e96166-030b-442f-a513-d3fa2e044030':
        taskWander()
    else:
        raise Exception('Unknown intent: "' + data.intent_name + '", query_text: "' + data.query_text + '"')

    #data.parameters
    #data.confidence
    #data.response_text
    
if __name__ == "__main__":

    rospy.init_node('tiago_task_manager', anonymous=False)

    rospy.Subscriber("tiago_cmd", Command, callback)

    rospy.spin()
