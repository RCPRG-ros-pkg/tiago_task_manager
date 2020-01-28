#!/usr/bin/env python
# encoding: utf8

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

from tiago_behaviours_msgs.msg import WanderAction, WanderGoal
from tiago_behaviours_msgs.msg import MoveToAction, MoveToGoal
from tiago_behaviours_msgs.msg import BringGoodsAction, BringGoodsGoal
from tiago_behaviours_msgs.msg import StopAction, StopGoal
from tiago_behaviours_msgs.msg import QuestionLoadAction, QuestionLoadGoal
from tiago_behaviours_msgs.msg import QuestionCurrentTaskAction, QuestionCurrentTaskGoal
from tiago_behaviours_msgs.msg import AckAction, AckGoal
from tiago_behaviours_msgs.msg import AckIgaveAction, AckIgaveGoal
from tiago_behaviours_msgs.msg import AckItookAction, AckItookGoal

import pl_nouns.odmiana as ro

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

class TaskManager:
    def __init__(self):
        self.o = ro.OdmianaRzeczownikow()
        rospy.Subscriber("tiago_cmd", Command, self.callback)

        # Creates the SimpleActionClient, passing the type of the action to the constructor.
        self.clients = {}
        self.clients['move_to'] = actionlib.SimpleActionClient('move_to', MoveToAction)
        self.clients['wander'] = actionlib.SimpleActionClient('wander', WanderAction)
        self.clients['bring_goods'] = actionlib.SimpleActionClient('bring_goods', BringGoodsAction)
        self.clients['stop'] = actionlib.SimpleActionClient('stop', StopAction)
        self.clients['q_load'] = actionlib.SimpleActionClient('q_load', QuestionLoadAction)
        self.clients['q_current_task'] = actionlib.SimpleActionClient('q_current_task', QuestionCurrentTaskAction)
        self.clients['ack'] = actionlib.SimpleActionClient('ack', AckAction)
        self.clients['ack_i_gave'] = actionlib.SimpleActionClient('ack_i_gave', AckIgaveAction)
        self.clients['ack_i_took'] = actionlib.SimpleActionClient('ack_i_took', AckItookAction)

        # Waits until the action server has started up and started
        # listening for goals.
        for client_name, client in self.clients.iteritems():
            print 'Waiting for action server "' + client_name + '"'
            client.wait_for_server()
        print 'Done.'

    def przypadki(self, word):
        blocks = self.o.getBlocks(word)
        if len(blocks) == 0:
            print u'Nie moge znaleźć nazwy miejsca w słowniku'
            word_m = word
        else:
            m_lp = self.o.getMianownikLp(blocks)
            if len(m_lp) == 0:
                m_lm = self.o.getMianownikLp(blocks)
                word_m = m_lm[0]
            else:
                word_m = m_lp[0]

        word_d = self.o.getDopelniaczLp(blocks, mianownik=word_m)
        if len(word_d) == 0:
            word_d = self.o.getDopelniaczLm(blocks, mianownik=word_m)
        if len(word_d) == 0:
            word_d = [word_m]

        word_b = self.o.getBiernikLp(blocks, mianownik=word_m)
        if len(word_b) == 0:
            word_b = self.o.getBiernikLm(blocks, mianownik=word_m)

        if len(word_b) == 0:
            word_b = [word_m]

        return word_m, word_d[0], word_b[0]

    def taskMoveTo(self, place_name):
        pl_name = place_name.strip().lower()

        if pl_name == '':
            print u'Mam gdzieś iść, ale nie podano miejsca'
            return False

        place_name_m, place_name_d, place_name_b = self.przypadki(pl_name)

        print u'Poproszono mnie o przejście do ' + place_name_d.decode('utf-8') + u' (' + place_name_m.decode('utf-8') + u')'

        client = self.clients['move_to']

        # Creates a goal to send to the action server.
        goal = MoveToGoal()

        goal.pose_valid = False
        goal.place_name = place_name_m
        goal.place_name_valid = True

        # Sends the goal to the action server.
        client.send_goal(goal)

        # Waits for the server to finish performing the action.
        client.wait_for_result()

        # Prints out the result of executing the action
        return client.get_result()

    def taskWander(self):
        print u'Poproszono mnie, abym zaczął patrolowac'
        client = self.clients['wander']

        # Creates a goal to send to the action server.
        goal = WanderGoal()

        # Sends the goal to the action server.
        client.send_goal(goal)

        # Waits for the server to finish performing the action.
        client.wait_for_result()

        # Prints out the result of executing the action
        return client.get_result()

    def taskBring(self, object_name):
        print object_name
        ob_name = object_name.strip().lower()
        ob_name_m, ob_name_d, ob_name_b = self.przypadki(ob_name)
        print u'Poproszono mnie o przyniesienie ' + ob_name_d.decode('utf-8') + u' (' + ob_name_m.decode('utf-8') + u')'
        client = self.clients['bring_goods']
        goal = BringGoodsGoal()
        goal.goods_name = object_name
        client.send_goal(goal)
        client.wait_for_result()
        return client.get_result()

    def taskStop(self):
        print u'Poproszono mnie o zatrzymanie się.'
        client = self.clients['stop']
        goal = StopGoal()
        client.send_goal(goal)
        client.wait_for_result()
        return client.get_result()

    def questionLoad(self):
        print u'Zapytano mnie: co wiozę?'

        client = self.clients['q_load']
        goal = QuestionLoadGoal()
        client.send_goal(goal)
        client.wait_for_result()
        return client.get_result()

    def questionCurrentTask(self):
        print u'Zapytano mnie: co robię?'
        client = self.clients['q_current_task']
        goal = QuestionCurrentTaskGoal()
        client.send_goal(goal)
        client.wait_for_result()
        return client.get_result()

    def respAck(self):
        print u'Usłyszałem ogólne potwierdzenie'
        client = self.clients['ack']
        goal = AckGoal()
        client.send_goal(goal)
        client.wait_for_result()
        return client.get_result()

    def respAckIgave(self):
        print u'Usłyszałem potwierdzenie podania'
        client = self.clients['ack_i_gave']
        goal = AckIgaveGoal()
        client.send_goal(goal)
        client.wait_for_result()
        return client.get_result()

    def respAckItook(self):
        print u'Usłyszałem potwierdzenie odebrania'
        client = self.clients['ack_i_took']
        goal = AckItookGoal()
        client.send_goal(goal)
        client.wait_for_result()
        return client.get_result()

    def callback(self, data):
        #print 'query_text', data.query_text
        #print 'intent_name', data.intent_name

        param_dict = {}
        for param_name, param_value in zip(data.param_names, data.param_values):
            param_dict[param_name] = param_value
            print 'param_name, param_value', param_name, param_value

        if data.intent_name == 'projects/incare-dialog-agent/agent/intents/176ab2ca-6250-4227-985b-cc82d5497d9f':
            self.taskBring(param_dict['przedmiot'])

        elif data.intent_name == 'projects/incare-dialog-agent/agent/intents/0165eceb-9621-4a7d-aecc-7a879951da18':
            self.taskMoveTo( param_dict['miejsce'] )

        elif data.intent_name == 'projects/incare-dialog-agent/agent/intents/7acd4325-4cdd-4e15-99be-ad545f4dddd5':
            self.taskStop()

        elif data.intent_name == 'projects/incare-dialog-agent/agent/intents/2f028022-05b6-467d-bcbe-e861ab449c17':
            print u'Niezrozumiałe polecenie: "' + data.query_text.decode('utf-8') + '"'

        elif data.intent_name == 'projects/incare-dialog-agent/agent/intents/d9e96166-030b-442f-a513-d3fa2e044030':
            self.taskWander()

        elif data.intent_name == 'projects/incare-dialog-agent/agent/intents/6a3d7152-53c5-4757-9eec-8d2e0cf16e69':
            # Do nothing: greeting
            pass

        elif data.intent_name == 'projects/incare-dialog-agent/agent/intents/b8743ab9-08a1-49e8-a534-abb65155c507':
            self.questionLoad()

        elif data.intent_name == 'projects/incare-dialog-agent/agent/intents/8f45359d-ee47-4e10-a1b2-de3f3223e5b4':
            self.questionCurrentTask()

        elif data.intent_name == 'projects/incare-dialog-agent/agent/intents/ef92199b-d298-470c-8df3-1e1047dd70d1':
            # Potwierdzenie
            self.respAck()

        elif data.intent_name == 'projects/incare-dialog-agent/agent/intents/d017cbd0-93f8-45b2-996e-043cdccab629':
            # Potw_podalem
            self.respAckIgave()

        elif data.intent_name == 'projects/incare-dialog-agent/agent/intents/181621b6-e91e-4244-a925-c5dc32ee1f1b':
            # Potw_odebralem
            self.respAckItook()

        else:
            raise Exception('Unknown intent: "' + data.intent_name + '", query_text: "' + data.query_text + '"')

if __name__ == "__main__":

    rospy.init_node('tiago_task_manager', anonymous=False)

    tm = TaskManager()

    rospy.spin()
