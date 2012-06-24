#!/usr/bin/env python

# Not all of the modules are needed for this version, but I left it anyway.
import roslib; roslib.load_manifest('openni_tracker')
import rospy

import tf
from bilibot_node.srv import SetArmPosition
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist

import sys, select, termios, tty
import threading
from math import * 

from collections import namedtuple 
Coord = namedtuple('Coord', ['x', 'y', 'z'])

# ArmController is an interface for controlling the arm 
# and keeping track of its location.
class ArmController(threading.Thread):

    def __init__(self, service_name):
        self._goal_pos = 255.
        self._clicks_per_sec = 100./2.2
        self._clicks_per_nsec = self._clicks_per_sec*10.**-9
        rospy.wait_for_service(service_name)
        self._set_arm_pos = rospy.ServiceProxy(service_name, SetArmPosition)
        self._current_pos = 0.
        self._set_arm_pos(self._goal_pos)
        self._last_update = rospy.Time.now()

    def _update_current_pos(self):
        clicks_needed = self._goal_pos - self._current_pos
        clicks_actual = (rospy.Time.now() - self._last_update).nsecs*self._clicks_per_nsec
        print 'clicks', clicks_needed, clicks_actual
        if fabs(clicks_needed) > clicks_actual:
            self._current_pos += copysign(clicks_actual, clicks_needed) 
        else:
            self._current_pos = self._goal_pos
        self._last_update = rospy.Time.now()

    # Returns the current arm position.
    @property
    def pos(self):
        self._update_current_pos()
        return self._current_pos
    # Sets a new arm position.
    def set_pos(self, val):
        if val > 255: val = 255
        if val < 120: val = 120
        if self._goal_pos != val:
            self._update_current_pos()
            self._goal_pos = val
            self._set_arm_pos(val)

    # determines if the arm has reached its destination position yet.
    def is_ready(self):
        self._update_current_pos()
        print 'pos',self._current_pos, self._goal_pos
        return fabs(self._current_pos - self._goal_pos) < 5.

if __name__ == '__main__':
    # Initialization.
    rospy.init_node('tracker') 

    pub = rospy.Publisher('cmd_vel', Twist)

    ac = ArmController('set_arm_pos')
    rospy.wait_for_service('toggle_hand_state')
    move_hand = rospy.ServiceProxy('toggle_hand_state', Empty)

    #listener = tf.TransformListener()
    #absFrame = 'openni_depth_frame'
    userno = 1
    target = None

    while not rospy.is_shutdown():
        # Cycle through users 1-9.
        userno = userno % 9
        userno += 1
        torsoFrame = 'torso_' + str(userno)
        rHandFrame = 'right_hand_' + str(userno)
        lHandFrame = 'left_hand_' + str(userno)
        rElbowFrame = 'right_elbow_' + str(userno)
        lElbowFrame = 'left_elbow_' + str(userno)
        # Determine if the user exists and if he/she is the current target.
        if not listener.frameExists(absFrame) or not listener.frameExists(torsoFrame):
            if target == userno:
                "Lost user", userno
                target = None
            continue
        # The next if/elif keeps the robot focused on a single target at a time.
        # If there isn't currently a target, set the target to current candidate.
        if target is None:
            target = userno
        # Else there is a target. If the target is not the current candidate, they should be ignored.
        elif target != userno:
            continue
        try:
            now = rospy.Time.now()

            listener.waitForTransform(absFrame, torsoFrame, now, rospy.Duration(0.5))
            absToTorso = listener.lookupTransform(absFrame, torsoFrame, now)

            listener.waitForTransform(rHandFrame, torsoFrame, now, rospy.Duration(0.5))
            rHandToElbow = listener.lookupTransform(rHandFrame, torsoFrame, now)

            listener.waitForTransform(lHandFrame, torsoFrame, now, rospy.Duration(0.5))
            lHandToElbow = listener.lookupTransform(lHandFrame, torsoFrame, now)

            #print "had to elbow transfrom", rHandToElbow
            rpos1, rangle1 = rHandToElbow
            lpos1, langle1 = lHandToElbow

            # transform messages use odd coordinate system. This corrects
            # it. 
            # rotate robot towards intruder
            # Enable the next line to allow the robot to move, WHEN ON THE
            # FLOOR 
            if rpos1[0] > .3 and rpos1[1] < .1 and lpos1[0] < -.3 and lpos1 > -.1:
                ac.set_pos(255)
                print "intruder surrenders...retract killer arm"
                target = None
                rospy.sleep(.3)
            else:
                print "Intruder", userno, "found... taking aim"

                pos, angle = absToTorso
                torso_pos = Coord(x=pos[1], y=pos[0], z=pos[2])
                print torso_pos

                cmd = Twist()
                cmd.angular.z = torso_pos.x 
                pub.publish(cmd)
                
                target_arm_pos = 230. - sqrt(asin(torso_pos.y*0.10))*115.
                ac.set_pos(target_arm_pos)

                if fabs(torso_pos.x) < 0.10 and ac.is_ready(): 
                    print "Intruder", userno, "in sights. Fire!"
                    move_hand()
                    
                rospy.sleep(0.1)


        except TypeError as detail:
            print detail
        except tf.LookupException as detail:
            target = None
            print "The required transforms aren't being published."
            print detail
        except tf.ExtrapolationException as detail:
            target = None
            print "Extrapolation error."
            print detail
        except tf.Exception as detail:
            target = None
            print detail
        except KeyboardInterrupt:
            break
            
