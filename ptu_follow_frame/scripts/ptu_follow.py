#!/usr/bin/env python
import rospy
import tf
from ptu_follow_frame.srv import StartFollowing, StartFollowingResponse
from std_srvs.srv import Empty, EmptyResponse
from sensor_msgs.msg import JointState
import math
import threading

SOFT_PAN_LIMIT_UPPER = 1.9
SOFT_PAN_LIMIT_LOWER = -1.9
SOFT_TILT_LIMIT_UPPER = 0.2
SOFT_TILT_LIMIT_LOWER = -0.1
MAX_PAN_VEL = 2
MIN_PAN_VEL = -2
MAX_TILT_VEL = 2
MIN_TILT_VEL = -2
PAN_ACCEL=1
TILT_ACCEL=1

class FollowFrame(object):
    """
    Controls a PTU to keep the origin in a given frame in the centre
    of the PTU frame.
    """

    def __init__(self):
        """Constructor"""
        self._target_frame = None
        self._follow_frequency = 10 # Hz
        self._running = False
        self._cmd_state = JointState()
        self._cmd_state.name = ["pan", "tilt"]
        self._cmd_state.position = [0, 0]
        self._cmd_state.velocity = [0, 0]
        self._last_cmd_vel=[0,0]
        self._ptu_state = None
        
        if rospy.get_name() == "":
            print "FollowFrame class needs to be created in a ROS node."
            print "Make sure rospy.init_node() is called before instantiation."
            return
        
        # set up services
        self._start_srv = rospy.Service(rospy.get_name()+"/set_following",
                                        StartFollowing,
                                        self._on_set_following_srv_cb)
        self._stop_srv = rospy.Service(rospy.get_name()+"/stop_following",
                                       Empty,
                                       self._on_stop_following_srv_cb)
        
        # command PTU publisher
        self._ptu_cmd = rospy.Publisher("/ptu/cmd_vel", JointState, queue_size=1)
        # state sub
        self._ptu_sub = rospy.Subscriber("/ptu/state", JointState,
                                                self._on_ptu_state)
    
    def _on_set_following_srv_cb(self, req):
        rospy.loginfo("Following activated on frame %s"% req.target_frame_id)
        response = StartFollowingResponse()
        r = self.set_following(req.target_frame_id)
        response.response = r[0]
        response.activated = r[1]
        return response
    
    def _on_stop_following_srv_cb(self, req):
        rospy.loginfo("Stopping PTU following.")
        self.stop_following()
        return EmptyResponse()
    
    def _on_ptu_state(self, state):
        self._ptu_state = state
    
    def _main_loop(self):
        rater = rospy.Rate(self._follow_frequency)
        while self._running:
            t = self._tf.getLatestCommonTime("/ptu_mount", self._target_frame)
            position, quaternion = self._tf.lookupTransform("/ptu_mount",
                                                           self._target_frame,
                                                           t)

            yaw = math.atan2(position[1], position[0])
            roll = math.atan2(position[2], position[0])
            print "=> drive yaw=", yaw
            print "=> drive roll=", roll

            if self._ptu_state is None:
                rospy.logwarn("Not got any PTU status info....")
                continue

            # If the yaw angle would pass the back, flip it
            new_pan=self._ptu_state.position[0] + yaw
            if new_pan < -math.pi:
                yaw=2*math.pi-yaw
            if new_pan > math.pi:
                yaw=-2*math.pi+yaw

            self._cmd_state.velocity[0] = yaw
            self._cmd_state.velocity[0] = min(self._cmd_state.velocity[0], MAX_PAN_VEL)
            self._cmd_state.velocity[0] = max(self._cmd_state.velocity[0], MIN_PAN_VEL)
            self._cmd_state.velocity[1] = -roll
            self._cmd_state.velocity[1] = min(self._cmd_state.velocity[1], MAX_TILT_VEL)
            self._cmd_state.velocity[1] = max(self._cmd_state.velocity[1], MIN_TILT_VEL)


            # Check the acceleration limits
            if self._cmd_state.velocity[0] - self._last_cmd_vel[0] < - PAN_ACCEL:
                self._cmd_state.velocity[0] = self._last_cmd_vel[0] - PAN_ACCEL
            if self._cmd_state.velocity[0] - self._last_cmd_vel[0] > PAN_ACCEL:
                self._cmd_state.velocity[0] = self._last_cmd_vel[0] + PAN_ACCEL
            if self._cmd_state.velocity[1] - self._last_cmd_vel[1] < - TILT_ACCEL:
                self._cmd_state.velocity[1] = self._last_cmd_vel[1] - TILT_ACCEL
            if self._cmd_state.velocity[1] - self._last_cmd_vel[1] > TILT_ACCEL:
                self._cmd_state.velocity[1] = self._last_cmd_vel[1] + TILT_ACCEL


            # Cut it off, dont try and drive beyond the limits.
            if (self._ptu_state.position[0] > SOFT_PAN_LIMIT_UPPER):
                self._cmd_state.velocity[0] = min(0, self._cmd_state.velocity[0])
            if (self._ptu_state.position[0] < SOFT_PAN_LIMIT_LOWER):
                self._cmd_state.velocity[0] = max(0, self._cmd_state.velocity[0])
            if (self._ptu_state.position[1] > SOFT_TILT_LIMIT_UPPER):
                self._cmd_state.velocity[1] = min(0, self._cmd_state.velocity[1])
            if (self._ptu_state.position[1] < SOFT_TILT_LIMIT_LOWER):
                self._cmd_state.velocity[1] = max(0, self._cmd_state.velocity[1])

            self._ptu_cmd.publish(self._cmd_state)
            self._last_cmd_vel[0]=self._cmd_state.velocity[0]
            self._last_cmd_vel[1]=self._cmd_state.velocity[1]

            try:
                rater.sleep()
            except:
                # Get out of here.#
                self._running = False
    
    def set_following(self, frame_id):
        if self._running:
            return "already active.", False
        
        self._target_frame = frame_id
        self._tf = tf.TransformListener()
        rospy.loginfo("PTU Frame follower initialising TF...")
        rospy.sleep(1)
        rospy.loginfo("ok.")
        if not self._tf.frameExists(frame_id):
            return ("frame does not exist. maybe? I know of:\n" +
                    self._tf.allFramesAsString(),
                    False)
        
        # Run the main loop in a thread
        self._running = True
        self._running_thread = threading.Thread(target=self._main_loop)
        self._running_thread.start()
        
        return "ok", True
                                                
    
    def stop_following(self):
        if not self._running:
            return
        
        self._tf = None
        self._target_frame = None
        
        # Stop the main loop thread
        self._running = False
        self._running_thread.join()
        
    def set_cmd_freq(self, hz):
        if not self._running:
            self._follow_frequency = hz
        else:
            prev_follow_frame = self._target_frame
            self.stop_following()
            self._follow_frequency = hz
            self.set_following(prev_follow_frame)

if __name__ == '__main__':
    rospy.init_node("ptu_follow_frame")
    follower = FollowFrame()
    follower.set_cmd_freq(50)
    rospy.spin()
