#!/usr/bin/env python

import rospy

from std_msgs.msg import String
from scitos_virtual_bumper.msg import virtualbumperreport
from scitos_virtual_bumper.msg import virtualbumperevent
from mongodb_store.message_store import MessageStoreProxy

import sensor_msgs.msg

class virtual_bumper_report(object):

    def __init__(self, name) :
        self.closest_node = "Unknown"
        self.info=''
        
        self.image_available=False
        
        self.camera_topic=rospy.get_param('~camera_topic','/head_xtion/rgb/image_color')
        #Subscribing to Localisation Topics
        rospy.loginfo("Subscribing to Localisation Topics")
        rospy.Subscriber('/closest_node', String, self.cNodeCallback)
        rospy.loginfo(" ...done")

        rospy.loginfo("Subscribing to virtual bumper Topics")
        rospy.Subscriber('/virtual_bumper_event', virtualbumperevent, self.EventCallback)
        rospy.Subscriber('/virtual_bumper_report', virtualbumperreport, self.ReportCallback)
        rospy.loginfo(" ...done")
        
        rospy.loginfo("Creating publisher for Notifications")
        self.text_notification_pub = rospy.Publisher('/notification', String, queue_size=1)
        self.img_notification_pub = rospy.Publisher('/notification_image', sensor_msgs.msg.Image, queue_size=1)
        rospy.loginfo(" ...done")

        rospy.loginfo("Creating Message store client")
        try:
            self.msg_store = MessageStoreProxy(collection='virtual_bumper')
            self.msgstore=True
        except:
            self.msgstore=False
            rospy.logerr("Message Store not available")
        
        rospy.loginfo("All Done ...")
        rospy.spin()


    def cNodeCallback(self, msg):
        """
         Current Node CallBack
         
        """
        self.closest_node = msg.data



    def EventCallback(self, msg):
        """
         Virtual Bumper Event CallBack
         
        """
        if msg.freeRunByVirtualBumper and msg.freeRunStarted:
            self.info='A virtual bumper event has been trigered at '+str(rospy.Time.now().secs)
            print self.info
            try:
                self.triger_image=rospy.wait_for_message(self.camera_topic,sensor_msgs.msg.Image, timeout=1)
                self.image_available=True
            except:
                rospy.logerr("%s image not available", self.camera_topic)
                self.image_available=False
            
            meta = {}
            meta["type"] = "Virtual Bumper Event"
            meta["text"] = self.info
        
            if self.msgstore:
                self.msg_store.insert(msg,meta)                 
        
        
    
    def ReportCallback(self, msg):
        """
         Virtual Bumper Report CallBack
         
        """      
        self.info=self.info+' and lasted until '+str(rospy.Time.now().secs)
        print self.info        
        
        self.text_notification_pub.publish(self.info)

        meta = {}
        meta["type"] = "Virtual Bumper Report"
        meta["text"] = self.info
        meta["image"] = self.image_available
        
        if self.image_available:
            self.img_notification_pub.publish(self.triger_image)
            if self.msgstore:
                self.msg_store.insert(self.triger_image,meta)

        if self.msgstore:
            self.msg_store.insert(msg,meta)



if __name__ == '__main__':
    rospy.init_node('virtual_bumper_report')
    server = virtual_bumper_report(rospy.get_name())
