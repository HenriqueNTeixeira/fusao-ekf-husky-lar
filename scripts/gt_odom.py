#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gt_odom.py
Le /gazebo/model_states, encontra o modelo do Husky e republica a
pose verdadeira como /gt/odom (nav_msgs/Odometry).

Usado APENAS como ground truth de avaliacao - NUNCA entra no filtro.
Adaptado do projeto da Parte 1 (mapeamento+AMCL).
"""
import rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry


class GroundTruthOdom:
    def __init__(self):
        # nome do modelo do robo no Gazebo (padrao do Husky: "husky")
        self.model_name = rospy.get_param("~model_name", "husky")
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.child_frame_id = rospy.get_param("~child_frame_id", "base_link")

        self.pub = rospy.Publisher("/gt/odom", Odometry, queue_size=10)
        self.sub = rospy.Subscriber("/gazebo/model_states",
                                    ModelStates, self.cb, queue_size=10)
        self._warned = False
        rospy.loginfo("[gt_odom] publicando /gt/odom para o modelo '%s'",
                      self.model_name)

    def cb(self, msg):
        try:
            idx = msg.name.index(self.model_name)
        except ValueError:
            if not self._warned:
                rospy.logwarn("[gt_odom] modelo '%s' nao encontrado. "
                              "Modelos: %s", self.model_name, msg.name)
                self._warned = True
            return

        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose = msg.pose[idx]
        odom.twist.twist = msg.twist[idx]
        self.pub.publish(odom)


if __name__ == "__main__":
    rospy.init_node("gt_odom")
    GroundTruthOdom()
    rospy.spin()
