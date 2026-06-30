#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
drive_path.py
Dirige o Husky por um trajeto FIXO (malha aberta), publicando uma
sequencia pre-definida de comandos de velocidade. Garante o MESMO
trajeto em todos os experimentos -> comparacao justa entre as configs.

ATENCAO: malha aberta NAO desvia de obstaculos. Use numa area livre
do laboratorio, ou prefira gravar/repetir um rosbag (ver README).

Parametros:
  ~cmd_topic : topico de comando (default /cmd_vel)
  ~rate_hz   : frequencia de publicacao (default 20)
  ~v         : velocidade linear base (m/s, default 0.4)
  ~w         : velocidade angular base (rad/s, default 0.5)

O trajeto padrao e um quadrado que retorna perto do ponto inicial
(bom para avaliar o "erro final").
"""
import rospy
from geometry_msgs.msg import Twist


def main():
    rospy.init_node("drive_path")
    cmd_topic = rospy.get_param("~cmd_topic", "/cmd_vel")
    rate_hz = rospy.get_param("~rate_hz", 20.0)
    v = rospy.get_param("~v", 0.4)
    w = rospy.get_param("~w", 0.5)

    pub = rospy.Publisher(cmd_topic, Twist, queue_size=10)
    rate = rospy.Rate(rate_hz)

    # tempo para girar ~90 graus: t = (pi/2) / w
    import math
    t_turn = (math.pi / 2.0) / w

    # sequencia: (linear_x, angular_z, duracao_s)
    # quadrado: 4 lados de ~2.4 m + 4 giros de 90 graus
    segmentos = [
        (v,   0.0, 6.0),
        (0.0, w,   t_turn),
        (v,   0.0, 6.0),
        (0.0, w,   t_turn),
        (v,   0.0, 6.0),
        (0.0, w,   t_turn),
        (v,   0.0, 6.0),
        (0.0, w,   t_turn),
    ]

    rospy.loginfo("[drive_path] aguardando 3s antes de iniciar...")
    rospy.sleep(3.0)
    rospy.loginfo("[drive_path] iniciando trajeto em '%s'", cmd_topic)

    for i, (lx, az, dur) in enumerate(segmentos):
        if rospy.is_shutdown():
            break
        rospy.loginfo("[drive_path] segmento %d/%d: v=%.2f w=%.2f por %.1fs",
                      i + 1, len(segmentos), lx, az, dur)
        msg = Twist()
        msg.linear.x = lx
        msg.angular.z = az
        t_end = rospy.Time.now() + rospy.Duration(dur)
        while not rospy.is_shutdown() and rospy.Time.now() < t_end:
            pub.publish(msg)
            rate.sleep()

    # para o robo
    pub.publish(Twist())
    rospy.loginfo("[drive_path] trajeto concluido, robo parado.")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
