#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
fusion_error.py
Compara a pose estimada pelo EKF (/odometry/filtered, nav_msgs/Odometry)
com o ground truth do Gazebo (/gt/odom). Calcula:
  - erro de posicao (distancia euclidiana) ao longo do tempo
  - RMSE de posicao
  - erro final
  - erro maximo e percentil 95
  - erro de orientacao (yaw)
Ao encerrar (Ctrl+C) salva um CSV e gera os graficos.

Adaptado do localization_error.py da Parte 1.
A pose estimada e o ground truth sao alinhados pelo deslocamento a
partir do ponto inicial (assume que o robo nasce alinhado com a origem),
removendo o offset constante entre o frame odom e o frame do Gazebo.

Parametros (~private):
  ~config_name : rotulo da configuracao (odom | odom_imu | odom_imu_gps)
  ~output_dir  : pasta de saida (default: resultados/<config_name>)
"""
import os
import csv
import math

import rospy
import numpy as np
import matplotlib
matplotlib.use("Agg")          # backend sem display
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry


def yaw_from_quat(q):
    """Extrai o yaw (rad) de um quaternion geometry_msgs/Quaternion."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def ang_diff(a, b):
    """Diferenca angular normalizada para [-pi, pi]."""
    d = a - b
    return math.atan2(math.sin(d), math.cos(d))


class FusionError:
    def __init__(self):
        self.config_name = rospy.get_param("~config_name", "odom")
        default_dir = os.path.join(
            os.path.expanduser("~"), "resultados_fusao", self.config_name)
        self.output_dir = rospy.get_param("~output_dir", default_dir)
        if not os.path.isdir(self.output_dir):
            os.makedirs(self.output_dir)

        # ground truth mais recente
        self.gt = None
        # offsets de alinhamento (definidos na primeira amostra pareada)
        self.gt0 = None        # (x, y, yaw) inicial do ground truth
        self.est0 = None       # (x, y, yaw) inicial da estimativa

        # buffers
        self.t = []            # tempo (s) relativo ao inicio
        self.err_pos = []      # erro de posicao (m)
        self.err_yaw = []      # erro de orientacao (rad)
        self.gt_xy = []        # trajetoria gt alinhada
        self.est_xy = []       # trajetoria estimada alinhada
        self.t0 = None

        rospy.Subscriber("/gt/odom", Odometry, self.cb_gt, queue_size=20)
        rospy.Subscriber("/odometry/filtered", Odometry,
                         self.cb_est, queue_size=20)
        rospy.on_shutdown(self.finish)
        rospy.loginfo("[fusion_error] config='%s' -> %s",
                      self.config_name, self.output_dir)

    def cb_gt(self, msg):
        self.gt = msg

    def cb_est(self, msg):
        if self.gt is None:
            return

        gx = self.gt.pose.pose.position.x
        gy = self.gt.pose.pose.position.y
        gyaw = yaw_from_quat(self.gt.pose.pose.orientation)

        ex = msg.pose.pose.position.x
        ey = msg.pose.pose.position.y
        eyaw = yaw_from_quat(msg.pose.pose.orientation)

        # primeira amostra: fixa a origem de cada trajetoria
        if self.gt0 is None:
            self.gt0 = (gx, gy, gyaw)
            self.est0 = (ex, ey, eyaw)
            self.t0 = msg.header.stamp.to_sec()

        # desloca ambas para sua propria origem (remove offset constante)
        gxr = gx - self.gt0[0]
        gyr = gy - self.gt0[1]
        exr = ex - self.est0[0]
        eyr = ey - self.est0[1]

        dp = math.hypot(exr - gxr, eyr - gyr)
        dyaw = abs(ang_diff(eyaw - self.est0[2], gyaw - self.gt0[2]))
        t = msg.header.stamp.to_sec() - self.t0

        self.t.append(t)
        self.err_pos.append(dp)
        self.err_yaw.append(dyaw)
        self.gt_xy.append((gxr, gyr))
        self.est_xy.append((exr, eyr))

    # -------------------------------------------------------------
    def finish(self):
        n = len(self.err_pos)
        if n == 0:
            rospy.logwarn("[fusion_error] nenhuma amostra coletada.")
            return

        err = np.array(self.err_pos)
        eyaw = np.array(self.err_yaw)
        rmse = float(np.sqrt(np.mean(err ** 2)))
        final = float(err[-1])
        emax = float(np.max(err))
        p95 = float(np.percentile(err, 95))
        mean = float(np.mean(err))
        yaw_rmse = float(np.sqrt(np.mean(eyaw ** 2)))
        yaw_mean = float(np.mean(eyaw))

        # ---- CSV por amostra ----
        csv_path = os.path.join(self.output_dir, "erros.csv")
        with open(csv_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "err_pos_m", "err_yaw_rad",
                        "gt_x", "gt_y", "est_x", "est_y"])
            for i in range(n):
                w.writerow([self.t[i], self.err_pos[i], self.err_yaw[i],
                            self.gt_xy[i][0], self.gt_xy[i][1],
                            self.est_xy[i][0], self.est_xy[i][1]])

        # ---- resumo das metricas ----
        summary_path = os.path.join(self.output_dir, "metricas.csv")
        with open(summary_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["config", "amostras", "rmse_pos_m", "erro_final_m",
                        "erro_max_m", "p95_pos_m", "media_pos_m",
                        "rmse_yaw_rad", "media_yaw_rad", "rmse_yaw_deg"])
            w.writerow([self.config_name, n, rmse, final, emax, p95, mean,
                        yaw_rmse, yaw_mean, math.degrees(yaw_rmse)])

        rospy.loginfo("[fusion_error] === RESUMO (%s) ===", self.config_name)
        rospy.loginfo("  amostras   : %d", n)
        rospy.loginfo("  RMSE pos   : %.4f m", rmse)
        rospy.loginfo("  erro final : %.4f m", final)
        rospy.loginfo("  erro max   : %.4f m", emax)
        rospy.loginfo("  p95 pos    : %.4f m", p95)
        rospy.loginfo("  RMSE yaw   : %.4f rad (%.2f deg)",
                      yaw_rmse, math.degrees(yaw_rmse))

        # ---- graficos ----
        self._plot_traj()
        self._plot_err_time()
        rospy.loginfo("[fusion_error] resultados salvos em %s",
                      self.output_dir)

    def _plot_traj(self):
        gt = np.array(self.gt_xy)
        est = np.array(self.est_xy)
        plt.figure(figsize=(7, 6))
        plt.plot(gt[:, 0], gt[:, 1], "k-", lw=2, label="Ground truth")
        plt.plot(est[:, 0], est[:, 1], "r--", lw=1.5,
                 label="EKF (%s)" % self.config_name)
        plt.plot(gt[0, 0], gt[0, 1], "go", label="inicio")
        plt.axis("equal")
        plt.grid(True, ls=":")
        plt.xlabel("x (m)")
        plt.ylabel("y (m)")
        plt.title("Trajetoria - %s" % self.config_name)
        plt.legend()
        plt.tight_layout()
        plt.savefig(os.path.join(self.output_dir, "trajetoria.png"), dpi=130)
        plt.close()

    def _plot_err_time(self):
        t = np.array(self.t)
        err = np.array(self.err_pos)
        plt.figure(figsize=(8, 4))
        plt.plot(t, err, "b-", lw=1.2)
        plt.grid(True, ls=":")
        plt.xlabel("tempo (s)")
        plt.ylabel("erro de posicao (m)")
        plt.title("Erro de posicao x tempo - %s" % self.config_name)
        plt.tight_layout()
        plt.savefig(os.path.join(self.output_dir, "erro_tempo.png"), dpi=130)
        plt.close()


if __name__ == "__main__":
    rospy.init_node("fusion_error")
    FusionError()
    rospy.spin()
