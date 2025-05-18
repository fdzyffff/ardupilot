/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


#include "Copter.h"

UK230::UK230()
{
    tgt_pose_est = Vector2f(0.0f, 0.0f);
    tgt_pose_est.x = current_position.x;
    tgt_pose_est.y = current_position.y;

    tgt_pose_obs = Vector2f(0.0f, 0.0f);
    tgt_pose_obs.x = current_position.x;
    tgt_pose_obs.y = current_position.y;

    tgt_accel_obs = Vector2f(0.0f, 0.0f);

    con_pose = Vector2f(0.0f, 0.0f);
    con_accel = Vector2f(0.0f, 0.0f);

    relapose = Vector2f(0.0f, 0.0f);
    tgt_pose = Vector2f(0.0f, 0.0f);

    dot_hattheta = 0.0f;
    hattheta = 0.0f;
    dot_hatksi = 0.0f;
    hatksi = 0.0f;
}

// initialise
void UK230::init()
{
 
}

// update 
void UK230::update()
{
    relapose = current_position - tgt_pose;
    float distance = relapose.length();
    if (distance < 10.0f) {
        detect = 1.0f;
    }
    tve_topub = tgt_vel - tgt_vel;
    tae_topub = tgt_accel_est;

    // tgt_pose_obs publish
    // tgt_accel_obs publish
    con_pose = detect * (tgt_pose_obs - tgt_pose);
    con_accel = detect * (tgt_accel_obs - tgt_accel_est);
    for (uint8_t i = 0; i < uav_num; i++) {
        relapose = current_position - otheruav[i].position;
        float distance = relapose.length();
        if (i != this_id && 0.2f < distance && distance < R) {
            con_pose = con_pose + tgt_pose_obs - otheruav[i].tgt_pose_obs;
            con_accel = con_accel + tgt_accel_obs - otheruav[i].tgt_accel_obs;
        }
    }

    float sigma = 0.0001f;
    dot_tgt_pose_obs = - cp * con_pose - hattheta * con_pose/sqrtf(con_pose.length_squared() + sigma*sigma);
    dot_tgt_accel_obs = - ca * con_accel - hatksi * con_accel/sqrtf(con_accel.length_squared() + sigma*sigma);
    dot_hattheta = gp * con_pose.length();
    dot_hatksi = gp * con_accel.length();
        
                    
        # 2.2 基于公式（21）-（22）计算分布式观测器更新率以及观测值
            #原方案sigma是一个随时间指数减小的量，但在分母上可能趋近于0导致报错
            #修改为固定的小量
            self.sigma = 2 * math.exp(- 2 * 20) 
            self.dot_tgt_pose_obs = - self.cp * self.con_pose - self.hattheta * self.con_pose/pow((pow(self.con_pose[0],2) + pow(self.con_pose[1],2) + pow(self.sigma,2)),0.5)
            self.dot_tgt_accel_obs = - self.ca * self.con_accel - self.hatksi * self.con_accel/pow((pow(self.con_accel[0],2) + pow(self.con_accel[1],2) + pow(self.sigma,2)),0.5)
            self.dot_hattheta = self.gp * (pow(self.con_pose[0],2) + pow(self.con_pose[1],2))/pow((pow(self.con_pose[0],2) + pow(self.con_pose[1],2) + pow(self.sigma,2)),0.5)
            self.dot_hatksi = self.gp * (pow(self.con_accel[0],2) + pow(self.con_accel[1],2))/pow((pow(self.con_accel[0],2) + pow(self.con_accel[1],2) + pow(self.sigma,2)),0.5)
            self.tgt_pose_obs.pose.position.x = self.tgt_pose_obs.pose.position.x + self.dot_tgt_pose_obs[0]/self.f
            self.tgt_pose_obs.pose.position.y = self.tgt_pose_obs.pose.position.y + self.dot_tgt_pose_obs[1]/self.f
            self.tgt_accel_obs.pose.position.x = self.tgt_accel_obs.pose.position.x + self.dot_tgt_accel_obs[0]/self.f
            self.tgt_accel_obs.pose.position.y = self.tgt_accel_obs.pose.position.y + self.dot_tgt_accel_obs[1]/self.f
            self.hattheta = self.hattheta + self.dot_hattheta/self.f
            self.hatksi = self.hatksi + self.dot_hatksi/self.f

        # 3. 无标签目标包围控制器（输入为self.tgt_pose_obs与self.tgt_accel_obs，输出为速度指令self.cmd_vel_enu）
        # 3.1 无人机间斥力计算
            self.repulsiontotal = numpy.array([0.0,0.0])
            for i in range(0, self.uav_num):
                self.relapose = numpy.array([self.pose.pose.position.x - self.otheruav_pose[i].pose.position.x, self.pose.pose.position.y - self.otheruav_pose[i].pose.position.y])
                self.distance = pow (pow(self.relapose[0],2)+pow(self.relapose[1],2), 0.5)
            # 这部分计算斥力时加了一个保险，如果两架无人机之间距离小于self.d，则设置一个固定的较大斥力，防止无人机间发生碰撞
                if i!= self.id and self.d < self.distance < self.R:
                    self.repulsionsolo = self.kphi * (1/(self.distance-self.d)-1/(self.R-self.d))*self.relapose/self.distance
                    self.repulsiontotal = self.repulsiontotal + self.repulsionsolo
                if i!= self.id and 0.2 < self.distance <= self.d:
                    self.repulsionsolo = self.kphi * (1/(0.4)-1/(self.R-self.d))*self.relapose/self.distance
                    self.repulsiontotal = self.repulsiontotal + self.repulsionsolo
                # if i!= self.id and 0.2 < self.distance < self.R:
                #     self.repulsionsolo = self.kphi * (1/(self.distance-self.d)-1/(self.R-self.d))*self.relapose/self.distance
                #     self.repulsiontotal = self.repulsiontotal + self.repulsionsolo


        # 3.2 目标对无人机的吸引力计算
            self.xypose = numpy.array([self.pose.pose.position.x, self.pose.pose.position.y])
            self.xy_tgt_pose_obs = numpy.array([self.tgt_pose_obs.pose.position.x, self.tgt_pose_obs.pose.position.y])
            self.xy_tgt_accel_obs = numpy.array([self.tgt_accel_obs.pose.position.x, self.tgt_accel_obs.pose.position.y])
            self.dot_hatk = -self.c1*self.hatk - (self.c1*self.c2-self.c3)/self.c2*(self.repulsiontotal + self.c2 * (self.xy_tgt_pose_obs - self.xypose))
            self.hatk = self.hatk + self.dot_hatk/self.f
            self.attract = self.c2 * (self.xy_tgt_pose_obs - self.xypose) + self.hatk + self.xy_tgt_accel_obs
        
        # 3.3 计算加速度（x与y轴的最大加速度均设置为1m/s^2）
            self.cmd_accel_enu.linear.x = self.attract[0] + self.repulsiontotal[0]
            self.cmd_accel_enu.linear.y = self.attract[1] + self.repulsiontotal[1]
            if numpy.abs(self.cmd_accel_enu.linear.x) > 1.0:
                self.cmd_accel_enu.linear.x = numpy.sign(self.cmd_accel_enu.linear.x) * 1.0
            if numpy.abs(self.cmd_accel_enu.linear.y) > 1.0:
                self.cmd_accel_enu.linear.y = numpy.sign(self.cmd_accel_enu.linear.y) * 1.0
        
        # 3.4 计算并发布速度指令
            # 3.4.1 正常运行无人机的速度（x与y轴的最大速度均设置为1m/s）
            self.cmd_vel_enu.linear.x = self.cmd_vel_enu.linear.x + self.cmd_accel_enu.linear.x/self.f
            self.cmd_vel_enu.linear.y = self.cmd_vel_enu.linear.y + self.cmd_accel_enu.linear.y/self.f 
            #self.cmd_vel_enu.linear.x = self.vel.twist.linear.x + self.cmd_accel_enu.linear.x/self.f
            #self.cmd_vel_enu.linear.y = self.vel.twist.linear.y + self.cmd_accel_enu.linear.y/self.f
            if numpy.abs(self.cmd_vel_enu.linear.x) > 1.0:
                self.cmd_vel_enu.linear.x = numpy.sign(self.cmd_vel_enu.linear.x) * 1.0
            if numpy.abs(self.cmd_vel_enu.linear.y) > 1.0:
                self.cmd_vel_enu.linear.y = numpy.sign(self.cmd_vel_enu.linear.y) * 1.0
            # 3.4.2 设置3号无人机在第90秒离开无人集群（上机实验时根据飞行情况手控一架无人机远离集群）
            if self.id == 2 and self.count/self.f > 90.0:
                self.cmd_vel_enu.linear.x = 0.0
                self.cmd_vel_enu.linear.y = 1.0           

            self.cmd_vel_enu_pub.publish(self.cmd_vel_enu)
            

            try:
                rate.sleep()
            except:
                continue
}
