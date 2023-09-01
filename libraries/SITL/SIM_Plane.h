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
/*
  simple plane simulator class
*/

#pragma once

#include "SIM_Aircraft.h"
#include "SIM_ICEngine.h"
#include <Filter/LowPassFilter.h>

namespace SITL {

/*
  a very simple plane simulator
 */
class Plane : public Aircraft {
public:
    Plane(const char *frame_str);

    /* update model by one time step */
    virtual void update(const struct sitl_input &input) override;

    /* static object creator */
    static Aircraft *create(const char *frame_str) {
        return new Plane(frame_str);
    }

protected:
    const float hover_throttle = 0.7f;
    const float air_density = 1.225; // kg/m^3 at sea level, ISA conditions
    float angle_of_attack;
    float beta;

    struct {
        // 20230821-羚控气动数据

        float s = 4.6;//0.45;机翼面积
        float b = 7.2;//1.88;翼展
        float c = 0.34;//0.24;平均气动弦
        float Ixx = 147;//x方向转动惯量kgm2
        float Iyy = 142;//y方向转动惯量
        float Izz = 264;//z方向转动惯量
        float c_lift_0 = 0.1053;//0.56;0度迎角下的升力系数
        float c_lift_deltae = 0.4698;//0;//升降舵对升力系数的影响,每弧度舵偏角
        float c_lift_a = 4.76;//6.9;升力线斜率，弧度
        float c_lift_q = 0;//动导数
        float mcoeff = 50;//分离因子？
        float oswald = 0.8;//0.9;//和展弦比相关，展弦比2.64
        float alpha_stall = 0.2443;//0.4712;失速迎角14度
        float c_drag_q = 0;//动导数
        float c_drag_deltae = 0.0;//升降舵对阻力系数的影响
        float c_drag_p = 0.038;//0.1;//零升阻力系数
        float c_y_0 = 0;
        float c_y_b = -0.8366;//-0.98;侧力导数
        float c_y_p = 0;//动导数
        float c_y_r = 0;//动导数
        float c_y_deltaa = 0;//副翼对侧力的影响
        float c_y_deltar = -0.2;//方向舵对侧力的影响
        float c_l_0 = 0;
        float c_l_p = -0.265;//-1.0;//动导数
        float c_l_b = -0.4;//-0.12;//侧滑对滚转力矩的影响
        float c_l_r = 0.14;//偏航速度对滚转的影响，借鉴
        float c_l_deltaa = 0.057;//0.25; 正负有待确认
        float c_l_deltar = -0.037;//方向舵对滚转力矩影响
        float c_m_0 = 0.1015;//0.045;//零迎角下的力矩系数
        float c_m_a = -3.38;//-0.7;//静导数，纵向稳定性相关，弧度单位，接近中立稳定
        float c_m_q = -20;//-20;动导数
        float c_m_deltae = 1.169;//1.0;  每弧度单位(0.008/度),默认的1.0对应0.0174，正负？
        float c_n_0 = 0;
        float c_n_b = 0.23;// 0.25;//侧滑对偏航力矩的影响
        float c_n_p = 0.022;//动导数，借鉴
        float c_n_r = -0.058;//-1;动导数
        float c_n_deltaa = 0.00;//副翼对偏航力矩的影响
        float c_n_deltar = 0.023;//0.1; 0.0013*57.3,弧度，方向舵舵效
        float deltaa_max = 0.3491;//最大20度
        float deltae_max = 0.3491;//最大20度
        float deltar_max = 0.3491;//最大20度
        // the X CoG offset should be -0.02, but that makes the plane too tail heavy
        // in manual flight. Adjusted to -0.15 gives reasonable flight
        //Vector3f CGOffset{-0.15, 0, -0.05};
        Vector3f CGOffset{0, 0, -0.05};
    } coefficient;

    float thrust_scale;
    bool reverse_thrust;
    bool elevons;
    bool vtail;
    bool dspoilers;
    bool reverse_elevator_rudder;
    bool ice_engine;
    bool tailsitter;
    bool copter_tailsitter;
    bool have_launcher;
    float launch_accel;
    float launch_time;
    uint64_t launch_start_ms;

    const uint8_t throttle_servo = 2;
    const int8_t choke_servo = 14;
    const int8_t ignition_servo = 12;
    const int8_t starter_servo = 13;
    const float slewrate = 100;
    ICEngine icengine{
        throttle_servo,
        choke_servo,
        ignition_servo,
        starter_servo,
        slewrate,
        true
    };

    float liftCoeff(float alpha) const;
    float dragCoeff(float alpha) const;
    Vector3f getForce(float inputAileron, float inputElevator, float inputRudder) const;
    Vector3f getTorque(float inputAileron, float inputElevator, float inputRudder, float inputThrust, const Vector3f &force) const;
    void calculate_forces(const struct sitl_input &input, Vector3f &rot_accel);
};

} // namespace SITL
