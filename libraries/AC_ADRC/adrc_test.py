from matplotlib import pyplot as plt
import numpy as np
import math


class Vector3F():
    def __init__(self, x, y , z):
        self.x = x
        self.y = y
        self.z = z

    def print(self):
        text = " [%0.6f, %0.6f, %0.6f]\n"%(self.x, self.y, self.z)
        print (text)

    def add(self, val_in):
        self.x += val_in.x
        self.y += val_in.y
        self.z += val_in.z

    def multi(self, val_in):
        self.x *= val_in
        self.y *= val_in
        self.z *= val_in

class Matrix3():
    def __init__(self, ax, ay, az, bx, by, bz, cx, cy, cz):
        self.a = Vector3F(ax, ay, az)
        self.b = Vector3F(bx, by, bz)
        self.c = Vector3F(cx, cy, cz)

    def det(self):
        return self.a.x * (self.b.y * self.c.z - self.b.z * self.c.y) \
             + self.a.y * (self.b.z * self.c.x - self.b.x * self.c.z) \
             + self.a.z * (self.b.x * self.c.y - self.b.y * self.c.x);

    def inverse(self, inv):
        d = self.det();
        print ("d : %f"%d)

        inv.a.x = (self.b.y * self.c.z - self.c.y * self.b.z) / d;
        inv.a.y = (self.a.z * self.c.y - self.a.y * self.c.z) / d;
        inv.a.z = (self.a.y * self.b.z - self.a.z * self.b.y) / d;
        inv.b.x = (self.b.z * self.c.x - self.b.x * self.c.z) / d;
        inv.b.y = (self.a.x * self.c.z - self.a.z * self.c.x) / d;
        inv.b.z = (self.b.x * self.a.z - self.a.x * self.b.z) / d;
        inv.c.x = (self.b.x * self.c.y - self.c.x * self.b.y) / d;
        inv.c.y = (self.c.x * self.a.y - self.a.x * self.c.y) / d;
        inv.c.z = (self.a.x * self.b.y - self.b.x * self.a.y) / d;
        return True

    def multi(self, val_in):
        return Vector3F(self.a.x * val_in.x + self.a.y * val_in.y + self.a.z * val_in.z, \
                        self.b.x * val_in.x + self.b.y * val_in.y + self.b.z * val_in.z, \
                        self.c.x * val_in.x + self.c.y * val_in.y + self.c.z * val_in.z);
    def print(self):
        text = " [%0.6f, %0.6f, %0.6f\n %0.6f, %0.6f, %0.6f\n %0.6f, %0.6f, %0.6f]\n"%(self.a.x, self.a.y, self.a.z, self.b.x, self.b.y, self.b.z, self.c.x, self.c.y, self.c.z)
        print (text)

def update_all(target, measurement, dt):
    # get info from last step
    global G_z1_k0
    global G_z2_k0
    global G_z3_k0
    global G_omega0
    global G_omegac
    global G_z1_k1
    global G_z2_k1
    global G_z3_k1
    global G_u_k0
    global G_u_k1
    _target = target
    _b0 = 80

    G_z1_k0 = G_z1_k1;
    G_z2_k0 = G_z2_k1;
    G_z3_k0 = G_z3_k1;

    G_u_k0 = G_u_k1;
    _y_k0 = measurement;
    _r_k1 = _target;

    _beta1 = 3.0*G_omega0;
    _beta2 = 3.0*G_omega0*G_omega0;
    _beta3 = G_omega0*G_omega0*G_omega0;

    # calculate output
    a11 = 1-dt*_beta1;
    a12 = dt;
    a13 = 0.0;

    a21 = -dt*_beta2;
    a22 = 1.0;
    a23 = dt;

    a31 = -dt*_beta3;
    a32 = 0.0;
    a33 = 1.0;

    M_A = Matrix3(a11, a12, a13, \
                  a21, a22, a23, \
                  a31, a32, a33);

    V_Z_k0 = Vector3F(G_z1_k0, G_z2_k0, G_z3_k0)

    V_U = Vector3F(0.0, dt*_b0, 0.0)
    V_U.multi(G_u_k0)

    V_Y = Vector3F(dt*_beta1, dt*_beta2, dt*_beta3)
    V_Y.multi(_y_k0)

    V_Z_k1 = (M_A.multi(V_Z_k0))
    V_Z_k1.add(V_U)
    V_Z_k1.add(V_Y)

    M_A_inv = Matrix3(0, 0, 0, 0, 0, 0, 0, 0, 0) ;
    print ("---------------------------------------")
    G_z1_k1 = V_Z_k1.x;
    G_z2_k1 = V_Z_k1.y;
    G_z3_k1 = V_Z_k1.z;
    # G_z1_k1 = _y_k0
    # G_z2_k1 = G_z2_k0
    V_Z_k1.print()

    _kp = G_omegac*G_omegac;
    _kd = 2*G_omegac;
    G_u_k1 = (-G_z3_k1 + _kp*(_r_k1-G_z1_k1)-_kd*G_z2_k1)/_b0;

    return G_u_k1;

def update_plant(u_in, dt):
    global G_plant_ret
    G_plant_ret = G_plant_ret + 5*u_in*dt# - G_plant_ret*G_plant_ret*0.01

def update_target(t, dt):
    global G_target_ret
    if (t < 20.0*dt):
        G_target_ret = 0.0
    else :
        G_target_ret = 1.0
    # G_target_ret = 5*t + np.sin(50*t)

def init():
    global G_z1_k0
    global G_z2_k0
    global G_z3_k0
    global G_omega0
    global G_omegac
    global G_z1_k1
    global G_z2_k1
    global G_z3_k1
    global G_u_k0
    global G_u_k1
    global G_plant_ret
    global G_target_ret

    G_z1_k0 = 0.0
    G_z2_k0 = 0.0
    G_z3_k0 = 0.0
    G_omega0 = 50
    G_omegac = 20
    G_z1_k1 = 0.0
    G_z2_k1 = 0.0
    G_z3_k1 = 0.0
    G_u_k0 = 0.0
    G_u_k1 = 0.0

    G_plant_ret = 0.0
    G_target_ret = 0.0


def run():
    init()
    delta_t = 0.0025
    x = []
    y1 = []
    y2 = []
    y3 = []
    for i in range(500):
        t = i*delta_t
        update_plant(G_u_k1, delta_t) # update G_plant_ret (it is y@k)
        update_target(t, delta_t) # update the target rate, now it is const or stair
        update_all(G_target_ret, G_plant_ret, delta_t) # update the control output, it is u@k+1

        print ("Time: %0.6f, Target: %0.6f, Control: %0.6f, Measurement: %0.6f"%(t, G_target_ret, G_u_k1, G_plant_ret))
        x.append(t)
        y1.append(G_target_ret)
        y2.append(G_plant_ret)
        y3.append(G_u_k1*0.01)


    plt.plot(x,y1)
    plt.xlabel("time")
    plt.ylabel("Target")
    plt.title('ADRC')
    plt.plot(x,y2)
    plt.plot(x,y3)
    #使用show展示图像
    plt.show()

def test():
    a11 = 1
    a12 = 1
    a13 = 1
    c1 = 1

    a21 = 2
    a22 = 1
    a23 = 1
    c2 = 2

    a31 = 3
    a32 = 2
    a33 = 0
    c3 = 7

    M_A = Matrix3(a11, a12, a13, \
                  a21, a22, a23, \
                  a31, a32, a33);
    V_C = Vector3F(c1, c2, c3);

    M_A_inv = Matrix3(0, 0, 0, 0, 0, 0, 0, 0, 0) ;
    V_X = Vector3F(0, 0, 0)
    if (M_A.inverse(M_A_inv)) :
        V_X = M_A_inv.multi(V_C);
        M_A.print()
        M_A_inv.print()
        V_C.print()
    print ((V_X.x, V_X.y, V_X.z))
    return V_X;

G_z1_k0 = 0.0
G_z2_k0 = 0.0
G_z3_k0 = 0.0
G_omega0 = 0.5
G_omegac = 0.4
G_z1_k1 = 0.0
G_z2_k1 = 0.0
G_z3_k1 = 0.0
G_u_k0 = 0.0
G_u_k1 = 0.0
G_plant_ret = 0.0
G_target_ret = 0.0

run()
# test()

