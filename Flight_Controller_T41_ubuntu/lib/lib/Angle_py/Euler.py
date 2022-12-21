import time
import math

# class Euler():
#     def __init__(self,dt,alpha_ac_,alpha_mg_):
#         self.wx = 0.0
#         self.wy = 0.0
#         self.wz = 0.0

#         self.ax = 0.0
#         self.ay = 0.0
#         self.az = 0.0

#         self.bx = 0.0
#         self.by = 0.0
#         self.bz = 0.0

#         self.dt = 0.0

#         self.phi = 0.0
#         self.th  = 0.0
#         self.psi = 0.0

#         self.alpha_ac = 0.0
#         self.alpha_mg = 0.0
        
#         print("Estimator starts")
    
#     def estimate_gy(self):
#         d_phi_dt = self.wx + self.wy*math.sin(self.phi)*math.tan(self.th) + self.wz*math.cos(self.phi)*math.tan(self.th)
#         d_th_dt  = self.wy*math.cos(self.phi) - self.wz*math.sin(self.phi)
#         d_psi_dt = self.wy*math.sin(self.phi)/math.cos(self.th) + self.wz*math.cos(self.phi)/math.cos(self.th)

#         self.phi = self.phi + d_phi_dt*dt
#         self.th  = self.th  + d_th_dt *dt
#         self.psi = self.psi + d_psi_dt*dt

#     def correct_ac(self):
#         g = math.sqrt(self.ax**2 + self.ay**2 + self.az**2)
#         phi_ac = math.atan2(self.ay,self.az)
#         th_ac  = -math.asin(self.ax,g)
        
#         e_phi = self.phi - phi_ac
#         e_th  = self.th - th_ac

#         if(e_phi>math.pi):
#             e_phi = e_phi - 2.0*math.pi
#         elif(e_phi<=-math.pi):
#             e_phi = e_phi + 2.0*math.pi
        
#         if(e_th>math.pi):
#             e_th = e_th - 2.0*math.pi
#         elif(e_th<=-math.pi):
#             e_th = e_th + 2.0*math.pi
        
#         self.phi = self.phi + self.alpha_ac*e_phi
#         self.th  = self.th  + self.alpha_ac*e_th

#     def correct_mg(self):
#         tamp_x = self.by*math.cos(self.phi) - self.bz*math.sin(self.phi)
#         temp_y = self.bx*math.cos(self.th) + self.bz*math.cos(self.phi)*math.sin(self.th) + self.by*math.sin(self.phi)*math.sin(self.th)
#         psi_mg = math.atan2(tamp_x, temp_y)
#         e_psi = self.psi - psi_mg
        
#         if(e_psi>math.pi):
#             e_psi = e_psi - 2.0*math.pi
#         elif(e_psi<=-math.pi):
#             e_psi = e_psi + 2.0*math.pi
        
#         self.psi = self.psi + self.alpha_mg*e_psi

#     def estimate(self,wx_,wy_,wz_,ax_,ay_,az_,bx_,by_,bz_,dt_,alpha_ac_,alpha_mg_):
#         self.wx = wx_
#         self.wy = wy_
#         self.wz = wz_

#         self.ax = ax_
#         self.ay = ay_
#         self.az = az_

#         self.bx = bx_
#         self.by = by_
#         self.bz = bz_

#         self.dt = dt_

#         self.alpha_ac = alpha_ac_
#         self.alpha_mg = alpha_mg_

#         self.estimate_gy()
#         self.correct_ac()
#         self.correct_mg()

# dt = 1.0

# wx = 0.0
# wy = 0.0
# wz = 0.0

# ax = 0.0
# ay = 0.0
# az = 0.0

# bx = 0.0
# by = 0.0
# bz = 0.0

# phi = 0.0
# th = 0.0
# psi = 0.0

# estimator = Euler(0.0,0.0,0.0)

dt = 1.0/3.0
t_start = True
t_init = 0.0
t_now = 0.0
t_last = 0.0

def wait():
    global t_last
    while((time.time()-t_last)<dt):
        pass

    t_last = time.time()

while(True):
    if(t_start==True):
        t_init = time.time()
        t_start = False
    
    t_now = time.time() - t_init

    # phi, th, psi = estimator.estimate(wx,wy,wz,ax,ay,az,bx,by,bz,dt)
    # print(phi, th, psi)
    print(t_now)
    
    wait()
    