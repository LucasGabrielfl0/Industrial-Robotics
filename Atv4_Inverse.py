from DobotEDU import *
import math
# Inverse Kinemactics Algorithm for Magician Lite robot, made for the Simulation.
# 






L1 = 150
L2 = 150
X_ACT = 90              # Actuator Offset
Z_ACT = 5               # Actuator Offset

#===================== Aux Functions =======================#
def GetInverse(xp,yp,zp,r_dg):
  # Checks if the point is Within work Volume
  # print("[System]: Point is Out of Bounds, choose again")

  # Remove offset (Actuator)
  Vp = math.sqrt((xp**2) + (yp**2))
  V_arm= Vp - X_ACT
  Z_arm= zp + Z_ACT

  
  Hip_2= (Z_arm**2) + (V_arm**2)
  Hip= math.sqrt(Hip_2)

  # Auxiliar Functions
  aux1  = Hip_2 + (L1**2) - (L2**2)
  aux2  = Hip_2 - (L1**2) - (L2**2)  

  # Angles in the ZxV plane
  alpha_rad  = math.acos(aux1/(2*L1*Hip))
  beta_rad   = math.acos(aux2/(-2*L1*L2))
  gamma_rad  = math.atan2( Z_arm, V_arm)


  # Angles
  j1 = math.atan2(yp, xp)
  j2 = (math.pi/2) - alpha_rad - gamma_rad
  j3 = (math.pi/2) + j2 - beta_rad
  
  # All angles in Degrees
  j1_dg= math.degrees(j1)
  j2_dg= math.degrees(j2)
  j3_dg= math.degrees(j3)
  j4_dg=  r_dg - j1_dg

  
  # Trunc to 0.000
  j1_dg=round(j1_dg, 3)
  j2_dg=round(j2_dg, 3)
  j3_dg=round(j3_dg, 3)
  j4_dg=round(j4_dg, 3)

  return j1_dg, j2_dg, j3_dg, j4_dg

#=========================================================================================#
# Main Code
px=250
py= 0
pz =50
pr=0

j1,j2,j3,j4 = GetInverse(px, py, pz, pr)                         # Get Inverse
print("J1: ", j1, "J2: ", j2,  "J3: ", j3, "J4: ", j4)    
m_lite.set_ptpcmd(ptp_mode=5, x= j1, y= j2, z=j3, r=j4) 
