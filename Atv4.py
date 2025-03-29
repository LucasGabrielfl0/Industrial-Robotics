from DobotEDU import *
import math

L1 = 150
L2 = 150
X_ACT = 90              # Actuator Offset
Z_ACT = 5               # Actuator Offset


def GetInverse(xp,yp,zp,r_dg):
  # Checks if the point is Within work Volume
  # print("[System]: Point is Out of Bounds, choose again")

  # Remove offset (Actuator)
  V_axys = math.sqrt((xp**2) + (yp**2))
  V_arm= V_axys - X_ACT
  Z_arm= zp + Z_ACT

  xyz_2= (Z_arm**2) + (V_arm**2)
  xyz = math.sqrt(xyz_2)

  #
  aux1 =  math.atan2( Z_arm, V_arm)
  aux2  = ( xyz_2 + (L1**2)  -  (L2**2) ) 
  aux3= 2*L1*math.sqrt(xyz_2)
  aux4 = xyz_2 - (L1**2)  -  (L2**2)  

  t1= aux1 + math.acos(aux2/aux3)
  t2= math.acos(aux4/(2*L1*L2))  

  j1= math.atan2(yp, xp)
  j2=(math.pi/2) - t1
  j3= t2 -(math.pi/2)+j2
  
  # All angles in Degrees
  j1_dg= math.degrees(j1)
  j2_dg= math.degrees(j2)
  j3_dg= math.degrees(j3)
  j4_dg=  r_dg - j1_dg

  
  # Trunc to 0.00
  j1_dg=round(j1_dg, 2)
  j2_dg=round(j2_dg, 2)
  j3_dg=round(j3_dg, 2)
  j4_dg=round(j4_dg, 2)

  return j1_dg, j2_dg, j3_dg, j4_dg

#=========================================================================================#
# Main Code
px=250
py= 55
pz =65
pr=90

m_lite.set_ptpcmd(ptp_mode=2, x= px, y= py, z=pz, r=pr) # Compare Result

j1,j2,j3,j4 = GetInverse(px, py, pz, pr)                         # Get Inverse
print("J1: ", j1, "J2: ", j2,  "J3: ", j3, "J4: ", j4)
#m_lite.set_ptpcmd(ptp_mode=5, x= j1, y= j2, z=j3, r=j4) 
