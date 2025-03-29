# version: Python3
from DobotEDU import *
import math

# Choose Mode 
Current_Mode = 1        # 1: D to F || 2: F to D 
if (Current_Mode == 1):
  X_1, Y_1, Z_1, R_1 =  [ 274 , -125,  -30, 0]   # Position of the PICK UP SPOT [D]
  DIST_F_D    =  192                                           # Distance between D and F blocks
else:
  DIST_F_D    =  -192      # Distance between D and F blocks
  X_1, Y_1, Z_1, R_1 =  [ 274 , -125-DIST_F_D,  -30, 0]   # Position of the PICK UP SPOT [D]

# Mechanical Parameters
L1 = 150
L2 = 150
X_ACT = 90              # Actuator Offset
Z_ACT = 5               # Actuator Offset


# Blocks
DIST_CONS   = 20        # Distance Between 2 consecutive blocks (row or column)
DIST_JUMP   =  14.5     # Distance (Z) for the dive to catch the Block


#===================== Aux Functions =======================#
def GetInverse(xp,yp,zp,r_dg):
  # Checks if the point is Within work Volume
  # print("[System]: Point is Out of Bounds, choose again")

  # Remove offset (Actuator)
  V_axys = math.sqrt((xp**2) + (yp**2))
  V_arm= V_axys - X_ACT
  Z_arm= zp + Z_ACT

  xyz_2= (Z_arm**2) + (V_arm**2)

  # Auxiliar Functions
  aux1 =  math.atan2( Z_arm, V_arm)
  aux2  = ( xyz_2 + (L1**2)  -  (L2**2) ) 
  aux3= 2*L1*math.sqrt(xyz_2)
  aux4 = xyz_2 - (L1**2)  -  (L2**2)  

  # Angles in the ZxV plane
  t1= aux1 + math.acos(aux2/aux3)
  t2= math.acos(aux4/(2*L1*L2))  

  # Angles
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


# Goes to a point and Picks [Mode = true] or Drops [Mode = False] a block
def Jump_to(px, py, pz, r_dg, mode):
  # Get Inverse
  j1_high, j2_high, j3_high, j4_high = GetInverse(px, py, pz, r_dg)
  j1_low,  j2_low,  j3_low,  j4_low  = GetInverse(px, py, pz-DIST_JUMP, r_dg)
  
  if(mode ==1):       # Catch Block
    mode = True
    pass

  else:               # Release Block
    mode = False
    pass

  m_lite.set_ptpcmd(ptp_mode=5, x=j1_high, y=j2_high, z=j3_high, r=j4_high)   # Go Above Position
  m_lite.wait(delay=0.5)                                                      # Wait

  m_lite.set_ptpcmd(ptp_mode=5, x=j1_low, y=j2_low, z=j3_low, r=j4_low)       # Dive
  m_lite.set_endeffector_suctioncup(enable=True, on=mode)                     # Grab
  m_lite.wait(delay=0.5)                                                      # Wait
  
  m_lite.set_ptpcmd(ptp_mode=5, x=j1_high, y=j2_high, z=j3_high, r=j4_high)   # Go up
  

#=====================================================#
#  Main Code

for line in range (4):
  Current_X= X_1 + DIST_CONS*line                               # Adds the line shift
  print("Current X: ", Current_X)

  for col in range (4):
    Current_Y= Y_1 + DIST_CONS*col                              # Adds the column shift
    print("Current y: ", Current_Y)
    
    Jump_to(Current_X, Current_Y, Z_1, R_1, True)               # Catach
    m_lite.wait(delay=0.5)                                      # Wait

    Jump_to(Current_X, Current_Y+DIST_F_D, Z_1, R_1,  False)    # Releasee
    m_lite.wait(delay=0.5)                                      # Wait

m_lite.set_homecmd()
# ==================================
