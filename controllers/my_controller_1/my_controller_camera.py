from controller import Robot,Motor,Device,DistanceSensor,Camera
robot = Robot() 
Time_step = 64
Max_speed = 6.28   
ps=[]
psNames=['ps0','ps1','ps2','ps3','ps4','ps5','ps6','ps7']
gsNames=['gs0','gs1','gs2']
gs=[]

for i in range(8):
    ps.append(robot.getDistanceSensor(psNames[i]))
    ps[i].enable(Time_step)
for i in range(3):
    gs.append(robot.getDistanceSensor(gsNames[i]))
    gs[i].enable(Time_step) 

camera = robot.getCamera('camera')
camera.enable(Time_step)    
leftMotor = robot.getMotor('left wheel motor')
rightMotor = robot.getMotor('right wheel motor')
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftspeed = 0.5*Max_speed
rightspeed = 0.5*Max_speed

def LineFollowingModule(void):
  DeltaS = 0
  DeltaS = gsValues[0] - gsValues[2]
  if gsValues[0]<500 and gsValues[2]>500:
      leftspeed = 0
      rightspeed = Max_speed
  elif gsValues[0]>500 and gsValues[2]<500:
      leftspeed = Max_speed
      rightspeed = 0
  else:
      leftspeed = 0.5*Max_speed
      rightspeed = 0.5*Max_speed
  return leftspeed, rightspeed

while robot.step(Time_step) != -1:
    psValues = []
    gsValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())
    for i in range(3):
        gsValues.append(gs[i].getValue())
        
    if gsValues[0]<500 and gsValues[2]>500:
       leftspeed = 0
       rightspeed = Max_speed
    elif gsValues[0]>500 and gsValues[2]<500:
       leftspeed = Max_speed
       rightspeed = 0
    else:
       leftspeed = 0.5*Max_speed
       rightspeed = 0.5*Max_speed
    leftMotor.setVelocity(leftspeed)
    rightMotor.setVelocity(rightspeed)
    print(gsValues[0],gsValues[1],gsValues[2],leftspeed,rightspeed)
pass

# Enter here exit cleanup code.
