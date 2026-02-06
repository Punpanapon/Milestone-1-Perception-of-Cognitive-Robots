from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# --- NEW: Enable the Camera ---
camera = robot.getDevice('camera')
camera.enable(timestep) 
# ------------------------------

# Motor Setup
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# Configuration
SPEED = 3.0
SIDE_DURATION = 100  
TURN_DURATION = 50   

# Variables
counter = 0
state = 'FORWARD'

while robot.step(timestep) != -1:
    # State 1: Drive Forward
    if state == 'FORWARD':
        left_motor.setVelocity(SPEED)
        right_motor.setVelocity(SPEED)
        
        if counter >= SIDE_DURATION:
            counter = 0
            state = 'TURN'
            
    # State 2: Turn 90 Degrees
    elif state == 'TURN':
        left_motor.setVelocity(-SPEED)
        right_motor.setVelocity(SPEED)
        
        if counter >= TURN_DURATION:
            counter = 0
            state = 'FORWARD'

    counter += 1