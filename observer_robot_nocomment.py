from controller import Robot, Keyboard
#--- CONFIGURATION & CONSTANTS ---
MAX_SPEED = 6.28
CRUISE_SPEED = 3.0

# --- Vision Settings ---
MOTION_SENSITIVITY = 40    
EDGE_SENSITIVITY = 30      
RED_THRESHOLD = 150        

#Classification Rules  
EDGE_LIMIT_WALL = 1        
EDGE_LIMIT_BOX = 8         

#Edge Cooldown 
# This forces the robot to skip over the "ABCD" text entirely.
EDGE_COOLDOWN_PIXELS = 30  

# --- Sensor Thresholds ---
OBSTACLE_DIST_THRESHOLD = 60

# --- Robot States ---
STATE_WANDER = 0
STATE_INSPECT = 1
STATE_AVOID = 2
STATE_STAY = 3

# --- Key Settings ---
PRESS_DURATION = 30 
# --- INITIALIZATION ---

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# 1. Camera
camera = robot.getDevice('camera')
camera.enable(timestep)
cam_width = camera.getWidth()
cam_height = camera.getHeight()

# 2. Sensors
ps = []
ps_names = ['ps0', 'ps7', 'ps1', 'ps6'] 
for name in ps_names:
    sensor = robot.getDevice(name)
    sensor.enable(timestep)
    ps.append(sensor)

# 3. Motors
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# 4. Keyboard
keyboard = Keyboard()
keyboard.enable(timestep)

# Globals
current_state = STATE_WANDER
state_timer = 0
previous_image = None
manual_mode = False
s_key_timer = 0

print(f">>> SYSTEM ONLINE. Resolution: {cam_width}x{cam_height}")
print(">>> PIPELINE: 3x3 Gaussian Blur -> Gradient -> Spatial Filter")


#HELPER FUNCTIONS


def get_sensor_max():
    return max(ps[0].getValue(), ps[1].getValue())

def raw_to_cm(raw_value):
    if raw_value < 10: return 99.9
    dist_cm = 0.5 * (1000 / (raw_value + 1)) 
    if dist_cm > 20: dist_cm = 20
    return round(dist_cm, 1)

def set_drive(left, right):
    left_motor.setVelocity(left)
    right_motor.setVelocity(right)

def get_gaussian_pixel(img, x, y, width):
    """
    [PIPELINE STEP 1]: 3x3 GAUSSIAN BLUR
    Applies the standard kernel from Slide 2.
    [1 2 1]
    [2 4 2] / 16
    [1 2 1]
    """
    # Safety Check
    if x < 1 or x >= width-1 or y < 1 or y >= 240-1:
        i = (y * width + x) * 4
        return img[i], img[i+1], img[i+2] # Return raw if border

    # Center pixel index
    i = (y * width + x) * 4
    
    # Neighbor indices
    i_up = ((y-1) * width + x) * 4
    i_down = ((y+1) * width + x) * 4
    i_left = (y * width + (x-1)) * 4
    i_right = (y * width + (x+1)) * 4
    
    # Apply Kernel (Approximation for speed)
    b = (4*img[i] + img[i_up] + img[i_down] + img[i_left] + img[i_right]) // 8
    g = (4*img[i+1] + img[i_up+1] + img[i_down+1] + img[i_left+1] + img[i_right+1]) // 8
    r = (4*img[i+2] + img[i_up+2] + img[i_down+2] + img[i_left+2] + img[i_right+2]) // 8
    
    return b, g, r

def run_vision_pipeline(img_current, img_prev, width, height):
    """
    Executes the Standard Lecture Pipeline with Spatial Filtering
    """
    if img_current is None or img_prev is None:
        return "WAITING", False, 0, 0

    vertical_edges_found = 0
    is_goal = False
    
    moving_pixels = 0
    total_blob_pixels = 0
    sum_x = 0
    min_x = width
    max_x = 0
    

    step = 2
    
    
    # PASS 1: MOTION & COLOR (Fast Scan)
    
    for y in range(0, height, step):
        for x in range(0, width, step):
            i = (y * width + x) * 4
            
            # Simple Grayscale for Motion
            b, g, r = img_current[i], img_current[i+1], img_current[i+2]
            gray = (r + g + b) // 3
            
            prev_gray = (img_prev[i] + img_prev[i+1] + img_prev[i+2]) // 3
            if abs(gray - prev_gray) > MOTION_SENSITIVITY:
                moving_pixels += 1
            
            # Goal Check
            if r > RED_THRESHOLD and g < 80 and b < 80:
                is_goal = True
            
            if moving_pixels > 0 or is_goal:
                sum_x += x
                total_blob_pixels += 1
                if x < min_x: min_x = x
                if x > max_x: max_x = x

    
    # PASS 2: FEATURE EXTRACTION (Blur -> Edge -> Count)

    scan_y = int(height / 2)
     
    # If we find an edge, we force this counter to 15, ignoring next pixels.
    cooldown_timer = 0
    
    # Loop through the middle row
    for x in range(2, width - 6):
        
        # If we are in cooldown, skip processing logic
        if cooldown_timer > 0:
            cooldown_timer -= 1
            continue
            
        # Get Gaussian Blurred Pixel (Current)
        b, g, r = get_gaussian_pixel(img_current, x, scan_y, width)
        gray = (r + g + b) // 3
        
        # Get Gaussian Blurred Pixel (Neighbor +4px)
        b2, g2, r2 = get_gaussian_pixel(img_current, x+4, scan_y, width)
        gray_next = (r2 + g2 + b2) // 3
        
        # Edge Gradient
        diff = abs(gray - gray_next)
        
        # Check Threshold
        if diff > EDGE_SENSITIVITY:
            # Filter Borders
            if x > 10 and x < (width - 10):
                vertical_edges_found += 1
                
                # TRIGGER COOLDOWN
                cooldown_timer = EDGE_COOLDOWN_PIXELS

    
    # PASS 3: CLASSIFICATION
    
    status_label = "UNKNOWN"
    
    if moving_pixels > 50:
        status_label = "DYNAMIC (Moving)"
    else:
        # 0-1 Edges = Wall
        if vertical_edges_found <= EDGE_LIMIT_WALL:
             status_label = "WALL (Background)"
             
        # 2-4 Edges = Box
        elif vertical_edges_found <= EDGE_LIMIT_BOX:
             status_label = "STATIC (Box)"
             
        # >4 Edges = Robot
        else:
             status_label = "STATIC (Complex Obj)"

    blob_center_x = sum_x // total_blob_pixels if total_blob_pixels > 0 else width // 2

    return status_label, is_goal, blob_center_x, vertical_edges_found

# MAIN CONTROL LOOP

while robot.step(timestep) != -1:
    
    raw_dist = get_sensor_max()
    dist_cm = raw_to_cm(raw_dist)
    current_image = camera.getImage()
    key = keyboard.getKey()
    
    # Mode Switches
    if key == ord('M'):
        manual_mode = not manual_mode
        print(f"Manual Mode: {manual_mode}")

    if key == ord('S'):
        s_key_timer += 1
        if s_key_timer == PRESS_DURATION:
            if current_state != STATE_STAY:
                print("\n>>> TURRET MODE ACTIVATED <<<")
                current_state = STATE_STAY
            else:
                print("\n>>> RESUMING PATROL <<<")
                current_state = STATE_WANDER
    else:
        s_key_timer = 0

    if manual_mode:
        if key == Keyboard.UP: set_drive(CRUISE_SPEED, CRUISE_SPEED)
        elif key == Keyboard.DOWN: set_drive(-CRUISE_SPEED, -CRUISE_SPEED)
        elif key == Keyboard.LEFT: set_drive(-CRUISE_SPEED, CRUISE_SPEED)
        elif key == Keyboard.RIGHT: set_drive(CRUISE_SPEED, -CRUISE_SPEED)
        else: set_drive(0, 0)
        
    else:
        # STATE: WANDER
        if current_state == STATE_WANDER:
            set_drive(CRUISE_SPEED, CRUISE_SPEED)
            if raw_dist > OBSTACLE_DIST_THRESHOLD:
                print(f"[!] Proximity Alert ({dist_cm} cm). Analyzing...")
                current_state = STATE_INSPECT
                state_timer = 0
                
        # STATE: INSPECT
        elif current_state == STATE_INSPECT:
            set_drive(0, 0)
            state_timer += 1
            
            if state_timer > 20: 
                label, is_goal, blob_x, edge_count = run_vision_pipeline(current_image, previous_image, cam_width, cam_height)
                
                # Close proximity override
                if "WALL" in label and dist_cm < 8.0:
                    label = "STATIC (Unknown Obstacle)"
                
                goal_tag = " + [GOAL]" if is_goal else ""
                
                if "WALL" in label:
                     print(f">> ANALYSIS: {label} (Edges: {edge_count}) | Ignoring...")
                else:
                     print(f">> ANALYSIS: {label}{goal_tag} | Edges: {edge_count}")
                
                current_state = STATE_AVOID
                state_timer = 0
                
        # STATE: AVOID
        elif current_state == STATE_AVOID:
            set_drive(CRUISE_SPEED, -CRUISE_SPEED)
            state_timer += 1
            if state_timer > 30: 
                current_state = STATE_WANDER
                state_timer = 0
                
        # STATE: STAY
        elif current_state == STATE_STAY:
            set_drive(0, 0)
            state_timer += 1
            if state_timer > 10: 
                label, is_goal, blob_x, edge_count = run_vision_pipeline(current_image, previous_image, cam_width, cam_height)
                goal_lbl = "[GOAL]" if is_goal else "[    ]"
                
                if "CLEAR" not in label:
                    print(f"| HUD | {label:22} | {dist_cm} cm | {goal_lbl} | Edges: {edge_count}")
                state_timer = 0

    previous_image = current_image