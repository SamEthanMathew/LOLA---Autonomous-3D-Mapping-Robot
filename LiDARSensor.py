import threading
import json
from rplidar import RPLidar, RPLidarException
import time
import serial.tools.list_ports
import os
import pygame
import math
from collections import deque
import numpy as np

# --- SLAM Implementation ---
def best_fit_transform(A, B):
    '''
    Calculates the least-squares best-fit transform that maps corresponding points A to B in m spatial dimensions
    Input:
      A: Nxm numpy array of corresponding points
      B: Nxm numpy array of corresponding points
    Returns:
      T: (m+1)x(m+1) homogeneous transformation matrix
      R: mxm rotation matrix
      t: mx1 translation vector
    '''

    assert A.shape == B.shape

    # get number of dimensions
    m = A.shape[1]

    # translate points to their centroids
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    AA = A - centroid_A
    BB = B - centroid_B

    # rotation matrix
    H = np.dot(AA.T, BB)
    U, S, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T, U.T)

    # special reflection case
    if np.linalg.det(R) < 0:
       Vt[m-1,:] *= -1
       R = np.dot(Vt.T, U.T)

    # translation
    t = centroid_B.T - np.dot(R, centroid_A.T)

    # homogeneous transformation
    T = np.identity(m+1)
    T[:m, :m] = R
    T[:m, m] = t

    return T, R, t

def nearest_neighbor(src, dst):
    '''
    Find the nearest (Euclidean) neighbor in dst for each point in src
    Input:
        src: Nxm array of points
        dst: Nxm array of points
    Output:
        distances: Euclidean distances of the nearest neighbor
        indices: dst indices of the nearest neighbor
    '''
    # Brute force NN (OK for small N < 500)
    # For speedup could use scipy.spatial.KDTree, but let's stick to numpy broadcasting
    # Distance matrix: dists[i, j] = ||src[i] - dst[j]||
    # This might be memory heavy for 1000x1000.
    # Let's simple loop or use simplified broadcasting if N is small.
    
    # Efficient broadcasting approach:
    # (x-x')^2 + (y-y')^2 = x^2 + x'^2 - 2xx' + ...
    # Let's iterate if N is large to save memory
    
    indices = np.zeros(src.shape[0], dtype=np.int32)
    distances = np.zeros(src.shape[0])
    
    for i, s in enumerate(src):
        d = np.linalg.norm(dst - s, axis=1)
        indices[i] = np.argmin(d)
        distances[i] = d[indices[i]]
        
    return distances, indices

def icp(A, B, init_pose=(0,0,0), max_iterations=10, tolerance=0.001):
    '''
    The Iterative Closest Point method: aligns source A to target B
    '''
    # Initial transform from init_pose (dx, dy, dtheta)
    src = np.copy(A)
    dst = np.copy(B)
    
    # Apply initial estimate to src
    # Rotation
    theta = init_pose[2]
    c, s = math.cos(theta), math.sin(theta)
    R_init = np.array([[c, -s], [s, c]])
    src = np.dot(src, R_init.T)
    src += init_pose[:2]
    
    prev_error = 0
    total_T = np.identity(3) # 3x3 for 2D points (x, y, 1)
    
    # Handle initial offset in matrix form
    T_init = np.identity(3)
    T_init[:2, :2] = R_init
    T_init[:2, 2] = init_pose[:2]
    total_T = np.dot(T_init, total_T)

    for i in range(max_iterations):
        # find the nearest neighbors between the current source and destination points
        distances, indices = nearest_neighbor(src, dst)

        # compute the transformation between the current source and nearest destination points
        T, _, _ = best_fit_transform(src, dst[indices])

        # update the current source
        # src is Nx2. T is 3x3. Homogenous coords needed.
        src_h = np.ones((src.shape[0], 3))
        src_h[:,:2] = src
        
        # transform
        src_h = np.dot(T, src_h.T).T
        src = src_h[:,:2]

        # update the total transformation
        total_T = np.dot(T, total_T)

        # check error
        mean_error = np.mean(distances)
        if np.abs(prev_error - mean_error) < tolerance:
            break
        prev_error = mean_error

    return total_T, distances

class SimpleSLAM:
    def __init__(self):
        # PROBABILISTIC GRID MAP
        # Key: (int_x, int_y) tuple representing 30mm grid cells
        # Value: [sum_x, sum_y, count]
        self.global_grid = {} 
        self.GRID_RESOLUTION = 30 # mm
        
        self.pose = np.array([0.0, 0.0, 0.0]) # x, y, theta
        self.path = [] # History of poses
        
        self.scan_buffer = [] # Buffer to accumulate scans before SLAM update
        self.SCAN_BUFFER_SIZE = 4 # How many scans to merge (robustness)
        self.prev_dense_scan = None
        
    def process_scan(self, scan_points):
        """
        Accumulates scans to create a dense 'Keyframe'.
        Runs ICP only when buffer is full.
        """
        # 1. Convert to Cartesian (Robot Frame)
        current_scan = []
        for p in scan_points:
            if p.distance > 0:
                rad = math.radians(p.angle)
                x = p.distance * math.cos(rad)
                y = p.distance * math.sin(rad)
                current_scan.append([x, y])
        
        # Add to buffer
        if len(current_scan) > 10:
             self.scan_buffer.extend(current_scan)
             
        # Only process if buffer is full (Signal Integration)
        # We assume robot moves slowly enough that 4 scans (~0.5s) is "static" enough
        # or that the blur is acceptable for specific density.
        current_rev = scan_points[0].revolution if scan_points else 0
        
        # Check if we have enough data (using a simple counter or checking length)
        # Since process_scan is called once per revolution roughly
        # We need a counter in the class, or just check list length logic
        # But process_scan is called per frame? No, per revolution based on main loop.
        # So we just count calls?
        # Let's use the length of the buffer as a proxy? No, scans vary.
        # Let's add a counter.
        
        if not hasattr(self, 'buffer_count'):
             self.buffer_count = 0
        self.buffer_count += 1
        
        if self.buffer_count < self.SCAN_BUFFER_SIZE:
             return
             
        # --- PROCESS KEYFRAME ---
        self.buffer_count = 0
        if not self.scan_buffer:
             return
             
        dense_scan = np.array(self.scan_buffer)
        self.scan_buffer = [] # Clear for next batch
        
        # DOWNSAMPLE if too huge (ICP speed)
        # 4 scans * 200 pts = 800 pts. limit to 400 for speed?
        if len(dense_scan) > 400:
             indices = np.linspace(0, len(dense_scan)-1, 400, dtype=int)
             dense_scan_small = dense_scan[indices]
        else:
             dense_scan_small = dense_scan

        # 2. Estimate Motion (ICP)
        d_pose = np.identity(3)
        moved = False
        
        if self.prev_dense_scan is not None:
             try:
                 # Align current dense scan to previous dense scan
                 T, distances = icp(dense_scan_small, self.prev_dense_scan)
                 d_pose = T
                 
                 # Check Motion Thresholds (Keyframe Logic)
                 # prevents drift when static
                 dx = d_pose[0, 2]
                 dy = d_pose[1, 2]
                 dtheta = math.atan2(d_pose[1, 0], d_pose[0, 0])
                 
                 dist_moved = math.sqrt(dx*dx + dy*dy)
                 rot_moved = abs(math.degrees(dtheta))
                 
                 if dist_moved > 20 or rot_moved > 2.0: # 20mm or 2 degrees
                      moved = True
                 else:
                      # If motion is tiny, ignore it to prevent drift?
                      # Or accumulate it? safer to ignore for "rock solid" static map
                      d_pose = np.identity(3) # Force zero
                      # But we might still want to add points to map if new area seen?
                      # No, if we didn't move, we see same thing.
                      pass

             except Exception as e:
                 print(f"ICP Error: {e}")
                 pass
        else:
             moved = True # First frame always updates
        
        # Extract refined delta
        dx = d_pose[0, 2]
        dy = d_pose[1, 2]
        dtheta = math.atan2(d_pose[1, 0], d_pose[0, 0])
        
        # 3. Update Global Pose
        gx = dx * math.cos(self.pose[2]) - dy * math.sin(self.pose[2])
        gy = dx * math.sin(self.pose[2]) + dy * math.cos(self.pose[2])
        
        self.pose[0] += gx
        self.pose[1] += gy
        self.pose[2] += dtheta
        
        # 4. Update Map & Reference
        # If we moved (or it's the first frame), update the map
        if moved or self.prev_dense_scan is None:
             c, s = math.cos(self.pose[2]), math.sin(self.pose[2])
             R_global = np.array([[c, -s], [s, c]])
             
             # Transform dense scan to global
             global_points = np.dot(dense_scan, R_global.T) + self.pose[:2]
             
             # PROBABILISTIC UPDATE
             for p in global_points:
                 gx, gy = p[0], p[1]
                 # Quantize to grid key
                 ix = int(round(gx / self.GRID_RESOLUTION))
                 iy = int(round(gy / self.GRID_RESOLUTION))
                 key = (ix, iy)
                 
                 if key in self.global_grid:
                     # Fuse with existing (Running Average)
                     cell = self.global_grid[key]
                     # Soft cap on count to allow slow adaptation if environment changes
                     # but high enough to converge noise (e.g. 50)
                     if cell[2] < 50:
                         cell[0] += gx
                         cell[1] += gy
                         cell[2] += 1
                     else:
                         # Rolling average for adaptation?
                         # For now just keep it stable
                         pass
                 else:
                     # New cell
                     self.global_grid[key] = [gx, gy, 1]
                          
             self.prev_dense_scan = dense_scan_small # Update reference
             self.path.append(self.pose.copy())
             
    def get_map_points(self):
        """Reconstruct list of points [x,y] from the grid (Mean values)"""
        points = []
        for key, val in self.global_grid.items():
            # if val[2] > 1: # Optional filter: only confirmed points?
            points.append([val[0]/val[2], val[1]/val[2]])
        return points

    def get_pose(self):
        """Returns (x, y, theta_degrees)"""
        return self.pose[0], self.pose[1], math.degrees(self.pose[2])

def normalize_angle(angle_deg):
    """Normalize angle to [-180, 180]"""
    while angle_deg > 180:
        angle_deg -= 360
    while angle_deg <= -180:
        angle_deg += 360
    return angle_deg



slam_system = SimpleSLAM()

# 46 degrees
# 133 degrees

class LidarSensor:
    def __init__(self, port=None):
        if port is None:
            port = self._detect_serial_port()
        
        # Note: self.lidar is initialized inside _detect_serial_port if detection succeeds,
        # otherwise we might need to initialize it here if specific port passed.
        if not hasattr(self, 'lidar'):
             print(f"Attempting to connect to specific/fallback port: {port}")
             self.lidar = RPLidar(port, timeout=5)
        
        # Robust initialization
        try:
            self.lidar.clear_input()
            time.sleep(0.5)
            print(self.lidar.get_info())
            print(self.lidar.get_health())
        except Exception as e:
            print(f"Initial connection error: {e}. Retrying reset...")
            try:
                self.lidar.stop()
                self.lidar.stop_motor()
                self.lidar.disconnect()
                time.sleep(1)
                self.lidar.connect()
                self.lidar.clear_input()
                print(self.lidar.get_info())
                print(self.lidar.get_health())
            except Exception as e2:
                 print(f"Retry failed: {e2}. Continuing anyway, scanning might fail.")
        self.scan_data = []               # Cumulative list of all scan points
        self.last_revolution_data = []    # Points from the last complete revolution
        self.port = None # Store the connected port
        self.current_revolution_points = []  # Points for the current revolution in progress
        self.running = True
        self.revolutions = 0
        self.last_angle = None            # To detect wrap-around in angle
        # Start the LiDAR scanning in its own thread
        self.thread = threading.Thread(target=self._scan_loop, daemon=True)
        self.thread.start()

    def _detect_serial_port(self):
        """
        Detects the serial port by trying to connect and read info from valid candidates.
        """
        ports = list(serial.tools.list_ports.comports())
        candidates = []
        
        # Filter for likely candidates on macOS/Linux
        for p in ports:
            if any(x in p.device for x in ["SLAB_USBtoUART", "usbserial", "USB", "ACM"]):
                candidates.append(p.device)
        
        # Prioritize SLAB_USBtoUART as it was confirmed working
        candidates.sort(key=lambda x: 0 if "SLAB_USBtoUART" in x else 1)
                
        print(f"Candidate ports: {candidates}")
        
        for port in candidates:
            print(f"Testing port: {port}...")
            for baud in [115200, 256000]:
                print(f"  Trying baudrate: {baud}...")
                try:
                    # Try to connect and get info (reduced timeout for faster checks)
                    temp_lidar = RPLidar(port, baudrate=baud, timeout=2)
                    # Force stop in case it's already spinning
                    try:
                        temp_lidar.stop()
                        temp_lidar.stop_motor()
                    except:
                        pass
                    temp_lidar.clear_input()
                    info = temp_lidar.get_info()
                    print(f"Success! Found LiDAR on {port} at {baud} baud: {info}")
                    temp_lidar.disconnect()
                    # Re-initialize the main lidar object with the correct baudrate
                    self.lidar = RPLidar(port, baudrate=baud, timeout=5)
                    self.port = port
                    return port
                except Exception as e:
                    print(f"  Failed on {port} @ {baud}: {e}")
                    if "Resource busy" in str(e) or "Errno 16" in str(e):
                        print(f"  --> PORT LOCKED: {port} is being used by another process or is stuck.")
                        print("      ACTION REQUIRED: Unplug and replug the LiDAR USB cable to reset the connection.")
                    try:
                        temp_lidar.disconnect()
                    except:
                        pass
                    continue
                
        print("Could not find a working LiDAR on any port. Defaulting to /dev/ttyUSB0")
        return '/dev/ttyUSB0'
    

    def _scan_loop(self):
        """
        Continuously fetch scan data in a separate thread.
        """
        print("scan looping", flush=True)
        # Ensure clean state before starting scans
        try:
             self.lidar.stop()
             self.lidar.stop_motor()
             time.sleep(1)
             self.lidar.start_motor()
             time.sleep(0.5)
        except:
             pass
             
        while self.running:
            try:
                # Track stats
                total_points_in_rev = 0
                valid_points_in_rev = 0
                rev_start_time = time.time()
                
                # Use iter_measurments for raw data access (avoid buffering/grouping latency)
                for new_scan, quality, angle, distance in self.lidar.iter_measurments():
                    if not self.running:
                        break
                    
                    total_points_in_rev += 1
                    
                    # 'new_scan' is a boolean flag indicating start of a new revolution
                    if new_scan:
                        # Revolution complete
                        dt = time.time() - rev_start_time
                        rpm = 60/dt if dt > 0 else 0
                        rev_start_time = time.time()
                        
                        count = valid_points_in_rev
                        print(f"Rev {self.revolutions} | RPM: {rpm:.1f} | Points: {count}/{total_points_in_rev} ({(count/total_points_in_rev*100) if total_points_in_rev else 0:.1f}%)", flush=True)
                            
                        self.last_revolution_data = self.current_revolution_points.copy()
                        
                        self.current_revolution_points = []
                        valid_points_in_rev = 0
                        total_points_in_rev = 0
                        self.revolutions += 1
                    
                    if distance > 0:
                        valid_points_in_rev += 1
                        # Create ScanObject (adapted for new format)
                        # We use a dummy list [quality, angle, distance] to match ScanObject init
                        scan_obj = ScanObject([quality, angle, distance], revolution=self.revolutions)
                        self.scan_data.append(scan_obj)
                        self.current_revolution_points.append(scan_obj)
            except RPLidarException as e:
                print(f"LiDAR connection/scan error: {e}. Retrying...", flush=True)
                try:
                    self.lidar.stop()
                    self.lidar.disconnect()
                    self.lidar.connect()
                    self.lidar.clear_input()
                except Exception as recon_e:
                    print(f"Failed to reconnect: {recon_e}")
                    time.sleep(2)
            except Exception as e:
                print(f"Unexpected error in scan loop: {e}", flush=True)
                time.sleep(1)
    
    def get_scan(self):
        """Return the full list of scan objects."""
        return self.scan_data

    def getLastRevolutionData(self):
        return self.last_revolution_data
    
    def waitForNewRevolution(self):
        currentRevolution = self.revolutions
        
        while True:
            time.sleep(0.01)
            if currentRevolution != self.revolutions:
                return
        
    
    def stop(self):
        """Stop scanning and clean up the LiDAR sensor."""
        self.running = False
        try:
            self.thread.join()
        except:
            pass
        self.lidar.stop()
        self.lidar.stop_motor()
        self.lidar.disconnect()

class ScanObject:
    def __init__(self, scanBase, revolution=None):
        self.scanStrength = scanBase[0]
        self.angle = scanBase[1]
        self.distance = scanBase[2] # Keep in mm
        self.revolution = revolution

def calculate_exploration_vector(cartesian_points):
    """
    Returns the angle (degrees) of the 'deepest' sector (most open space).
    Input: numpy array or list of [x, y] points (in robot local frame)
    """
    if len(cartesian_points) == 0:
        return None
        
    # Analyze in 10-degree sectors
    SECTOR_SIZE = 10
    num_sectors = 360 // SECTOR_SIZE
    sector_sums = [0.0] * num_sectors
    sector_counts = [0] * num_sectors
    
    # Convert Cart to Polar
    # We can do this efficiently
    # But loop is fine for <3000 points
    for p in cartesian_points:
        x, y = p[0], p[1]
        dist = math.sqrt(x*x + y*y)
        if dist > 0:
            angle_rad = math.atan2(y, x)
            angle_deg = math.degrees(angle_rad) % 360
            
            idx = int(angle_deg // SECTOR_SIZE)
            
            if 0 <= idx < num_sectors:
                sector_sums[idx] += dist
                sector_counts[idx] += 1
                
    best_idx = -1
    max_avg = -1.0
    
    for i in range(num_sectors):
        if sector_counts[i] > 5: # Threshold
            avg = sector_sums[i] / sector_counts[i]
            if avg > max_avg:
                max_avg = avg
                best_idx = i
                
    if best_idx != -1:
        # Return the center angle of the best sector
        return best_idx * SECTOR_SIZE + (SECTOR_SIZE / 2)
    return None

        

if __name__ == "__main__":
    # Initialize LiDAR
    lidar = LidarSensor()
    time.sleep(2) # Allow time for spin up and connection
    
    # Try to slow down motor to increase point density (more samples per degree)
    # Default is often 1023 (max). 600-700 is a good target for A1/A2.
    try:
        lidar.lidar.set_pwm(200)
        print("Set PWM to 660 for denser scanning")
    except Exception as e:
        print(f"Could not set PWM (might not be supported): {e}")
    
    
    # Initialize Pygame
    pygame.init()
    WIDTH, HEIGHT = 1000, 800
    screen = pygame.display.set_mode((WIDTH, HEIGHT), pygame.RESIZABLE)
    pygame.display.set_caption("LiDAR Radar View")
    
    # Fonts
    font_hud = pygame.font.SysFont("monospace", 16)
    font_label = pygame.font.SysFont("monospace", 12)

    # Colors (Sci-Fi Theme)
    COLOR_BG = (10, 15, 20)      # Dark Blue-Black
    COLOR_GRID = (40, 60, 80)    # Faint Blue-Grey
    COLOR_AXIS = (60, 90, 120)   # Brighter Axis
    COLOR_POINT_NEW = (0, 255, 255) # Cyan (Newest)
    COLOR_POINT_OLD = (0, 100, 100) # Dark Cyan (Oldest)
    COLOR_ROBOT = (255, 50, 50)  # Red
    COLOR_TEXT = (200, 255, 255) # White-Cyan

    # Visualization settings
    scale = 0.15  # Pixels per mm (0.15 = 150px per meter. Fits ~5m room on screen)
    offset_x = WIDTH // 2
    offset_y = HEIGHT // 2
    view_offset_x = 0
    view_offset_y = 0
    is_dragging = False
    last_mouse_pos = (0, 0)
    
    clock = pygame.time.Clock()
    running = True

    
    # Visualization buffer
    # We use point_buffer for raw "trails" (local frame)
    point_buffer = deque(maxlen=2000) 
    last_processed_revolution = -1
    
    # SLAM Mode Flag
    SHOW_SLAM_MAP = True
    
    # Stats for HUD
    current_rpm = 0.0
    current_valid_pct = 0.0
    
    # Exploration Vector Smoothing (EMA)
    exp_smooth_x = 0.0
    exp_smooth_y = 0.0

    print("Radar UI started. Drag to Pan, Scroll to Zoom.")

    try:
        while running:
            # Event handling
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.VIDEORESIZE:
                    WIDTH, HEIGHT = event.w, event.h
                    offset_x = WIDTH // 2
                    offset_y = HEIGHT // 2
                elif event.type == pygame.MOUSEWHEEL:
                    # Zoom in/out
                    zoom_factor = 1.1 if event.y > 0 else 0.9
                    scale *= zoom_factor
                    scale = max(0.01, min(scale, 10.0)) # Relaxed clamp to allow infinite zoom
                
                # Pan logic
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    if event.button == 1: # Left click
                        is_dragging = True
                        last_mouse_pos = event.pos
                elif event.type == pygame.MOUSEBUTTONUP:
                    if event.button == 1:
                        is_dragging = False
                elif event.type == pygame.MOUSEMOTION:
                    if is_dragging:
                        dx, dy = event.pos[0] - last_mouse_pos[0], event.pos[1] - last_mouse_pos[1]
                        view_offset_x += dx
                        view_offset_y += dy
                        last_mouse_pos = event.pos
            
            # --- Rendering ---
            screen.fill(COLOR_BG)
            
            # center_x/y is now the WORLD ORIGIN (0,0) on screen
            map_origin_x = offset_x + view_offset_x
            map_origin_y = offset_y + view_offset_y

            # 1. Draw Grid (Centered on World Origin)
            # Calculate grid step size
            steps = [500, 1000, 2000, 5000, 10000] # 0.5m, 1m, 2m, 5m, 10m
            
            # Find closest nice step
            visible_radius_mm = min(WIDTH, HEIGHT) / 2 / scale
            grid_step_mm = 1000
            for s in steps:
                if visible_radius_mm / s >= 2.5: 
                   grid_step_mm = s
            
            # Draw circles centered on World Origin
            current_r_mm = grid_step_mm
            while True:
                r_px = int(current_r_mm * scale)
                if r_px > max(WIDTH, HEIGHT) * 2: # Stop if far off screen
                    break
                    
                # Only draw if roughly visible? Pygame handles clipping.
                pygame.draw.circle(screen, COLOR_GRID, (int(map_origin_x), int(map_origin_y)), r_px, 1)
                
                # Label along the axis
                dist_m = current_r_mm / 1000
                label_text = f"{dist_m:.1f}m" if dist_m % 1 != 0 else f"{int(dist_m)}m"
                label = font_label.render(label_text, True, COLOR_GRID)
                screen.blit(label, (map_origin_x + 5, map_origin_y - r_px - 15))
                
                current_r_mm += grid_step_mm
            
            # Draw Axes (World Frame)
            pygame.draw.line(screen, COLOR_AXIS, (map_origin_x - 10000, map_origin_y), (map_origin_x + 10000, map_origin_y), 1)
            pygame.draw.line(screen, COLOR_AXIS, (map_origin_x, map_origin_y - 10000), (map_origin_x, map_origin_y + 10000), 1)

            # 2. Process Data
            data_points = lidar.getLastRevolutionData()
            
            if data_points and data_points[0].revolution > last_processed_revolution:
                last_processed_revolution = data_points[0].revolution
                point_buffer.extend(data_points)
                # Update SLAM
                slam_system.process_scan(data_points)
            
            # Get Robot Pose
            rob_x, rob_y, rob_theta = slam_system.pose[0], slam_system.pose[1], slam_system.pose[2]
            
            # Robot Screen Position
            rob_sx = map_origin_x + rob_x * scale
            rob_sy = map_origin_y + rob_y * scale

            # 3. Draw Points with Fading (Raw Buffer)
            # These are relative to the robot. In Absolute view, we must transform them.
            # We assume they belong to the *current* pose for visualization (approx).
            if point_buffer:
                num_points = len(point_buffer)
                for i, point in enumerate(point_buffer):
                    dist = point.distance
                    if dist > 0:
                        # Local Angle
                        angle_rad_local = math.radians(point.angle)
                        # Global Angle = Robot Theta + Local Angle
                        angle_rad_global = rob_theta + angle_rad_local
                        
                        # Global Pos = Robot Pos + Rotated Vector
                        gx = rob_x + dist * math.cos(angle_rad_global)
                        gy = rob_y + dist * math.sin(angle_rad_global)
                        
                        # Screen Pos
                        sx = map_origin_x + gx * scale
                        sy = map_origin_y + gy * scale
                        
                        if -10 <= sx <= WIDTH+10 and -10 <= sy <= HEIGHT+10:
                            intensity = int(50 + (205 * (i / num_points))) 
                            color = (0, intensity, intensity) 
                            pygame.draw.circle(screen, color, (int(sx), int(sy)), 2)

            # 3.5. Draw SLAM Global Map (Grid)
            # Points are already in Global Frame
            if SHOW_SLAM_MAP:
                for key, val in slam_system.global_grid.items():
                    if val[2] < 1: continue 
                    mx = val[0] / val[2]
                    my = val[1] / val[2]
                    
                    sx = map_origin_x + mx * scale
                    sy = map_origin_y + my * scale
                    
                    if 0 <= sx <= WIDTH and 0 <= sy <= HEIGHT:
                         cness = min(255, 50 + val[2]*10)
                         pygame.draw.circle(screen, (cness, cness, cness), (int(sx), int(sy)), 1)
                         
                # Draw Path
                if len(slam_system.path) > 1:
                    path_points = []
                    for pose in slam_system.path:
                        px = map_origin_x + pose[0] * scale
                        py = map_origin_y + pose[1] * scale
                        path_points.append((px, py))
                    
                    if len(path_points) > 1:
                        pygame.draw.lines(screen, (0, 255, 0), False, path_points, 2)


            # 4. Draw Robot Marker (At calculated screen pos)
            pygame.draw.circle(screen, COLOR_ROBOT, (int(rob_sx), int(rob_sy)), 6)
            # Direction indicator
            arrow_len = 20
            ax = rob_sx + arrow_len * math.cos(rob_theta)
            ay = rob_sy + arrow_len * math.sin(rob_theta)
            pygame.draw.line(screen, COLOR_ROBOT, (int(rob_sx), int(rob_sy)), (int(ax), int(ay)), 2)

            # 5. Draw Exploration Vector (Yellow Line)
            # Use SLAM MAP for stability
            map_pts = slam_system.get_map_points()
            if len(map_pts) > 10:
                recent_map = np.array(map_pts)
                
                # Transform Global -> Local (for calculation)
                c, s = math.cos(rob_theta), math.sin(rob_theta)
                R_inv = np.array([[c, s], [-s, c]])
                
                pts_centered = recent_map - np.array([rob_x, rob_y])
                local_map_pts = np.dot(pts_centered, R_inv.T)
                
                target_angle_local = calculate_exploration_vector(local_map_pts)
                
                # NAVIGATION COMMAND
                turn_cmd_deg = 0.0

                if target_angle_local is not None:
                    # Smoothing (Local Frame)
                    t_rad_local = math.radians(target_angle_local)
                    tx, ty = math.cos(t_rad_local), math.sin(t_rad_local)
                    
                    alpha = 0.1 
                    if exp_smooth_x == 0 and exp_smooth_y == 0:
                        exp_smooth_x, exp_smooth_y = tx, ty
                    else:
                        exp_smooth_x = (1 - alpha) * exp_smooth_x + alpha * tx
                        exp_smooth_y = (1 - alpha) * exp_smooth_y + alpha * ty
                    
                    # Convert Smoothed Vector to Global Angle for drawing
                    local_angle_smooth_rad = math.atan2(exp_smooth_y, exp_smooth_x)
                    local_angle_smooth_deg = math.degrees(local_angle_smooth_rad)
                    
                    # Calculate Turn Command (Normalize to -180 to 180)
                    # Exploration Vector is already in Robot Frame: 0 deg = Forward (X)
                    # We need to turn to face it.
                    turn_cmd_deg = normalize_angle(local_angle_smooth_deg)
                    
                    global_draw_angle = rob_theta + local_angle_smooth_rad
                    
                    # Draw Line from Robot
                    vec_len = 150 
                    ex = rob_sx + vec_len * math.cos(global_draw_angle)
                    ey = rob_sy + vec_len * math.sin(global_draw_angle)
                    
                    pygame.draw.line(screen, (255, 255, 0), (rob_sx, rob_sy), (ex, ey), 4)
                    
                    label_target = font_label.render(f"GOAL ({turn_cmd_deg:.1f}°)", True, (255, 255, 0))
                    screen.blit(label_target, (ex + 10, ey))
                


            # 6. Draw HUD
            hud_y = 10
            line_height = 20
            
            # Format Turn Command
            turn_str = "N/A"
            if 'turn_cmd_deg' in locals() and target_angle_local is not None:
                d = turn_cmd_deg
                if abs(d) < 5:
                    turn_str = "FORWARD"
                elif d > 0:
                    turn_str = f"LEFT {abs(d):.1f}°" # Standard: +Angle is Left (CCW)
                else: 
                    turn_str = f"RIGHT {abs(d):.1f}°"
            
            texts = [
                f"FPS: {clock.get_fps():.1f}",
                f"Buffer: {len(point_buffer)} pts",
                f"Scale: {scale:.2f} px/mm",
                f"Status: Connected ({lidar.port})",
                f"CMD: {turn_str}",
                "Controls: Drag=Pan, Scroll=Zoom"
            ]
            
            for text_str in texts:
                s = font_hud.render(text_str, True, COLOR_TEXT)
                screen.blit(s, (10, hud_y))
                hud_y += line_height

            # Update display
            pygame.display.flip()
            clock.tick(30)            
            
    except KeyboardInterrupt:
        pass
    finally:
        print("Stopping LiDAR...")
        lidar.stop()
        pygame.quit()

