import pyzed.sl as sl
import cv2
import OpenGL.GL as gl
import time
import numpy as np
import pywavefront
import argparse
import math as m
import time
import heapq
import os
from map_modifier_HD import *
from serial import Serial

def euler_to_rotation(r,p,y,t):
    """
        INPUT:
            * r, p, y = [1x1] euler element
            * t = [3x1] translation matrix
        OUTPUT:
            * pose = [3x4] tra
    """
    rz = np.array([[m.cos(y),-m.sin(y),0],
                   [m.sin(y),m.cos(y),0],
                   [0,0,1]])
    ry = np.array([[m.cos(p),0,m.sin(p)],
                   [0,1,0],
                   [-m.sin(p),0,m.cos(p)]])
    rx = np.array([[1,0,0],
                   [0,m.cos(r),-m.sin(r)],
                   [0,m.sin(r),m.cos(r)]])
    return np.concatenate((rz.dot(ry).dot(rx),t.reshape((3,1))),axis=-1)

def transform_real_pose_to_grille_pose(pose, border_param, map_2D):
    """
        This important function will draw the pose of the robot in the 2D map.
        INPUT:
            * pose [1x3]
    """
    index_axes_1 = m.floor(((pose[0] + (border_param[0,0]**2)**0.5) * map_2D.shape[0]) / (border_param[0,1] + (border_param[0,0]**2)**0.5))
    index_axes_2 = m.floor(((pose[2] + (border_param[0,2]**2)**0.5) * map_2D.shape[1]) / (border_param[0,3] + (border_param[0,2]**2)**0.5))
    return index_axes_1, index_axes_2

def get_border(points_cloud):
    """
        DESCRIPTION: get all border from 3D points cloud.
        WARNING: be carefull to the axis orientation in this process.
        OUTPUT: 
            * border_param [1x4] = x_min, x_max, y_min, y_max
    """
    border_param = np.array([
        [points_cloud[:,0].min(), points_cloud[:,0].max(), points_cloud[:,2].min(), points_cloud[:,2].max(),]
    ])
    print("succes border param extraction.")
    return border_param

# NAVIGATION FUNCTIONfrom serial import Serial
def generate_key_point(path):
    """
        DESCRIPTION: this function will cute smartly the total path 
            and generate a intermediaire keypoint array.
    """
    size = path.shape[0]
    diviser = 1

    # first, found diviser value.
    i = 1
    if size > 40:
        false = True
        while false:
            if size / i < 40:
                false = False
                diviser = i
            else:
                i += 1

    # second, take i points in array + end point.
    key_points = np.zeros((diviser, 2)) 
    index = int(size / diviser)

    i = 1
    for point in key_points:
        point[:] = [path[index*i,0],path[index*i,1]]
        i +=1

    return key_points

def astar(array, start, goal):
   
    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
    close_set = set()
    came_from = {}
    gscore = {start:0}
    fscore = {start:heuristic(start, goal)}
    oheap = []
    heapq.heappush(oheap, (fscore[start], start))

    while oheap:
        current = heapq.heappop(oheap)[1]
        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            return data
        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j        

            tentative_g_score = gscore[current] + heuristic(current, neighbor)

            """if 0 <= neighbor[0] < array.shape[0]:
                if 0 <= neighbor[1] < array.shape[1]: """             
            if array[neighbor[0]][neighbor[1]] == 2:
                tentative_g_score += 10

            elif 0 <= neighbor[0] < array.shape[0]:
                if 0 <= neighbor[1] < array.shape[1]:              
                    if array[neighbor[0]][neighbor[1]] == 1:
                        continue
                else:
                    continue

            else:
                continue

            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
 
            if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)

                heapq.heappush(oheap, (fscore[neighbor], neighbor)) 

def heuristic(a, b, safe=None): 
    return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2) 

def navigation_processing(x_pos, y_pos, current_map, ELEVATOR):
    """
        DESCRIPTION: the function will return the map with path print on.
    """
    #print(ELEVATOR)
    current = (x_pos, y_pos)

    route = astar(current_map, current, ELEVATOR)
    if route is not None:
        route = route + [current]
        route = route[::-1]
        return route
    return None

def key_points_reach(x_pos, y_pos, key_point):
    """
        DESCRIPTION: output True if we are in the keypoint (next to him).
    """

    distance = ((x_pos - key_point[0])**2 + (y_pos - key_point[1])**2)**0.5
    if distance < 20:
        return True
    return False

# COMMAND MOTOR FUNCTION
def rad_to_norm_deg(radian):
    """
        DESCRIPTION: get radian non normalize rotation and set degre normalize.
    """
    return (radian%(2*m.pi))*(180/m.pi)

def calcul_vector(current_position, testouille):
    """
        DESCRIPTION:
        OUTPUT:
            * current_position = (x, y) index of current robot position.
            * testouille = numpy[20,2]
        INPUT:
            * angle_degree = (float) of vector from path
            * norm_vector = (float) of norm from vector
    """

    y_sum = (testouille[:,0]-current_position[0]).sum()
    x_sum = (testouille[:,1]-current_position[1]).sum()

    norm_vector = ((x_sum)**2 + (y_sum)**2)**0.5
    
    angle_degree = np.arccos((x_sum)/(x_sum**2+y_sum**2)**0.5)
    if y_sum < 0:
        angle_degree = (m.pi - angle_degree) + m.pi

    return angle_degree * (180/m.pi)

def command_main_function(current_angle, next_path, x_pos, y_pos):
    """
        DESCRIPTION: This function will take current pose
        INTPUT :
            * current_angle = [float] orientation in radian.
            * next_path = [np.array of 20 (x,y)] next path.
            * x_pos, y_pos = position in index.
        OUTPUT :
            * commande = [R,P]  
                * R = commande R(rotation) F(tout_droit)
                * P = puissance (neg left, pos right, neg back, pos front)
    """
    #vector_deg  = (360 - calcul_vector((x_pos, y_pos), list(map(tuple, next_path))) + 180) % 360
    vector_deg = (calcul_vector((x_pos, y_pos), next_path) + 180) % 360
    current_deg = rad_to_norm_deg(current_angle)
    threshold = 20
    distance_deg = 0
    #print(vector_deg, current_deg)
    turn = False

    #print(current_deg, vector_deg)

    if (current_deg - vector_deg) % 360 > 180:
        distance_deg = 360 - ((current_deg - vector_deg) % 360)
        if distance_deg > threshold:
            #print("COMMAND - TURN LEFT")
            return 'b'
        else:
            #print("COMMAND - GO FORWARD")
            return 'a'
    else:
        distance_deg = (current_deg - vector_deg) % 360
        if distance_deg > threshold:
            #print("COMMAND - TURN RIGHT")
            return 'c'
        else:
            #print("COMMAND - GO FORWARD")
            return 'a'

def send_command(current_command, new_command, ser):
    """
        DESCRIPTION: will send message to controler if it's different.
        INTPUT:
            * ser = object who control the serial port.
        OUTPUT:
            * last_command = (char) return the last send command.
    """
    last_command = current_command
    if current_command != new_command:
        print(current_command)
        ser.write(new_command.encode())
        last_command = new_command
        """clear = lambda: os.system('clear')
        clear()"""
        print("NEW COMMAND ",last_command)
    return last_command

def showing_mapping():
    """
        DESCRIPTION: Run a 3D mapping section and get live result.
    """
    # OPEN MICROCONTROL COMMUNICATION.
    ser = Serial('/dev/ttyUSB0', 9600)
    commande_motor = 'e'
    #ser.write(commande_motor.encode()) 
    
    # READ ARG.
    parser = argparse.ArgumentParser()
    parser.add_argument("path")
    parser.add_argument("x")
    parser.add_argument("y")
    parser.add_argument("z")
    parser.add_argument("r")
    parser.add_argument("p")
    parser.add_argument("y")
    args = parser.parse_args()

    matrice = euler_to_rotation(float(args.r), float(args.p), float(args.y), np.array([args.x, args.y, args.z],dtype=float))

    # READ OBJ OBJECT.
    scene = pywavefront.Wavefront('/home/thomas/Documents/rane_slam/mk3slam.0.3/Final/data/{0}.obj'.format(args.path))
    points_3D = np.zeros((len(scene.vertices),3))
    for i in range(len(scene.vertices)):
        points_3D[i,:] = np.array([scene.vertices[i][0],scene.vertices[i][1],scene.vertices[i][2]])

    # INIT GRILLE.
    print("Initialisation mapping data.")
    map_lab = cv2.imread('/home/thomas/Documents/rane_slam/mk3slam.0.3/3D_to_2D/map_2D.png',0)

    map_lab_nav = np.zeros((map_lab.shape[0], map_lab.shape[1]))        # Navigation Map. (0, 1)
    map_lab_nav = map_lab // 255
    map_lab_nav = matrix_change_2(map_lab_nav, 10)

    map_lab = cv2.cvtColor(map_lab,cv2.COLOR_GRAY2RGB)

    for i in list(range(map_lab.shape[0])):
        for j in list(range(map_lab.shape[1])):
            if(map_lab_nav[i,j] == 2):
                map_lab[i,j,:] = [255,100,0]

    border_param = get_border(points_3D)
    print("Mapping data initialisation complete.")

    # ZED CONFIGURATION CAMERA.        print(path[index*i,0])
    print("Initialisation zed 2 camera.")
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720  # Use HD720 video mode
    init_params.coordinate_units = sl.UNIT.METER         # Set coordinate units
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
    zed.open(init_params)
    print("Camera zed 2 open.")

    # INITIALISATION OBJECT FOR MAPPING.
    pymesh = sl.Mesh()        # Current incremental mesh.
    image = sl.Mat()          # Left image from camera.
    pose = sl.Pose()          # Pose object.

    # TRACKING PARAMETERS.
    tracking_parameters = sl.PositionalTrackingParameters()
    tracking_parameters.enable_area_memory = True
    tracking_parameters.area_file_path = '/home/thomas/Documents/rane_slam/mk3slam.0.3/Final/data/{0}.area'.format(args.path)

    # INITIALISATION POSITION

    t = sl.Transform()
    t[0,0] = matrice[0,0]
    t[1,0] = matrice[1,0]
    t[2,0] = matrice[2,0]
    t[0,1] = matrice[0,1]
    t[1,1] = matrice[1,1]
    t[2,1] = matrice[2,1]
    t[0,2] = matrice[0,2]
    t[1,2] = matrice[1,2]
    t[2,2] = matrice[2,2]
    t[0,3] = matrice[0,3]
    t[1,3] = matrice[1,3]
    t[2,3] = matrice[2,3]
  
    tracking_parameters.set_initial_world_transform(t)
    zed.enable_positional_tracking(tracking_parameters)

    # DEPTH PROXIMITY TRACKING
    res = sl.Resolution()
    res.width = 720
    res.height = 404
    point_cloud = sl.Mat(res.width, res.height, sl.MAT_TYPE.U8_C4, sl.MEM.CPU)


    # TIME PARAMETERS.
    last_call = time.time()
    runtime = sl.RuntimeParameters()

    # INIT NAVIGATION.
    print("Processing the global navigation path.")
    x_pos, y_pos = transform_real_pose_to_grille_pose(np.array([0,0,0]), border_param, map_lab[...,0])
    ELEVATOR = (320, 70)
    map_path = navigation_processing(x_pos, y_pos, map_lab_nav, ELEVATOR)
    u = np.array(map_path)
    key_points = generate_key_point(u)
    print("Process global navigation path complete.")
    case = 0

    commande_motor = 'a'
    ser.write(commande_motor.encode()) 
    time.sleep(0.5)
    commande_motor = 'd'
    ser.write(commande_motor.encode()) 
    # INIT PROCESS COMPLETE
    print("Initialisation process complete.")
    commande_motor = 'e'
    ser.write(commande_motor.encode()) 

    while True:

        # -get image.
        zed.grab(runtime)
        zed.retrieve_image(image, sl.VIEW.LEFT)

        # -get position and spatial mapping state.
        zed.get_position(pose)

        # Display the translation and timestamp
        py_translation = sl.Translation()
        tx = round(pose.get_translation(py_translation).get()[0], 3)
        ty = round(pose.get_translation(py_translation).get()[1], 3)
        tz = round(pose.get_translation(py_translation).get()[2], 3)
        #print("Translation: Tx: {0}, Ty: {1}, Tz {2}, Timestamp: {3}\n".format(tx, ty, tz, pose.timestamp.get_milliseconds()))

        # Display the orientation quaternion
        py_orientation = sl.Orientation()
        ox = round(pose.get_orientation(py_orientation).get()[0], 3)
        oy = round(pose.get_orientation(py_orientation).get()[1], 3)
        oz = round(pose.get_orientation(py_orientation).get()[2], 3)
        ow = round(pose.get_orientation(py_orientation).get()[3], 3)
        #print("Orientation: Ox: {0}, Oy: {1}, Oz {2}, Ow: {3}\n".format(ox, oy, oz, ow))

        # DRAW ALL STATE.
        gl.glLineWidth(1)
        gl.glColor3f(0.0, 0.0, 1.0)

        # -transform brute data in numpy to draw pose.
        a = pose.pose_data()
        pose2 = np.array([[a[0,0],a[0,1],a[0,2],a[0,3]],
                          [a[1,0],a[1,1],a[1,2],a[1,3]],
                          [a[2,0],a[2,1],a[2,2],a[2,3]],
                          [a[3,0],a[3,1],a[3,2],a[3,3]]])
        
        x_pos, y_pos = transform_real_pose_to_grille_pose(pose2[:3,3], border_param, map_lab[...,0])
        copy = map_lab.copy()
        copy_nav = map_lab_nav.copy()
        
        copy[x_pos  ,y_pos  ,:] = [0,0,255]
        copy[x_pos-1,y_pos  ,:] = [0,0,255]
        copy[x_pos-1,y_pos-1,:] = [0,0,255]
        copy[x_pos-1,y_pos+1,:] = [0,0,255]
        copy[x_pos  ,y_pos+1,:] = [0,0,255]
        copy[x_pos  ,y_pos-1,:] = [0,0,255]
        copy[x_pos+1,y_pos  ,:] = [0,0,255]
        copy[x_pos+1,y_pos-1,:] = [0,0,255]
        copy[x_pos+1,y_pos+1,:] = [0,0,255]

        # UPDATE wAITPOINT.
        if key_points_reach(x_pos, y_pos, key_points[case,:]):
            case += 1

        # -get proximity.
        """zed.retrieve_measure(point_cloud, sl.MEASURE.XYZ ,sl.MEM.CPU, res)
        for ligne in point_cloud.get_data()[201:202]:
            for point in ligne:
                point[3] = 1
                if (point[1]**2)**0.5 < 0.3 and point[2] > -1.5 and (point[0]**2)**0.5 < 2.5:
                    x, y = transform_real_pose_to_grille_pose((pose.pose_data().m).dot(point.reshape((4,1))), border_param, map_lab[...,0])
                    i = -1
                    while i < 2:
                        j = -1
                        while j < 2:
                            if (x+i !=  key_points[case,0]) and (y+j !=  key_points[case,0]):
                                copy_nav[x+i, y+j] = 1
                                copy[x+i, y+j, :] = [0, 255, 255]
                            j +=1
                        i +=1"""

        # NAVIGATION PROCESS.
        last_call = time.time()
        map_path = navigation_processing(x_pos, y_pos, copy_nav, (key_points[case,0],key_points[case,1]))
        u = None
        #print(time.time()-last_call)
        if map_path is not None:
            u = np.array(map_path)
            copy[u[:,0],u[:,1]] = [0, 255, 0]
        else:
            #print("No path generate.")
            pass

        # COMMANDE PROCESS.
        if map_path is not None:
            commande_motor = send_command(commande_motor, command_main_function(pose.get_euler_angles()[1], u[:20,:], x_pos, y_pos),ser)
            print(commande_motor)

        cv2.WINDOW_NORMAL
        cv2.namedWindow("windows",0)
        cv2.imshow("windows", copy)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    
    #sl.SPATIAL_MAPPING_STATE.NOT_ENABLED
    zed.disable_positional_tracking()
    zed.close()

if __name__=="__main__":
    showing_mapping()