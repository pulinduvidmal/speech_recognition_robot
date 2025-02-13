import os
import time
import socket
import numpy as np
import wave
import whisper
import URBasic
from flask import Flask, request, jsonify
import threading
import numpy as np

import random

global vehicle_output
vehicle_output="PLANE"

robot_busy=False
robot_overall_process_busy=False
time_interval=5

# Flask app setup
app = Flask(__name__)

# Directory to save uploaded audio files temporarily
UPLOAD_FOLDER = './uploaded_audio'
os.makedirs(UPLOAD_FOLDER, exist_ok=True)
app.config['UPLOAD_FOLDER'] = UPLOAD_FOLDER

# Load the Whisper model
model = whisper.load_model("base")

# Constants for the robot
ROBOT_IP = '192.168.1.200'
ACCELERATION = 0.7
VELOCITY = 0.8
PORT = 30002 




# def test(vehicle_type):
#     robot_busy=True
#     print(vehicle_type)
#     time.sleep(2)
#     robot_busy=False


robot_initialized = False

# [x, y, z, rx, ry, rz]

left_table_centre=[-0.883, -0.293, 0.329,  1.2,2.9, 0.0] 

right_table_centre=[0.882, -0.293, 0.327, 2.917,-1.227,0 ] 


middle_table_centre=[-0.001, -0.463, 0.137,  1.2,-2.9,0] 




initial_pose1 = np.add(middle_table_centre,[0,0,0.6,0,0,0]).tolist()



# Function to send URScript commands to the robot
def send_urscript_command(command, robot_ip, port=PORT):
    try:
        with socket.create_connection((robot_ip, port), timeout=5) as sock:
            sock.sendall(command.encode('utf-8'))
            print(f"Command sent: {command.strip()}")
    except Exception as e:
        print(f"Failed to send command: {e}")


def move_robot_to_pose(robot, pose, acceleration, velocity):
    print(f"Moving robot to pose: {pose}")
    robot.movel(pose, a=acceleration, v=velocity)


# def move_robot_through_path(robot, path, acceleration, velocity, blend_radius=0.05):
#     """
#     Move the robot through a series of waypoints smoothly, stopping exactly at the last one.

#     Parameters:
#     - robot: Robot instance
#     - path: List of poses (each pose is [x, y, z, rx, ry, rz])
#     - acceleration: Motion acceleration
#     - velocity: Motion velocity
#     - blend_radius: Radius to smooth trajectory (default: 0.05)
#     """
#     waypoints = []

#     for i, pose in enumerate(path):
#         waypoint = {
#             "pose": pose,
#             "a": acceleration,
#             "v": velocity,
#             "t": 0,  # No fixed time delay
#             "r": 0.01 if i < len(path) - 1 else 0  # Stop at last waypoint
#         }
#         waypoints.append(waypoint)

#     robot.movej(waypoints)

# def move_robot_through_path(robot, path, acceleration, velocity, blend_radius=0.05):
#     """
#     Move the robot through a series of waypoints smoothly, stopping exactly at the last one.

#     Parameters:
#     - robot: Robot instance
#     - path: List of poses (each pose is [x, y, z, rx, ry, rz])
#     - acceleration: Motion acceleration
#     - velocity: Motion velocity
#     - blend_radius: Radius to smooth trajectory (default: 0.05)
#     """
#     waypoints = []

#     for i, pose in enumerate(path):
#         waypoint = {
#             "pose": pose,
#             "a": acceleration,
#             "v": velocity,
#             "t": 0,  # No fixed time delay
#             "r": blend_radius if i < len(path) - 1 else 0  # Stop at last waypoint
#         }
#         waypoints.append(waypoint)

#     # Extract only poses from the waypoint dictionaries
#     #pose_list = [wp["pose"] for wp in waypoints]

#     # Call movej with only the list of poses
#     robot.movej_waypoints(waypoint)




def control_gripper(robot_ip, close):
    time.sleep(0.05)
    robot.sync()
    if close:
        send_urscript_command("set_tool_digital_out(0, False)\n", robot_ip)
        
        robot.sync()
        #send_urscript_command("set_tool_digital_out(0, False)\n", robot_ip)
        #time.sleep(0.05)
        send_urscript_command("set_tool_digital_out(1, True)\n", robot_ip)
        robot.sync()
       # send_urscript_command("set_tool_digital_out(1, True)\n", robot_ip)
       
    else:
        send_urscript_command("set_tool_digital_out(1, False)\n", robot_ip)
        #send_urscript_command("set_tool_digital_out(1, False)\n", robot_ip)
        #time.sleep(0.05)
        robot.sync()
        send_urscript_command("set_tool_digital_out(0, True)\n", robot_ip)
        robot.sync()
       # send_urscript_command("set_tool_digital_out(0, True)\n", robot_ip)
    time.sleep(2)


if not robot_initialized:
    global robot
    print("Initializing robot")
    robot_model = URBasic.robotModel.RobotModel()
    robot = URBasic.urScriptExt.UrScriptExt(host=ROBOT_IP, robotModel=robot_model)
    robot.reset_error()
    print("Robot initialized")
    control_gripper(ROBOT_IP, close=False)
    time.sleep(2)
    move_robot_to_pose(robot, initial_pose1, ACCELERATION, VELOCITY)
    move_robot_to_pose(robot, initial_pose1, ACCELERATION, VELOCITY)
    time.sleep(1)
    





component_positions = {
    "car": {
        "body": np.add(right_table_centre,[-0.155, 0, 0,  0, 0, 0]).tolist(),
        "wheels_A":np.add(left_table_centre,[-0.085, -0.12, 0,  0, 0, 0]).tolist(),
        "wheels_B": np.add(left_table_centre,[ -0.085,-0.225, 0,  0, 0, 0]).tolist(),
        "chasis": np.add(left_table_centre,[-0.085, 0.13, 0,  0, 0, 0]).tolist()
        
    },
    "bus": {
         "wheels_A":np.add(left_table_centre,[-0.085, -0.12, 0,  0, 0, 0]).tolist(),
        "wheels_B": np.add(left_table_centre,[ -0.085,-0.225, 0,  0, 0, 0]).tolist(),
       "chasis": np.add(left_table_centre,[-0.085, 0.13, 0,  0, 0, 0]).tolist(),
        
        "body": np.add(right_table_centre,[-0.155, 0.17, 0,  0, 0, 0]).tolist()
    },
    "motorbike": {  

        "wheels_B": np.add(left_table_centre,[ -0.085,-0.225, 0,  0, 0, 0]).tolist(),
        "body": np.add(right_table_centre,[-0.155, -0.15, 0,  0, 0, 0]).tolist(),
        "trunk": np.add(right_table_centre,[0.075, 0, 0,  0, 0, 0]).tolist()
    },
    "plane": {
        "wheels_B": np.add(left_table_centre,[ 0.155,-0.225, 0,  0, 0, 0]).tolist(),
        "left_wing": np.add(left_table_centre,[-0.085, 0, 0,  0, 0, 0]).tolist(),
        "right_wing": np.add(left_table_centre,[0.155, -0.21, 0,  0, 0, 0]).tolist(),
        "body": np.add(right_table_centre,[0.075, -0.15, 0,  0, 0, 0]).tolist()
    },
    "ship": {
        "hull": np.add(left_table_centre,[0.155, 0.165, 0,  0, 0, 0]).tolist(),
        "cabin": np.add(right_table_centre,[0.075, 0.185, 0,  0, 0, 0]).tolist(),
        "deck": np.add(left_table_centre,[0.155, -0.065, 0,  0, 0, 0]).tolist()
    },
    "place": {
        "0": [-0.136,-0.531,0.137,1.203,-2.9,0],
        "1":np.add(middle_table_centre, [-0.129, -0.064, 0,   0, 0, 0]).tolist(),
        "2":np.add(middle_table_centre, [-0.11, 0, 0,   0, 0, 0]).tolist(),
        "3": np.add(middle_table_centre,[0.0, 0.11, 0,  0, 0, 0]).tolist(),
        "4": np.add(middle_table_centre,[0.0, -0.09, 0,  0, 0, 0]).tolist(),
        "carbody": np.add(middle_table_centre,[0,0.075, 0,  0, 0, 0]).tolist(),
        "carwheelsA": np.add(middle_table_centre,[0.11, 0.015, 0,  0, 0, 0]).tolist(),
        "carwheelsB": np.add(middle_table_centre,[-0.11, 0.015, 0,  0, 0, 0]).tolist(),
        "carchasis": np.add(middle_table_centre,[0,-0.05, 0,  0, 0, 0]).tolist(),

        "motorbikewheel": np.add(middle_table_centre,[0.06, -0.09, 0,  0, 0, 0]).tolist(),
         "motorbikebody": np.add(middle_table_centre,[0.06, 0.02, 0,  0, 0, 0]).tolist(),
         "motorbiketrunk": np.add(middle_table_centre,[-0.04, 0.02, 0,  0, 0, 0]).tolist(),

         "planewheel": np.add(middle_table_centre,[0.11, 0, 0,  0, 0, 0]).tolist(),
         "planeleft": np.add(middle_table_centre,[0.0, 0.15, 0,  0, 0, 0]).tolist(),
         "planeright": np.add(middle_table_centre,[0.0, -0.08, 0,  0, 0, 0]).tolist(),
         "planebody": np.add(middle_table_centre,[0.0, 0.04, 0,  0, 0, 0]).tolist(),


         "shiphull": np.add(middle_table_centre,[0.0, 0.05, 0,  0, 0, 0]).tolist(),
         "shipdeck": np.add(middle_table_centre,[-0.09, -0.04, 0,  0, 0, 0]).tolist(),
         "shipcabin": np.add(middle_table_centre,[0.09, -0.04, 0,  0, 0, 0]).tolist()


    },
    "traj": {
        "right_table": [0.5, -0.345, 0.279,  1.167, -2.944, -0.016],
        "placing_table": [0.038, -0.404, 0.368,  1.167, -2.944, -0.016] ,
        "left_table": [-0.620, -0.016, 0.379,  1.167, -2.944, -0.016]
    },
    "traj_car": {
        "body": np.add(right_table_centre,[-0.15, 0, 0.5,0, 0, 0]).tolist(),
        "wheels_A":np.add(left_table_centre,[-0.15, -0.15, 0.5,  0, 0, 0]).tolist(),
        "wheels_B": np.add(left_table_centre,[0.11, -0.11, 0.5,  0, 0, 0]).tolist(),
        "chasis": np.add(left_table_centre,[-0.15, 0.15, 0.5,  0, 0, 0]).tolist()
    },
    "traj_bus": {
        "wheels_A":np.add(left_table_centre,[-0.11, -0.11, 0.5,  0., 0, 0]).tolist(),
        "wheels_B": np.add(left_table_centre,[0.11
        , -0.11, 0.5,  0, 0, 0]).tolist(),
        "chasis": np.add(left_table_centre,[-0.11, 0.11, 0.5,  0, 0, 0]).tolist(),
        "body": np.add(right_table_centre,[-0.11, 0.11, 0.5,  0, 0, 0]).tolist()
    },
    "traj_motorbike": {
        "wheels_B":np.add(left_table_centre,[0.11, -0.11, 0.5,  0, 0, 0]).tolist(),
        "body": np.add(right_table_centre,[-0.11, -0.11, 0.5,  0, 0, 0]).tolist(),
        "trunk": np.add(right_table_centre,[0.11, 0, 0.5,  0, 0, 0]).tolist()
    },
    "traj_plane": {
        "wheels_B": np.add(left_table_centre,[0.11, -0.11, 0.5,  0, 0, 0]).tolist(),
        "left_wing": np.add(left_table_centre,[-0.11, 0, 0.5,  0, 0, 0]).tolist(),
        "right_wing": np.add(left_table_centre,[0.11, 0, 0.5,  0, 0, 0]).tolist(),
        "body": np.add(right_table_centre,[0.11, -0.11, 0.5,  0, 0, 0]).tolist()
    },
    "traj_ship": {
        "hull": np.add(left_table_centre,[0.11, 0.21, 0.5,  0, 0, 0]).tolist(),
        "cabin": np.add(left_table_centre,[0.11, 0.11, 0.5,  0, 0, 0]).tolist(),
        "deck": np.add(right_table_centre,[0.11, 0.11, 0.5,  0, 0, 0]).tolist()
    },
    "traj_place": {
        "0":[-0.135,-0.250,0.937,1.203,-2.9,0],
        "1":np.add(middle_table_centre, [0.11, 0, 0.6,  0, 0, 0] ).tolist(),
        "2":np.add(middle_table_centre, [-0.11, 0, 0.6,  0, 0, 0]).tolist(),
        "3":np.add(middle_table_centre,[0.0, 0.11, 0.6,  0, 0, 0]).tolist(),
        "4":np.add(middle_table_centre,[0.0, -0.09, 0.6,  0, 0, 0]).tolist(),
        "carbody": np.add(middle_table_centre,[0,0.075, 0.6,  0, 0, 0]).tolist(),
        "carwheelsA": np.add(middle_table_centre,[
        0.11, 0, 0.6,  0, 0, 0]).tolist(),
        "carwheelsB": np.add(middle_table_centre,[-0.11, 0, 0.6,  0, 0, 0]).tolist(),
        "carchasis": np.add(middle_table_centre,[0,-0.05, 0.6,  0, 0, 0]).tolist(),

        "motorbikewheel": np.add(middle_table_centre,[0.06, -0.09, 0.6,  0, 0, 0]).tolist(),
         "motorbikebody": np.add(middle_table_centre,[0.06, 0.02, 0.6,  0, 0, 0]).tolist(),
         "motorbiketrunk": np.add(middle_table_centre,[-0.04, 0.02, 0.6,  0, 0, 0]).tolist(),


         "planewheel": np.add(middle_table_centre,[0.11, 0, 0.6,  0, 0, 0]).tolist(),
         "planeleft": np.add(middle_table_centre,[0.0, 0.15, 0.6,  0, 0, 0]).tolist(),
         "planeright": np.add(middle_table_centre,[0.0, -0.08, 0.6,  0, 0, 0]).tolist(),
         "planebody": np.add(middle_table_centre,[0.0, 0.04, 0.6,  0, 0, 0]).tolist(),


          "shiphull": np.add(middle_table_centre,[0.0, 0.05, 0.6,  0, 0, 0]).tolist(),
         "shipdeck": np.add(middle_table_centre,[-0.09, -0.04, 0.6,  0, 0, 0]).tolist(),
         "shipcabin": np.add(middle_table_centre,[0.09, -0.04, 0.6,  0, 0, 0]).tolist()
    }


}


# Function to process the audio file
def process_audio(file_path):
    global vehicle_output
    def load_audio(file_path, sr=16000):
        with wave.open(file_path, "rb") as wf:
            n_frames = wf.getnframes()
            audio_data = np.frombuffer(wf.readframes(n_frames), dtype=np.int16)
            audio_data = audio_data.astype(np.float32) / 32768.0  # Normalize to [-1.0, 1.0]
            return audio_data

    def check_keywords(text, keywords):
        found = [word for word in keywords if word in text.lower()]
        return found
        #return next((word for word in keywords if word in text.lower()), None)

    # Load and preprocess the audio
    audio = load_audio(file_path)
    audio = whisper.pad_or_trim(audio)#"wheels_A":np.add(left_table_centre,[-0.11, -0.11, 0,  0, 0, 0]).tolist(),
    mel = whisper.log_mel_spectrogram(audio).to(model.device)

    # Decode with Whisper
    options = whisper.DecodingOptions(language="en")
    result = whisper.decode(model, mel, options)

    decoded_text = result.text.lower()
    print(f"Decoded text: {decoded_text}")

    #keywords = ["car", "bus", "motorbike", "plane", "ship"]
    keywords = ["ca", "bus", "mot", "pla", "sh"]

    # Check if keywords exist in the transcribed text
    matched_keywords = check_keywords(decoded_text, keywords)

    if matched_keywords:
        print(matched_keywords[0])
        if matched_keywords[0] == "ca": vehicle_output="CAR"
        elif matched_keywords[0] == "bus": vehicle_output="BUS"
        elif matched_keywords[0] == "mot": vehicle_output="MOTORBIKE"
        elif matched_keywords[0] == "pla": vehicle_output="PLANE"
        elif matched_keywords[0] == "sh": vehicle_output="SHIP"

        return matched_keywords[0].upper()  # Return the first recognized keyword in uppercase
    else:
        return None
    #return check_keywords(decoded_text, keywords)



def initilize_all():
    control_gripper(ROBOT_IP, close=False)
    #time.sleep(2)
    print("initilizing")
    #time.sleep(2)


 
 ##1
    move_robot_to_pose(robot, component_positions["traj_place"]["0"], ACCELERATION, VELOCITY)
    #time.sleep(10)
    move_robot_to_pose(robot, component_positions["place"]["0"], ACCELERATION, VELOCITY)
    control_gripper(ROBOT_IP, close=True)
    move_robot_to_pose(robot, component_positions["traj_place"]["0"], ACCELERATION, VELOCITY)
    
    move_robot_to_pose(robot, component_positions["traj_car"]["wheels_A"] ,ACCELERATION, VELOCITY)
    move_robot_to_pose(robot, component_positions["car"]["wheels_A"], ACCELERATION, VELOCITY)
    control_gripper(ROBOT_IP, close=False)
    move_robot_to_pose(robot, component_positions["traj_car"]["wheels_A"] , ACCELERATION, VELOCITY)


#2
    move_robot_to_pose(robot, component_positions["traj_place"]["0"], ACCELERATION, VELOCITY)
    move_robot_to_pose(robot, component_positions["place"]["0"], ACCELERATION, VELOCITY)
    control_gripper(ROBOT_IP, close=True)
    move_robot_to_pose(robot, component_positions["traj_place"]["0"], ACCELERATION, VELOCITY)
    
    move_robot_to_pose(robot, component_positions["traj_car"]["wheels_B"] ,ACCELERATION, VELOCITY)
    move_robot_to_pose(robot, component_positions["car"]["wheels_B"], ACCELERATION, VELOCITY)
    control_gripper(ROBOT_IP, close=False)
    move_robot_to_pose(robot, component_positions["traj_car"]["wheels_B"] , ACCELERATION, VELOCITY)


#3
    move_robot_to_pose(robot, component_positions["traj_place"]["0"], ACCELERATION, VELOCITY)
    move_robot_to_pose(robot, component_positions["place"]["0"], ACCELERATION, VELOCITY)
    control_gripper(ROBOT_IP, close=True)
    move_robot_to_pose(robot, component_positions["traj_place"]["0"], ACCELERATION, VELOCITY)
    
    move_robot_to_pose(robot, component_positions["traj_plane"]["left_wing"] ,ACCELERATION, VELOCITY)
    move_robot_to_pose(robot, component_positions["plane"]["left_wing"], ACCELERATION, VELOCITY)
    control_gripper(ROBOT_IP, close=False)
    move_robot_to_pose(robot, component_positions["traj_plane"]["left_wing"] , ACCELERATION, VELOCITY)


#4
    move_robot_to_pose(robot, component_positions["traj_place"]["0"], ACCELERATION, VELOCITY)
    move_robot_to_pose(robot, component_positions["place"]["0"], ACCELERATION, VELOCITY)
    control_gripper(ROBOT_IP, close=True)
    move_robot_to_pose(robot, component_positions["traj_place"]["0"], ACCELERATION, VELOCITY)
    
    move_robot_to_pose(robot, component_positions["traj_plane"]["right_wing"] ,ACCELERATION, VELOCITY)
    move_robot_to_pose(robot, component_positions["plane"]["right_wing"], ACCELERATION, VELOCITY)
    control_gripper(ROBOT_IP, close=False)
    move_robot_to_pose(robot, component_positions["traj_plane"]["right_wing"] , ACCELERATION, VELOCITY)


#5
    move_robot_to_pose(robot, component_positions["traj_place"]["0"], ACCELERATION, VELOCITY)
    move_robot_to_pose(robot, component_positions["place"]["0"], ACCELERATION, VELOCITY)
    control_gripper(ROBOT_IP, close=True)
    move_robot_to_pose(robot, component_positions["traj_place"]["0"], ACCELERATION, VELOCITY)

    move_robot_to_pose(robot, component_positions["traj_car"]["chasis"] ,ACCELERATION, VELOCITY)
    move_robot_to_pose(robot, component_positions["car"]["chasis"], ACCELERATION, VELOCITY)
    control_gripper(ROBOT_IP, close=False)
    move_robot_to_pose(robot, component_positions["traj_car"]["chasis"], ACCELERATION, VELOCITY)


#6
    move_robot_to_pose(robot, component_positions["traj_place"]["0"], ACCELERATION, VELOCITY)
    move_robot_to_pose(robot, component_positions["place"]["0"], ACCELERATION, VELOCITY)
    control_gripper(ROBOT_IP, close=True)
    control_gripper(ROBOT_IP, close=True)
    move_robot_to_pose(robot, component_positions["traj_place"]["0"], ACCELERATION, VELOCITY)

    move_robot_to_pose(robot, component_positions["traj_ship"]["deck"] ,ACCELERATION, VELOCITY)
    move_robot_to_pose(robot, component_positions["ship"]["deck"], ACCELERATION, VELOCITY)
    control_gripper(ROBOT_IP, close=False)
    control_gripper(ROBOT_IP, close=False)
    move_robot_to_pose(robot, component_positions["traj_ship"]["deck"], ACCELERATION, VELOCITY)


#7
    move_robot_to_pose(robot, component_positions["traj_place"]["0"], ACCELERATION, VELOCITY)
    move_robot_to_pose(robot, component_positions["place"]["0"], ACCELERATION, VELOCITY)
    control_gripper(ROBOT_IP, close=True)
    move_robot_to_pose(robot, component_positions["traj_place"]["0"], ACCELERATION, VELOCITY)

    move_robot_to_pose(robot, component_positions["traj_ship"]["hull"] ,ACCELERATION, VELOCITY)
    move_robot_to_pose(robot, component_positions["ship"]["hull"], ACCELERATION, VELOCITY)
    control_gripper(ROBOT_IP, close=False)
    move_robot_to_pose(robot, component_positions["traj_ship"]["hull"], ACCELERATION, VELOCITY)


#8
    move_robot_to_pose(robot, component_positions["traj_place"]["0"], ACCELERATION, VELOCITY)
    move_robot_to_pose(robot, component_positions["place"]["0"], ACCELERATION, VELOCITY)
    control_gripper(ROBOT_IP, close=True)
    move_robot_to_pose(robot, component_positions["traj_place"]["0"], ACCELERATION, VELOCITY)

    move_robot_to_pose(robot, component_positions["traj_motorbike"]["body"] ,ACCELERATION, VELOCITY)
    move_robot_to_pose(robot, component_positions["motorbike"]["body"], ACCELERATION, VELOCITY)
    control_gripper(ROBOT_IP, close=False)
    move_robot_to_pose(robot, component_positions["traj_motorbike"]["body"], ACCELERATION, VELOCITY)


#9
    move_robot_to_pose(robot, component_positions["traj_place"]["0"], ACCELERATION, VELOCITY)
    move_robot_to_pose(robot, component_positions["place"]["0"], ACCELERATION, VELOCITY)
    control_gripper(ROBOT_IP, close=True)
    move_robot_to_pose(robot, component_positions["traj_place"]["0"], ACCELERATION, VELOCITY)

    move_robot_to_pose(robot, component_positions["traj_plane"]["body"] ,ACCELERATION, VELOCITY)
    move_robot_to_pose(robot, component_positions["plane"]["body"], ACCELERATION, VELOCITY)
    control_gripper(ROBOT_IP, close=False)
    move_robot_to_pose(robot, component_positions["traj_plane"]["body"], ACCELERATION, VELOCITY)


#10
    move_robot_to_pose(robot, component_positions["traj_place"]["0"], ACCELERATION, VELOCITY)
    move_robot_to_pose(robot, component_positions["place"]["0"], ACCELERATION, VELOCITY)
    control_gripper(ROBOT_IP, close=True)
    move_robot_to_pose(robot, component_positions["traj_place"]["0"], ACCELERATION, VELOCITY)

    move_robot_to_pose(robot, component_positions["traj_car"]["body"] ,ACCELERATION, VELOCITY)
    move_robot_to_pose(robot, component_positions["car"]["body"], ACCELERATION, VELOCITY)
    control_gripper(ROBOT_IP, close=False)
    move_robot_to_pose(robot, component_positions["traj_car"]["body"], ACCELERATION, VELOCITY)


#11
    move_robot_to_pose(robot, component_positions["traj_place"]["0"], ACCELERATION, VELOCITY)
    move_robot_to_pose(robot, component_positions["place"]["0"], ACCELERATION, VELOCITY)
    control_gripper(ROBOT_IP, close=True)
    move_robot_to_pose(robot, component_positions["traj_place"]["0"], ACCELERATION, VELOCITY)

    move_robot_to_pose(robot, component_positions["traj_motorbike"]["trunk"] ,ACCELERATION, VELOCITY)
    move_robot_to_pose(robot, component_positions["motorbike"]["trunk"], ACCELERATION, VELOCITY)
    control_gripper(ROBOT_IP, close=False)
    move_robot_to_pose(robot, component_positions["traj_motorbike"]["trunk"], ACCELERATION, VELOCITY)

#12
    move_robot_to_pose(robot, component_positions["traj_place"]["0"], ACCELERATION, VELOCITY)
    move_robot_to_pose(robot, component_positions["place"]["0"], ACCELERATION, VELOCITY)
    control_gripper(ROBOT_IP, close=True)
    move_robot_to_pose(robot, component_positions["traj_place"]["0"], ACCELERATION, VELOCITY)

    move_robot_to_pose(robot, component_positions["traj_bus"]["body"] ,ACCELERATION, VELOCITY)
    move_robot_to_pose(robot, component_positions["bus"]["body"], ACCELERATION, VELOCITY)
    control_gripper(ROBOT_IP, close=False)
    move_robot_to_pose(robot, component_positions["traj_bus"]["body"], ACCELERATION, VELOCITY)


#13
    move_robot_to_pose(robot, component_positions["traj_place"]["0"], ACCELERATION, VELOCITY)
    move_robot_to_pose(robot, component_positions["place"]["0"], ACCELERATION, VELOCITY)
    control_gripper(ROBOT_IP, close=True)
    move_robot_to_pose(robot, component_positions["traj_place"]["0"], ACCELERATION, VELOCITY)

    move_robot_to_pose(robot, component_positions["traj_ship"]["cabin"] ,ACCELERATION, VELOCITY)
    move_robot_to_pose(robot, component_positions["ship"]["cabin"], ACCELERATION, VELOCITY)
    control_gripper(ROBOT_IP, close=False)
    move_robot_to_pose(robot, component_positions["traj_ship"]["cabin"], ACCELERATION, VELOCITY)



# Function to run the robot assembly sequence based on the detected keyword
def execute_robot_sequence(vehicle_type):
    
    start_time=time.time()

    global robot_busy
    global robot_overall_process_busy
    robot_busy=True
    robot_overall_process_busy=True
    #vehicle_type ="CAR"

    
####################################car###############################
    #while (True):
        #vehicle_type ="CAR"
    if vehicle_type =="CA":
        

        
        move_robot_to_pose(robot, component_positions["car"]["wheels_A"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["car"]["wheels_A"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=True)
        
        move_robot_to_pose(robot, component_positions["traj_place"]["carwheelsA"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["traj_place"]["carwheelsA"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["carwheelsA"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["carwheelsA"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=False)
        move_robot_to_pose(robot, component_positions["traj_place"]["carwheelsA"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["traj_place"]["carwheelsA"], ACCELERATION, VELOCITY)
        


        move_robot_to_pose(robot, component_positions["car"]["body"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["car"]["body"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=True)
        
        move_robot_to_pose(robot, component_positions["traj_place"]["carbody"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["traj_place"]["carbody"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["carbody"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["carbody"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=False)
        move_robot_to_pose(robot, component_positions["traj_place"]["carbody"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["traj_place"]["carbody"], ACCELERATION, VELOCITY)

        
        move_robot_to_pose(robot, component_positions["car"]["chasis"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["car"]["chasis"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=True)
        
        move_robot_to_pose(robot, component_positions["traj_place"]["carchasis"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["traj_place"]["carchasis"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["carchasis"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["carchasis"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=False)
        move_robot_to_pose(robot, component_positions["traj_place"]["carchasis"], ACCELERATION, VELOCITY)  
        move_robot_to_pose(robot, component_positions["traj_place"]["carchasis"], ACCELERATION, VELOCITY)
        # move_robot_to_pose(robot, initial_pose1, ACCELERATION, VELOCITY)

        print("Car assembly complete.")
        robot_busy=False 
        time.sleep(time_interval)
        


######reverse
        move_robot_to_pose(robot, component_positions["traj_place"]["carchasis"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["traj_place"]["carchasis"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["carchasis"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["carchasis"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=True)
        move_robot_to_pose(robot, component_positions["traj_place"]["carchasis"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["traj_place"]["carchasis"], ACCELERATION, VELOCITY)

        # move_robot_to_pose(robot, component_positions["traj_car"]["chasis"] ,ACCELERATION, VELOCITY)
        # move_robot_to_pose(robot, component_positions["traj_car"]["chasis"] ,ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["car"]["chasis"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["car"]["chasis"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=False)
        # move_robot_to_pose(robot, component_positions["traj_car"]["chasis"], ACCELERATION, VELOCITY)
        # move_robot_to_pose(robot, component_positions["traj_car"]["chasis"], ACCELERATION, VELOCITY)

        move_robot_to_pose(robot, component_positions["traj_place"]["carbody"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["traj_place"]["carbody"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["carbody"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["carbody"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=True)
        move_robot_to_pose(robot, component_positions["traj_place"]["carbody"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["traj_place"]["carbody"], ACCELERATION, VELOCITY)
                
        # move_robot_to_pose(robot, component_positions["traj_car"]["body"], ACCELERATION, VELOCITY)  
        # move_robot_to_pose(robot, component_positions["traj_car"]["body"], ACCELERATION, VELOCITY)  
        move_robot_to_pose(robot, component_positions["car"]["body"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["car"]["body"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=False)
        
        
        move_robot_to_pose(robot, component_positions["traj_place"]["carwheelsA"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["traj_place"]["carwheelsA"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["carwheelsA"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["carwheelsA"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=True)
        move_robot_to_pose(robot, component_positions["traj_place"]["carwheelsA"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["traj_place"]["carwheelsA"], ACCELERATION, VELOCITY)
        
        # move_robot_to_pose(robot, component_positions["traj_car"]["wheels_A"] ,ACCELERATION, VELOCITY)
        # move_robot_to_pose(robot, component_positions["traj_car"]["wheels_A"] ,ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["car"]["wheels_A"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["car"]["wheels_A"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=False)
        # move_robot_to_pose(robot, component_positions["traj_car"]["wheels_A"], ACCELERATION, VELOCITY)
        # move_robot_to_pose(robot, component_positions["traj_car"]["wheels_A"], ACCELERATION, VELOCITY)
        
        move_robot_to_pose(robot, initial_pose1, ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, initial_pose1, ACCELERATION, VELOCITY)

        
        


##################################################################bus#######################################################
    
        
    #vehicle_type ="BUS" 
    if vehicle_type =="BUS":
        # time.sleep(10)
        # print("bus assembly complete.")'


        #  Move to left table and pick wheels_A
        #move_robot_to_pose(robot, component_positions["traj_bus"]["wheels_A"], ACCELERATION, VELOCITY)
        
        move_robot_to_pose(robot, component_positions["bus"]["wheels_A"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["bus"]["wheels_A"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=True)
        #move_robot_to_pose(robot, component_positions["traj_bus"]["wheels_A"], ACCELERATION, VELOCITY)

        #  Move to placing table and place at position 2
        move_robot_to_pose(robot, component_positions["traj_place"]["carwheelsA"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["traj_place"]["carwheelsA"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["carwheelsA"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["carwheelsA"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=False)
        move_robot_to_pose(robot, component_positions["traj_place"]["carwheelsA"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["traj_place"]["carwheelsA"], ACCELERATION, VELOCITY)

        
        #  Move to left table and pick body
        #move_robot_to_pose(robot, component_positions["traj_bus"]["body"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["bus"]["body"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["bus"]["body"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=True)
        
        
        
        #  Move to placing table and place at position 1
        move_robot_to_pose(robot, component_positions["traj_place"]["carbody"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["traj_place"]["carbody"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["carbody"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["carbody"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=False)
        move_robot_to_pose(robot, component_positions["traj_place"]["carbody"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["traj_place"]["carbody"], ACCELERATION, VELOCITY)

        

        # Move to right table and pick chasisi
        #move_robot_to_pose(robot, component_positions["traj_bus"]["chasis"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["bus"]["chasis"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["bus"]["chasis"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=True)
        #move_robot_to_pose(robot, component_positions["traj_bus"]["chasis"], ACCELERATION, VELOCITY)

        # Move to placing table and place at position 4
        move_robot_to_pose(robot, component_positions["traj_place"]["carchasis"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["traj_place"]["carchasis"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["carchasis"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["carchasis"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=False)
        move_robot_to_pose(robot, component_positions["traj_place"]["carchasis"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["traj_place"]["carchasis"], ACCELERATION, VELOCITY)

        
        

        print("bus assembly complete.")
        robot_busy=False 
        time.sleep(time_interval)


######reverse
        move_robot_to_pose(robot, component_positions["traj_place"]["carchasis"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["traj_place"]["carchasis"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["carchasis"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["carchasis"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=True)
        move_robot_to_pose(robot, component_positions["traj_place"]["carchasis"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["traj_place"]["carchasis"], ACCELERATION, VELOCITY)

        #move_robot_to_pose(robot, component_positions["traj_bus"]["chasis"] ,ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["bus"]["chasis"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["bus"]["chasis"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=False)
        #move_robot_to_pose(robot, component_positions["traj_bus"]["chasis"], ACCELERATION, VELOCITY)

        move_robot_to_pose(robot, component_positions["traj_place"]["carbody"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["traj_place"]["carbody"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["carbody"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["carbody"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=True)
        move_robot_to_pose(robot, component_positions["traj_place"]["carbody"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["traj_place"]["carbody"], ACCELERATION, VELOCITY)
                
        #move_robot_to_pose(robot, component_positions["traj_bus"]["body"], ACCELERATION, VELOCITY)  
        move_robot_to_pose(robot, component_positions["bus"]["body"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["bus"]["body"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=False)
        # move_robot_to_pose(robot, component_positions["traj_bus"]["body"], ACCELERATION, VELOCITY)
    
        # move_robot_to_pose(robot, component_positions["traj_place"]["carwheelsB"], ACCELERATION, VELOCITY)
        # move_robot_to_pose(robot, component_positions["place"]["carwheelsB"], ACCELERATION, VELOCITY)
        # control_gripper(ROBOT_IP, close=True)
        # move_robot_to_pose(robot, component_positions["traj_place"]["carwheelsB"], ACCELERATION, VELOCITY)
        
        # move_robot_to_pose(robot, component_positions["traj_bus"]["wheels_B"], ACCELERATION, VELOCITY)
        # move_robot_to_pose(robot, component_positions["bus"]["wheels_B"], ACCELERATION, VELOCITY)
        # control_gripper(ROBOT_IP, close=False)
        # move_robot_to_pose(robot, component_positions["traj_bus"]["wheels_B"], ACCELERATION, VELOCITY)
        
        move_robot_to_pose(robot, component_positions["traj_place"]["carwheelsA"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["traj_place"]["carwheelsA"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["carwheelsA"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["carwheelsA"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=True)
        move_robot_to_pose(robot, component_positions["traj_place"]["carwheelsA"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["traj_place"]["carwheelsA"], ACCELERATION, VELOCITY)
        
        #move_robot_to_pose(robot, component_positions["traj_bus"]["wheels_A"] ,ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["bus"]["wheels_A"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["bus"]["wheels_A"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=False)
        #move_robot_to_pose(robot, component_positions["traj_bus"]["wheels_A"], ACCELERATION, VELOCITY)
        
        move_robot_to_pose(robot, initial_pose1, ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, initial_pose1, ACCELERATION, VELOCITY)
        

#########################################################motorbike##################################################
    
    #vehicle_type ="MOTORBIKE"
    if vehicle_type =="MOT":

        
        
    
        
        #  Move to left table and pick body
        #move_robot_to_pose(robot, component_positions["traj_motorbike"]["body"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["motorbike"]["body"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["motorbike"]["body"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=True)
        #move_robot_to_pose(robot, component_positions["traj_motorbike"]["body"], ACCELERATION, VELOCITY)
        #  Move to placing table and place at position 1
        move_robot_to_pose(robot, component_positions["traj_place"]["motorbikebody"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["traj_place"]["motorbikebody"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["motorbikebody"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["motorbikebody"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=False)
        move_robot_to_pose(robot, component_positions["traj_place"]["motorbikebody"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["traj_place"]["motorbikebody"], ACCELERATION, VELOCITY)

        #  Move to left table and pick wheels_A
        #move_robot_to_pose(robot, component_positions["traj_motorbike"]["wheels_B"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["motorbike"]["wheels_B"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["motorbike"]["wheels_B"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=True)
        #move_robot_to_pose(robot, component_positions["traj_motorbike"]["wheels_B"], ACCELERATION, VELOCITY)
        #  Move to placing table and place at position 2
        move_robot_to_pose(robot, component_positions["traj_place"]["motorbikewheel"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["traj_place"]["motorbikewheel"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["motorbikewheel"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["motorbikewheel"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=False)
        move_robot_to_pose(robot, component_positions["traj_place"]["motorbikewheel"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["traj_place"]["motorbikewheel"], ACCELERATION, VELOCITY)


        

        # Move to right table and pick chasisi
        #move_robot_to_pose(robot, component_positions["traj_motorbike"]["trunk"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["motorbike"]["trunk"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["motorbike"]["trunk"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=True)
        #move_robot_to_pose(robot, component_positions["traj_motorbike"]["trunk"], ACCELERATION, VELOCITY)
        # Move to placing table and place at position 4
        move_robot_to_pose(robot, component_positions["traj_place"]["motorbiketrunk"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["traj_place"]["motorbiketrunk"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["motorbiketrunk"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["motorbiketrunk"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=False)
        move_robot_to_pose(robot, component_positions["traj_place"]["motorbiketrunk"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["traj_place"]["motorbiketrunk"], ACCELERATION, VELOCITY)

        
        # move_robot_to_pose(robot, initial_pose1, ACCELERATION, VELOCITY)
        # move_robot_to_pose(robot, initial_pose1, ACCELERATION, VELOCITY)

        print("motorbike assembly complete.")
        robot_busy=False 
        time.sleep(time_interval)


######reverse
        move_robot_to_pose(robot, component_positions["traj_place"]["motorbiketrunk"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["traj_place"]["motorbiketrunk"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["motorbiketrunk"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["motorbiketrunk"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=True)
        move_robot_to_pose(robot, component_positions["traj_place"]["motorbiketrunk"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["traj_place"]["motorbiketrunk"], ACCELERATION, VELOCITY)

        # move_robot_to_pose(robot, component_positions["traj_motorbike"]["trunk"] ,ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["motorbike"]["trunk"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["motorbike"]["trunk"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=False)
        #move_robot_to_pose(robot, component_positions["traj_motorbike"]["trunk"], ACCELERATION, VELOCITY)


        move_robot_to_pose(robot, component_positions["traj_place"]["motorbikewheel"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["traj_place"]["motorbikewheel"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["motorbikewheel"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["motorbikewheel"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=True)
        move_robot_to_pose(robot, component_positions["traj_place"]["motorbikewheel"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["traj_place"]["motorbikewheel"], ACCELERATION, VELOCITY)
        
        #move_robot_to_pose(robot, component_positions["traj_motorbike"]["wheels_B"] ,ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["motorbike"]["wheels_B"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["motorbike"]["wheels_B"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=False)
        #move_robot_to_pose(robot, component_positions["traj_motorbike"]["wheels_B"], ACCELERATION, VELOCITY)

        move_robot_to_pose(robot, component_positions["traj_place"]["motorbikebody"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["traj_place"]["motorbikebody"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["motorbikebody"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["motorbikebody"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=True)
        move_robot_to_pose(robot, component_positions["traj_place"]["motorbikebody"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["traj_place"]["motorbikebody"], ACCELERATION, VELOCITY)
                
        #move_robot_to_pose(robot, component_positions["traj_motorbike"]["body"], ACCELERATION, VELOCITY)  
        move_robot_to_pose(robot, component_positions["motorbike"]["body"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["motorbike"]["body"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=False)
        #move_robot_to_pose(robot, component_positions["traj_motorbike"]["body"], ACCELERATION, VELOCITY)
    
        
    
        
        move_robot_to_pose(robot, initial_pose1, ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, initial_pose1, ACCELERATION, VELOCITY)
        





######################################################################plane########################       
    #vehicle_type ="PLANE" 
    if vehicle_type =="PLA":

        #  Move to left table and pick wheels_A
        #move_robot_to_pose(robot, component_positions["traj_plane"]["right_wing"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["plane"]["right_wing"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["plane"]["right_wing"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=True)
        # move_robot_to_pose(robot, component_positions["traj_plane"]["right_wing"], ACCELERATION, VELOCITY)

        #  Move to placing table and place at position 2
        move_robot_to_pose(robot, component_positions["traj_place"]["planeright"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["traj_place"]["planeright"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["planeright"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["planeright"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=False)
        move_robot_to_pose(robot, component_positions["traj_place"]["planeright"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["traj_place"]["planeright"], ACCELERATION, VELOCITY)
        
        #  Move to left table and pick wheels_B
        # move_robot_to_pose(robot, component_positions["traj_plane"]["left_wing"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["plane"]["left_wing"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["plane"]["left_wing"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=True)
        # move_robot_to_pose(robot, component_positions["traj_plane"]["left_wing"], ACCELERATION, VELOCITY)

        #  Move to placing table and place at position 2
        move_robot_to_pose(robot, component_positions["traj_place"]["planeleft"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["traj_place"]["planeleft"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["planeleft"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["planeleft"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=False)
        move_robot_to_pose(robot, component_positions["traj_place"]["planeleft"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["traj_place"]["planeleft"], ACCELERATION, VELOCITY)
        
        # #  Move to left table and pick body
        # move_robot_to_pose(robot, component_positions["traj_plane"]["wheels_B"], ACCELERATION, VELOCITY)
        # move_robot_to_pose(robot, component_positions["plane"]["wheels_B"], ACCELERATION, VELOCITY)
        # control_gripper(ROBOT_IP, close=True)
        # move_robot_to_pose(robot, component_positions["traj_plane"]["wheels_B"], ACCELERATION, VELOCITY)
        
        
        # #  Move to placing table and place at position 1
        # move_robot_to_pose(robot, component_positions["traj_place"]["planewheel"], ACCELERATION, VELOCITY)
        # move_robot_to_pose(robot, component_positions["place"]["planewheel"], ACCELERATION, VELOCITY)
        # control_gripper(ROBOT_IP, close=False)
        # move_robot_to_pose(robot, component_positions["traj_place"]["planewheel"], ACCELERATION, VELOCITY)

        

        # Move to right table and pick chasisi
        # move_robot_to_pose(robot, component_positions["traj_plane"]["body"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["plane"]["body"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["plane"]["body"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=True)
        # move_robot_to_pose(robot, component_positions["traj_plane"]["body"], ACCELERATION, VELOCITY)

        # Move to placing table and place at position 4
        move_robot_to_pose(robot, component_positions["traj_place"]["planebody"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["traj_place"]["planebody"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["planebody"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["planebody"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=False)
        move_robot_to_pose(robot, component_positions["traj_place"]["planebody"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["traj_place"]["planebody"], ACCELERATION, VELOCITY)

        
        # move_robot_to_pose(robot, initial_pose1, ACCELERATION, VELOCITY)
        # move_robot_to_pose(robot, initial_pose1, ACCELERATION, VELOCITY)

        print("plane assembly complete.")
        robot_busy=False 
        time.sleep(time_interval)


######reverse
        move_robot_to_pose(robot, component_positions["traj_place"]["planebody"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["traj_place"]["planebody"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["planebody"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["planebody"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=True)
        move_robot_to_pose(robot, component_positions["traj_place"]["planebody"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["traj_place"]["planebody"], ACCELERATION, VELOCITY)


        #move_robot_to_pose(robot, component_positions["traj_plane"]["body"] ,ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["plane"]["body"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["plane"]["body"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=False)
        #move_robot_to_pose(robot, component_positions["traj_plane"]["body"], ACCELERATION, VELOCITY)

        # move_robot_to_pose(robot, component_positions["traj_place"]["planewheel"], ACCELERATION, VELOCITY)
        # move_robot_to_pose(robot, component_positions["place"]["planewheel"], ACCELERATION, VELOCITY)
        # control_gripper(ROBOT_IP, close=True)
        # move_robot_to_pose(robot, component_positions["traj_place"]["planewheel"], ACCELERATION, VELOCITY)
                
        # move_robot_to_pose(robot, component_positions["traj_plane"]["wheels_B"], ACCELERATION, VELOCITY)  
        # move_robot_to_pose(robot, component_positions["plane"]["wheels_B"], ACCELERATION, VELOCITY)
        # control_gripper(ROBOT_IP, close=False)
        # move_robot_to_pose(robot, component_positions["traj_plane"]["wheels_B"], ACCELERATION, VELOCITY)
    
        move_robot_to_pose(robot, component_positions["traj_place"]["planeleft"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["traj_place"]["planeleft"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["planeleft"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["planeleft"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=True)
        move_robot_to_pose(robot, component_positions["traj_place"]["planeleft"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["traj_place"]["planeleft"], ACCELERATION, VELOCITY)
        
        # move_robot_to_pose(robot, component_positions["traj_plane"]["left_wing"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["plane"]["left_wing"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["plane"]["left_wing"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=False)
        #move_robot_to_pose(robot, component_positions["traj_plane"]["left_wing"], ACCELERATION, VELOCITY)
        
        move_robot_to_pose(robot, component_positions["traj_place"]["planeright"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["traj_place"]["planeright"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["planeright"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["planeright"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=True)
        move_robot_to_pose(robot, component_positions["traj_place"]["planeright"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["traj_place"]["planeright"], ACCELERATION, VELOCITY)
        
        #move_robot_to_pose(robot, component_positions["traj_plane"]["right_wing"] ,ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["plane"]["right_wing"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["plane"]["right_wing"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=False)
        #move_robot_to_pose(robot, component_positions["traj_plane"]["right_wing"], ACCELERATION, VELOCITY)
        
        move_robot_to_pose(robot, initial_pose1, ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, initial_pose1, ACCELERATION, VELOCITY)
        
        




    #vehicle_type ="SHIP"
    if vehicle_type =="SH":
        #  Move to left table and pick wheels_A
        #move_robot_to_pose(robot, component_positions["traj_ship"]["cabin"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["ship"]["cabin"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["ship"]["cabin"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=True)
        #move_robot_to_pose(robot, component_positions["traj_ship"]["cabin"], ACCELERATION, VELOCITY)

        #  Move to placing table and place at position 2
        move_robot_to_pose(robot, component_positions["traj_place"]["shipcabin"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["traj_place"]["shipcabin"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["shipcabin"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["shipcabin"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=False)
        move_robot_to_pose(robot, component_positions["traj_place"]["shipcabin"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["traj_place"]["shipcabin"], ACCELERATION, VELOCITY)
        
    
        
        #  Move to left table and pick body
        #move_robot_to_pose(robot, component_positions["traj_ship"]["deck"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["ship"]["deck"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["ship"]["deck"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=True)
        #move_robot_to_pose(robot, component_positions["traj_ship"]["deck"], ACCELERATION, VELOCITY)
        
        
        #  Move to placing table and place at position 1
        move_robot_to_pose(robot, component_positions["traj_place"]["shipdeck"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["traj_place"]["shipdeck"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["shipdeck"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["shipdeck"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=False)
        move_robot_to_pose(robot, component_positions["traj_place"]["shipdeck"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["traj_place"]["shipdeck"], ACCELERATION, VELOCITY)

        

        # Move to right table and pick chasisi
        #move_robot_to_pose(robot, component_positions["traj_ship"]["hull"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["ship"]["hull"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["ship"]["hull"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=True)
        #move_robot_to_pose(robot, component_positions["traj_ship"]["hull"], ACCELERATION, VELOCITY)

        # Move to placing table and place at position 4
        move_robot_to_pose(robot, component_positions["traj_place"]["shiphull"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["traj_place"]["shiphull"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["shiphull"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["shiphull"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=False)
        move_robot_to_pose(robot, component_positions["traj_place"]["shiphull"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["traj_place"]["shiphull"], ACCELERATION, VELOCITY)


        
        
        print("ship assembly complete.")
        robot_busy=False 
        time.sleep(time_interval)
        


######reverse
        move_robot_to_pose(robot, component_positions["traj_place"]["shiphull"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["traj_place"]["shiphull"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["shiphull"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["shiphull"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=True)
        move_robot_to_pose(robot, component_positions["traj_place"]["shiphull"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["traj_place"]["shiphull"], ACCELERATION, VELOCITY)

        #move_robot_to_pose(robot, component_positions["traj_ship"]["hull"] ,ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["ship"]["hull"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["ship"]["hull"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=False)
        #move_robot_to_pose(robot, component_positions["traj_ship"]["hull"], ACCELERATION, VELOCITY)

        move_robot_to_pose(robot, component_positions["traj_place"]["shipdeck"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["traj_place"]["shipdeck"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["shipdeck"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["shipdeck"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=True)
        move_robot_to_pose(robot, component_positions["traj_place"]["shipdeck"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["traj_place"]["shipdeck"], ACCELERATION, VELOCITY)
                
        #move_robot_to_pose(robot, component_positions["traj_ship"]["deck"], ACCELERATION, VELOCITY)  
        move_robot_to_pose(robot, component_positions["ship"]["deck"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["ship"]["deck"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=False)
        #move_robot_to_pose(robot, component_positions["traj_ship"]["deck"], ACCELERATION, VELOCITY)
    
        
        move_robot_to_pose(robot, component_positions["traj_place"]["shipcabin"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["traj_place"]["shipcabin"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["shipcabin"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["place"]["shipcabin"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=True)
        move_robot_to_pose(robot, component_positions["traj_place"]["shipcabin"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["traj_place"]["shipcabin"], ACCELERATION, VELOCITY)
        
        #move_robot_to_pose(robot, component_positions["traj_ship"]["cabin"] ,ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["ship"]["cabin"], ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, component_positions["ship"]["cabin"], ACCELERATION, VELOCITY)
        control_gripper(ROBOT_IP, close=False)
        #move_robot_to_pose(robot, component_positions["traj_ship"]["cabin"], ACCELERATION, VELOCITY)
        
        move_robot_to_pose(robot, initial_pose1, ACCELERATION, VELOCITY)
        move_robot_to_pose(robot, initial_pose1, ACCELERATION, VELOCITY)
        
            
       
    end_time=time.time()  
    print("execute time",end_time-start_time) 
    robot_overall_process_busy=False
    
        #vehicle_type ="CAR"
        
#execute_robot_sequence("SHIP")

# while (True):
#     keywords = ["CAR", "BUS", "MOTORBIKE", "PLANE", "SHIP"]
    
#     random_word = random.choice(keywords)
#     print(random_word)
#     time.sleep(2)
#     execute_robot_sequence(random_word)

# #robot_busy=False
# #initilize_all()


# Flask endpoint to receive audio file
@app.route('/act', methods=['POST'])
def classify_audio():
    if 'file' not in request.files:
        return jsonify({"result": None, "error": "No file provided"}), 400

    file = request.files['file']
    if file.filename == '':
        return jsonify({"result": None, "error": "Empty filename"}), 400

    # Save the audio file
    file_path = os.path.join(app.config['UPLOAD_FOLDER'], file.filename)
    file.save(file_path)

    # Process audio and get the detected keyword
    detected_keyword = process_audio(file_path)
    os.remove(file_path)

    if detected_keyword:
        if not robot_busy and not robot_overall_process_busy:
            # Run robot sequence in a separate thread to prevent blocking
            #threading.Thread(target=initilize_all).1start()
            #threading.Thread(target=test).start()
            threading.Thread(target=execute_robot_sequence, args=(detected_keyword,)).start()
            # detected_keyword="CAR"
            detected_keyword=vehicle_output
            #threading.Thread(target=test, args=(detected_keyword,)).start()
            return jsonify({"result": detected_keyword, "message": f"{detected_keyword.capitalize()} assembly started."})
        else:
            return jsonify({"result": None, "error": "Robot is currently busy."}), 400
    else:
        return jsonify({"result": None, "error": "No keyword detected."}), 400


# API to check robot's busy state
@app.route('/state', methods=['GET'])
def check_robot_state():
    global robot_busy
    return jsonify({"busy": robot_busy})


if __name__ == '__main__':
        
    app.run(host='0.0.0.0', port=5000, debug=False)
