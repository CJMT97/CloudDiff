import os
import serial
import sys
import time
import subprocess
import socket
import signal
import tkinter as tk
import gphoto2 as gp
from tkinter import filedialog
from tkinter import messagebox

ser = serial.Serial(
    port='/dev/ttyCH341USB0',         
    baudrate=115200,
    bytesize=serial.EIGHTBITS,     
    parity=serial.PARITY_NONE,     
    stopbits=serial.STOPBITS_ONE,  
    xonxoff=False,                 
    rtscts=False,                 
    dsrdtr=False                   
)

def is_port_free(port):
    """
    Verifies that port is free for use.
    Probably not needed anymore, but keep just in case.
    """
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        return s.connect_ex(('localhost', port)) != 0
    
def sanitize_model(model):
    """
    Create camera names to use in naming files
    Useful for notekeeping purposes, but probably not 
    vital for photogrammetry methods.
    """
    return (model.strip()
            .replace(" ", "_")
            .replace("/", "-")
            .replace("\\", "-")
            .replace(":", "")
            [:30]) 

# ================= SET UP BASICS =========================
# This stage ensures that cameras and turntable are ready for operation.

#Unmount cameras for gphoto2 access. Cameras are mounted by default when switched on.
subprocess.run("gio mount -s gphoto2", shell=True, preexec_fn=os.setsid)

#Get cameras and store them
portsList = gp.PortInfoList()
portsList.load()
abilitiesList = gp.CameraAbilitiesList()
abilitiesList.load()

cams = abilitiesList.detect(portsList)

print(cams)


# ================= HANDLE LIVE VIEW =========================


# Launch video streams and viewers
processes = []

for idx, (model, port) in enumerate(cams):
    http_port = 8080 + idx
    
    # 1. Enable viewfinder for Nikon cameras. 
    # This code has only been tested on Nikon and Canon cameras.
    # Other models might have other such requirements.
    subprocess.run(
        f"gphoto2 --port {port} --set-config /main/actions/viewfinder=1",
        shell=True,
        check=True
    )
    
    # 2. Add short delay for camera to initialize live view
    # Nikon cameras take some time to initialize sadly.
    time.sleep(2.5 if "Nikon" in model else 0.5)
    
    # 3. Modified streaming command with Nikon compatibility flags
    ffmpeg_cmd = (
        f"gphoto2 --port {port} --capture-movie --stdout "
        f"| ffmpeg -re -f mjpeg -i - -c:v copy -f mpjpeg "  # Maintain original stream
        f"-listen 1 http://localhost:{http_port}/cam{idx}"
    )

    p_ffmpeg = subprocess.Popen(
        ffmpeg_cmd,
        shell=True,
        preexec_fn=os.setsid,
        stderr=subprocess.PIPE  
    )
    processes.append(p_ffmpeg)
    
    # 4. Extended delay before VLC launch for slow cameras (NIKON...)
    time.sleep(3 if "Nikon" in model else 1)
    
    # 5. VLC with hardware decoding
    vlc_cmd = (
        f"vlc --demux=mjpeg --network-caching=300 "
        f"http://localhost:{http_port}/cam{idx}" 
    )

    p_vlc = subprocess.Popen(vlc_cmd, shell=True, preexec_fn=os.setsid)
    processes.append(p_vlc)

# User confirmation dialog
root = tk.Tk()
root.withdraw()
confirmed = messagebox.askyesno("Setup", "Cameras ready? Click Yes to continue.")

# Cleanup if user cancels
if not confirmed:
    for proc in processes:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    sys.exit("Setup aborted")

# Terminate preview streams
for proc in processes:
    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)

# ================= CAPTURE IMAGES =========================

# Choose a folder to save images to
root = tk.Tk()
root.withdraw()  # Hide main window
base_capture_dir = filedialog.askdirectory(
    title="Select folder to save captures"
)
if not base_capture_dir:
    print("No folder selected. Exiting.")
    sys.exit(0)

print(f"Captures will be saved in: {base_capture_dir}")

# Make directories to store photos from each camera inside selected folder
camera_dirs = {}
for model, port in cams:
    safe_name = sanitize_model(model)
    cam_dir = os.path.join(base_capture_dir, safe_name)
    os.makedirs(cam_dir, exist_ok=True)
    camera_dirs[port] = cam_dir
    print(f"Created directory for {model}: {cam_dir}")
    
print("Live preview setup complete. Proceed with capture sequence.")

print("Enter number of angles (6 minimum to work): ")
angles = int(input())
angToPrint = 360.0/angles
command = 'CT+TRUNSINGLE(1,' + str(angToPrint) + ');'
commandToSend = command.encode('utf-8')

count = 0

for _ in range(angles):
    print(f"\n=== Starting rotation {count} ===")
    
    # Send rotation instruction
    ser.write(commandToSend)
    time.sleep(7)  # Delay to complete rotation. The more angles, the lower this can be. 
    
    # Capture from all cameras
    for model, port in cams:
        safe_name = sanitize_model(model)
        filename = f"{safe_name}_angle-{count:02d}.jpg"
        full_path = os.path.join(camera_dirs[port], filename)
        
        print(f"Capturing from {model}...")
        subprocess.run([
            "gphoto2",
            "--port", port,
            "--capture-image-and-download",
            "--filename", full_path
        ], check=True)
        time.sleep(1)  # Pause shortly to handle file operations
    
    count += 1
    time.sleep(3)  # Delay to handle unexpected things. Unnecessary probably, but not hurting anything.

ser.close()
