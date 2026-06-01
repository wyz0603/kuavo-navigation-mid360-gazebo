import subprocess
import os

script_dir = os.path.dirname(os.path.abspath(__file__))

def main():
    try:
        subprocess.run(["python3", script_dir + "/ros_service.py"], check=True)
        # Debug: pick basket and putdowon basket
        # subprocess.run(["python3", script_dir + "/pick_basket.py"], check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error running ROS node: {e}")

if __name__ == "__main__":
    main()
