import subprocess
import os

def launch_isaac_sim():
    # Get the username from environment variable
    username = os.environ.get('USER', 'defaultuser')
    print(f"username: {username}")
    
    # Define the target directory and isaac sim path
    target_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    target_dir = os.path.join(target_dir, "nio-isaac")
    isaac_python = f"/home/{username}/.local/share/ov/pkg/isaac-sim-4.1.0/python.sh"
    
    print(f"target_dir: {target_dir}")
    
    try:
        # Change to the target directory
        os.chdir(target_dir)
        print(f"Changed directory to: {target_dir}")
        
        # Set up environment variables
        env = os.environ.copy()
        env['PYTHONPATH'] = f"{target_dir}:{env.get('PYTHONPATH', '')}"
        
        # Run the isaac sim command with bash explicitly
        command = f"bash -c '{isaac_python} nio/main.py'"
        print(f"Executing command: {command}")
        print(f"PYTHONPATH: {env['PYTHONPATH']}")
        
        process = subprocess.Popen(
            command, 
            shell=True, 
            executable='/bin/bash',
            env=env
        )
        print("Started Isaac Sim application")
        
        # Wait for the process to complete
        process.wait()
        
    except FileNotFoundError:
        print(f"Error: Directory {target_dir} or {isaac_python} not found")
    except subprocess.CalledProcessError as e:
        print(f"Error running Isaac Sim: {e}")
    except Exception as e:
        print(f"Unexpected error: {e}")

if __name__ == "__main__":
    launch_isaac_sim() 