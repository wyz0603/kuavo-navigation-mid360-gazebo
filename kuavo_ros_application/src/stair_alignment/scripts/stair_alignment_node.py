#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import subprocess
import os

script_dir = os.path.dirname(os.path.abspath(__file__))

def main():
    try:
        subprocess.run(["python3", script_dir + "/stair_alignment_service.py"], check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error running ROS stair alignment node: {e}")

if __name__ == "__main__":
    main()