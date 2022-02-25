#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import transforms3d as t3d

try:
    with open("transformation.txt", "r") as f:
        R = np.zeros((3,3))     # Rotation matrix
        t = np.zeros(3)         # Translation vector
        
        # Read data
        for i in range(3):
            line = f.readline().split("\t")
            R[i,0] = float(line[0])
            R[i,1] = float(line[1])
            R[i,2] = float(line[2])
            t[i]   = float(line[3]) / 1000.0
        
        # Convert R to roll-pitch-yaw
        euler = t3d.euler.mat2euler(R)
        
        # Output result
        print("<origin xyz=\"%.7f %.7f %.7f\" rpy=\"%.9f %.9f %.9f\"/>" %
              (t[0], t[1], t[2], euler[0], euler[1], euler[2]))
        
except:
    print("Cannot find file 'transformation.txt'")
