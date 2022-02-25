#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2 as cv
import yaml

# Load parameters
with open('../config/marker.yaml') as f:
    par = yaml.load(f, yaml.loader.SafeLoader)
squares_x = par['squares_x']
squares_y = par['squares_y']
marker_length = int(1000*par['marker_length'])
square_length = int(1000*par['square_length'])

# Generate marker board
attr = getattr(cv.aruco, par['dictionary'])
dictionary = cv.aruco.Dictionary_get(attr)
board = cv.aruco.CharucoBoard_create(squares_x, squares_y,
                                     square_length, marker_length,
                                     dictionary)

# Write marker board to file
imboard = board.draw((squares_x*square_length, squares_y*square_length))
cv.imwrite("image.png", imboard)