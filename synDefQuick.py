
import numpy as np
import logging

# hole detection
HOLE_WIDTH_MIN = 20
HOLE_HEIGHT_MIN = 20
HOLE_WIDTH_MAX = 40
HOLE_HEIGHT_MAX = 40
HOLE_AREA_MIN = 300
ROI_HOLES =35 # was 40
ANTI_VECT = 1.5

# ball detection 
BALL_WIDTH_MAX = 60
BALL_HEIGHT_MAX = 60
BALL_AREA_MIN = 200
BLUE_LOWER = np.array([100, 150, 0])
BLUE_UPPER = np.array([140,255,255])
RED_LOWER = np.array([100, 150, 0])
RED_UPPER = np.array([140,255,255])
ROI_VECT = 65
HOLES_VECT = 10
VECT_CAP = 4

# lead line detection
MAZE_ROI = [68-2, 30-30, 513+40, 406+10]# x0, y0, x1, y1
MASK_WALL = [327-15, 167+5, 327+45, 167+55] # x, y, w, h


# other
MASK_DELTA = 50 # masks the ball in the window
MASK_WINDOW = 15 # initial masking of the starting position to fiond the line
ROI_LINE = MASK_WINDOW + 20 # how far the ROI gets to to find the lead line
VECT_ITER_OFFSET_GAIN = 12 # num of ieteration when stable before applying gain
STUCK_PIXELS = 15 # delta of stuck pixels
VIS_WINDOW_SIZE = 7 # number of visible window nodes
NODE_WINDOW= 12 # how big a window of a signle node is (pixels)
HIGH_ACC_TH = 60 # threshold below which the masking is smaller
MEDIUM_ACC_TH = 70
COMPLETE_TH = 20
WINDOW_OFFSET = 1

# servo delay
NUM_DELTA = 4
SOFT_DELAY_MS = 50/NUM_DELTA

# PID
KPX = 0.90 #was 0.9
KPY = 1.35 #was 1.2
TDX = 0 #was 0 #experiment with increasing further with no integral gain
TDY = 0 #was 0
TIX = 10000
TIY = 10000
VECT_ITER_OFFSET = 1

FLICK_OFFSET = 10

#VECT_ITER_OFFEST_GAIN
VECT_ITER_OFFEST_GAIN = 12


# initialize logging
logging.basicConfig(format='%(asctime)s - %(message)s', level=logging.INFO)
logging.info("Process Image initialized")

