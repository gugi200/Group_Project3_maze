
import numpy as np
import logging

# hole detection
HOLE_WIDTH_MIN = 15
HOLE_HEIGHT_MIN = 15
HOLE_WIDTH_MAX = 40
HOLE_HEIGHT_MAX = 40
HOLE_AREA_MIN = 225
ROI_HOLES =40
ANTI_VECT = 1.5

# ball detection 
BALL_WIDTH_MAX = 60
BALL_HEIGHT_MAX = 60
BALL_AREA_MIN = 150
BLUE_LOWER = np.array([93,150,20])#([100, 150, 0])	[93,178,35]   [107,218,35]
BLUE_UPPER = np.array([138,255,168])#([140,255,178])	[138,255,168] [112,255,168]
RED_LOWER = np.array([100, 150, 0])
RED_UPPER = np.array([140,255,255])
ROI_VECT = 65
HOLES_VECT = 10
VECT_CAP = 4

# lead line detection
#MAZE_ROI = [68-10, 30-30, 513+45, 406+20]# x0, y0, x1, y1
MAZE_ROI = [68-10, 30-20, 513+45, 406+20]# x0, y0, x1, y1 please use these values for hard_code_feedback
MASK_WALL = [327-5, 167, 327-5+40, 167+40] # x, y, w, h


# other
MASK_DELTA = 50 # masks the ball in the window
MASK_WINDOW = 23 #was 8 initial masking of the starting position to fiond the line. Reduce to increase number of nodes on the line
ROI_LINE = MASK_WINDOW + 20 # how far the ROI gets to to find the lead line
VECT_ITER_OFFSET_GAIN = 12 # num of ieteration when stable before applying gain
STUCK_PIXELS = 10 # delta of stuck pixels
VIS_WINDOW_SIZE = 7 # number of visible window nodes
NODE_WINDOW= 12 # how big a window of a signle node is (pixels)
HIGH_ACC_TH = 60 # threshold below which the masking is smaller
MEDIUM_ACC_TH = 70

# servo delay
NUM_DELTA = 4
SOFT_DELAY_MS = 50/NUM_DELTA

# PID
KPX = 0.25 #was 0.95
KPY = 0.4 #was 1.2
TDX = 5 #was 0.3 #experiment with increasing further with no integral gain
TDY = 5 #was 0.3
TIX = 30 #60 #just this x value as 25 got it to hole 12 on medium maze
TIY = 60
JOLT_IMPULSE = 3.3 #for slow and steady method. multiplier for amount of additional proportional gain applied on first loop of a new point to get ball rolling
SMALL_JOLT = 1.8
LOOPS_PER_POINT = 100 #was 15 for slow and steady method. Number of loops spent getting ball to a point with PID

# video_processing_test4 parameters (ball must get to node)
MAX_DISTANCE_WHEN_BALL_ON_NODE = 10
LOOPS_WHILE_BALL_ON_NODE = 8
LOOPS_PER_JOLT = 40

VECT_ITER_OFFSET = 1

FLICK_OFFSET = 80

#VECT_ITER_OFFEST_GAIN
VECT_ITER_OFFEST_GAIN = 12


# initialize logging
logging.basicConfig(format='%(asctime)s - %(message)s', level=logging.ERROR)
logging.info("Process Image initialized")
