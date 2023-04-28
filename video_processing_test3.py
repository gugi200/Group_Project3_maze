from cv2 import (cvtColor, GaussianBlur, Canny, COLOR_RGB2GRAY, imshow, 
threshold, findContours, THRESH_BINARY, RETR_EXTERNAL, CHAIN_APPROX_NONE, 
rectangle, resize, waitKey, destroyAllWindows, VideoCapture, arrowedLine, 
CHAIN_APPROX_SIMPLE, inRange,contourArea, drawContours, COLOR_BGR2HSV, FILLED, boundingRect)
import cv2
import numpy as np
import time
import logging
import multiprocessing
import threading
from scipy import ndimage
from periphery import PWM
import wiringpi
import servo
from synDef import *
from numba import jit

               


class ProcessImage():

    # test the blue thereshold
    # and the ROI delta and mask delta
    
    def __init__(self):

        
        # intialize video processing prameters
        self.contour_cond = 0
        self.contour_ok = 0
        self.nearest_idx_maze = [0, 0]
        self.xball_cen = 0
        self.yball_cen = 0
        self.ball_coo = (0, 0, 0, 0)
        self.prevBall = [0, 0]
        self.accuracyMode = 1
        # self.counter = 0

        
        # initialize servos parameters
        self.servos = servo.servo()
        self.currentAngle1 = 90
        self.currentAngle2 = 90

        # vector interation offset
        self.vectIter = 0
        
        self.count = 0
        self.index = 0
        self.coords = [0,0]

    def getCanny(self, image):
        '''get black and white image displaying edges'''
        logging.debug("[getCanny]   Start")
        #masks a box onto the maze (currently at wrong coords)
        #image[MASK_WALL[1]:MASK_WALL[3], MASK_WALL[0]:MASK_WALL[2]] = np.ones(
        #        ((MASK_WALL[3]-MASK_WALL[1]), MASK_WALL[2]-MASK_WALL[0], 3)).astype(np.uint8)
        gray_image = cvtColor(image, COLOR_RGB2GRAY)
        blur_image = GaussianBlur(gray_image, (5, 5), 0)
        canny_image = Canny(blur_image, 50, 150)
        #imshow("masked area of pre-canny", canny_image)
        logging.debug("[getCanny]   End")
        return canny_image

    def getLeadLine(self, cannyImg, lineFrame):
        '''find the longest contour with respect to the largest are it encloses'''
        logging.debug("[getLeadLine]    Start")
        frame = np.copy(lineFrame)
        _, thresh = threshold(cannyImg, 128, 255, THRESH_BINARY)
        contours, _ = findContours(thresh, RETR_EXTERNAL, CHAIN_APPROX_NONE)
        # self.counter = self.counter + 1
        self.contours = max(contours, key=contourArea)
        self.showLeadLine(lineFrame)
        #if self.counter == 3:
        self.contour_ok = int(input("0 for not ok, 1 for ok: "))

        if self.contour_ok:
            logging.info("[getLeadLine]    The cotour has been accepted")
            self.contour_img = np.zeros((lineFrame.shape[0], lineFrame.shape[1]))
            for pixel in self.contours:    
                self.contour_img[pixel[0][1], pixel[0][0]] = 1
            self.contour_img_mask = np.ones_like(lineFrame)*255
            self.ogContourImg = np.copy(self.contour_img)
            self.getRollingWindow(frame)
            
            #self.ballThread = Thread(target=self.threadBallDetect, args=())
            self.maskContour = np.ones((self.contour_img.shape[0], self.contour_img.shape[1])).astype(np.uint8)
            
            
        logging.debug("[getLeadLine]    End")    

    def showLeadLine(self, frame):
        '''show the the area the longest contour encloses'''
        drawContours(frame, [self.contours], -1, (0, 0, 255), FILLED)
        imshow('Lead line - longest line' ,frame)
        
    def getRollingWindow(self, frame):
        logging.debug("[getRollingWindow]    Start")
        
        
        while not self.ball_coo[3] and not self.ball_coo[1]:
            self.ball_coo = ProcessImage.detectBall(frame)
        x_pos = self.ball_coo[0]
        y_pos = self.ball_coo[1]
        line = np.copy(self.ogContourImg)
        
        nodeList = []
        
        while True:
            x0 = x_pos - MASK_WINDOW
            y0 = y_pos - MASK_WINDOW
            x1 = x_pos + MASK_WINDOW        
            y1 = y_pos + MASK_WINDOW
            if x0<0:
                x0=0
            if x1>self.contour_img.shape[1]:
                x1 = self.contour_img.shape[1] - 1
            if y0<0:
                y0=0
            if y1>self.contour_img.shape[0]:
                y1 = self.contour_img.shape[0] - 1
            line[y0:y1, x0:x1] = 0
            
            lineROI, x0_ROI, y0_ROI = ProcessImage.jitROI(line, ROI_LINE, x0, y0, x1, y1)
            nonzero = np.argwhere(lineROI==1)
            if len(nonzero)==0:
                self.nodes = np.array(nodeList).T
                logging.info("Nodes along the lead line has been found, num: %i", 
                             len(self.nodes[0]))
                #imshow('Vector and masked', frame)
                logging.debug("[getRollingWindow]    End")

                return 
            
            distances = np.sqrt(
                np.square(nonzero[:,0] - y_pos + y0_ROI) \
                    + np.square(nonzero[:,1] - x_pos + x0_ROI))
            nearest_idx = nonzero[np.argmin(distances)]
            frame = arrowedLine(frame, (x_pos, y_pos), 
                                (nearest_idx[1]+ x0_ROI, nearest_idx[0] + y0_ROI),
                                (0, 255, 0), 2) 

            x_pos = nearest_idx[1] + x0_ROI
            y_pos = nearest_idx[0] + y0_ROI
            #print(x_pos, y_pos)

            nodeList.append([y_pos, x_pos])
        
    @staticmethod
    @jit(nopython=True)   
    def jitROI(image, delta, x0=0, y0=0, x1=0, y1=0):
            x0_ROI = x0 - delta
            x1_ROI = x1 + delta
            y0_ROI = y0 - delta
            y1_ROI = y1 + delta

            if x0_ROI<0:
                x0_ROI=0
            if x1_ROI>image.shape[1]:
                x1_ROI = image.shape[1] - 1
            if y0_ROI<0:
                y0_ROI=0
            if y1_ROI>image.shape[0]:
                y1_ROI = image.shape[0] - 1
            return image[y0_ROI:y1_ROI, x0_ROI:x1_ROI], x0_ROI, y0_ROI 
 
            
    @staticmethod
    def detectBall(image, debug=False):
        '''find a blue object of specific dimensions and area'''
        logging.debug("[detectBall]    Start")
        image = np.copy(image)
        x_c, y_c, w_c, h_c = [0, 0, 0, 0]
        
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, BLUE_LOWER, BLUE_UPPER)
        #if debug:
            #cv2.imshow('DEBUG blue colour', mask)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) != 0:
            for contour in contours:
                x, y, w, h = cv2.boundingRect(contour)
                area = cv2.contourArea(contour)
                if ((area > BALL_AREA_MIN) and (w<BALL_WIDTH_MAX) and (h<BALL_HEIGHT_MAX)):
                    x_c = x
                    y_c = y
                    w_c = w
                    h_c = h
                    logging.info("[detectBall]    The global coordinates of the ball: (%s, %s), (%s, %s), area: %i" , x, y, x+w, y+w, area)
                else:
                    logging.debug("[detectBall]    The ball hasn't been detected")
            logging.debug("[detectBall]    Exiting")
            return (x_c, y_c, w_c, h_c)

        else:
            return (0, 0)

        


    def showBallVector(self, frame, ball_coo, vector_coo):
        '''show the image with a vector'''

        frame = cv2.rectangle(frame, (ball_coo[0],ball_coo[1]), 
                          (ball_coo[0]+ball_coo[2],ball_coo[1]+ball_coo[3]), 
                          (255, 0, 0),  1)
        frame = cv2.arrowedLine(frame, (self.xball_cen, self.yball_cen), 
                                (self.xball_cen+vector_coo[0], self.yball_cen+vector_coo[1]),
                                (0, 255, 0), 3)
        return frame
        



      

    def testPID(self, frame):
    #limits frame size of whole maze. Currently not working very well when detecting line
        #frame, _, _ = ProcessImage.jitROI(frame, 0,
        #                                        MAZE_ROI[0], MAZE_ROI[1], 
        #                                        MAZE_ROI[2], MAZE_ROI[3]) 
        frame1 = np.copy(frame)
        cannyImg = self.getCanny(frame1)
        if self.contour_ok==0:    
            lineFrame = np.copy(frame)
            self.getLeadLine(cannyImg, lineFrame)
        if not self.contour_ok:
            return
        #print(self.nodes)
        
        if (self.count%LOOPS_PER_POINT == 0) or (self.count==0): #divisible by 60 i.e. 60 loops have passed
            reset = 1 #tells PID it's the first loop of the controller and vector is pointing at a new coordinate for first time
            self.index = self.index + 1 #increment to next coordinate in list
            #print(self.index)
            #print(self.nodes.shape)
            self.coords[0] = self.nodes[1][self.index] #increment to next coordinate in list
            self.coords[1] = self.nodes[0][self.index]
            #print(self.count, self.coords) 
        #elif self.count%20==0:
        #    reset = 4
        else:
            reset = 0
            
        self.count = self.count + 1
        
        
        ball_coo = ProcessImage.detectBall(frame, debug=False)

        if not ball_coo[0]:
            cv2.imshow("dfeg", frame)
            return 0
        self.xball_cen = int(ball_coo[0]+(ball_coo[2]/2))
        self.yball_cen = int(ball_coo[1]+(ball_coo[3]/2))
        
        #To prevent 
        if reset == 0 and (self.xball_cen<88 or self.xball_cen>507):
            reset = 2 #this disables the x axis derivative gain only
        elif reset == 0 and (self.yball_cen<35 or self.yball_cen>395):
            reset = 3
        
        finalAngle1, finalAngle2 =self.servos.PositionPDController(
                        self.coords[0]-self.xball_cen, self.coords[1]-self.yball_cen, reset)
                    
        try:
            self.runServo.terminate()
            
        except AttributeError:
            pass
        self.runServo = multiprocessing.Process(
            target=self.servos.moveBothServos, args=(
                finalAngle1, finalAngle2))                  
        self.runServo.start()
        frame = self.showBallVector(frame, ball_coo, (self.coords[0]-self.xball_cen, self.coords[1]-self.yball_cen))
        cv2.imshow("Vector and Ball", frame)
        #return frame
        


    


# get rid of the remeber if goes black for the windowing code          
            
    
def live_feedV2():
    vid = cv2.VideoCapture(0)
    #servos = servo.servo()
    image = ProcessImage()
    condition = True
    current_t = 0 
    while(condition):
        condition = int(input('give 0: '))
    timestamp_start = time.time()

    count = 0
    index = 0
    while(True):
        # Capture the video frame
        # by frame
        if count%110 == 0: #divisible by 100 i.e. 100 loops have passed
            index = index + 1
        #coords = path_coords[index]
        ret, frame = vid.read()
        if current_t>=3:
            image.testPID(frame)
        else:
            current_t = time.time()-timestamp_start
            # the 'q' button is set as the
        # quitting button you may use any
        # desired button of your choice
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
        count = count + 1
        
    # After the loop release the cap object
    vid.release()
    # Destroy all the windows
    cv2.destroyAllWindows()

live_feedV2()
