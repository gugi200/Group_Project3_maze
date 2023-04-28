from cv2 import (cvtColor, GaussianBlur, Canny, COLOR_RGB2GRAY, imshow, 
threshold, findContours, THRESH_BINARY, RETR_EXTERNAL, CHAIN_APPROX_NONE, 
rectangle, resize, waitKey, destroyAllWindows, VideoCapture, arrowedLine, 
CHAIN_APPROX_SIMPLE, inRange,contourArea, drawContours, COLOR_BGR2HSV, FILLED, boundingRect)
import numpy as np
import time
import logging
from multiprocessing import Process, active_children
from threading import Thread
from numba import jit
from scipy import ndimage
from periphery import PWM
import wiringpi
import servo
from synDefQuickMed import *

import sys            


class ProcessImage():

    # test the blue thereshold
    # and the ROI delta and mask delta
    
    def __init__(self):

        
        # intialize video processing prameters
        
        # parameters for lead line
        self.contour_cond = 0
        self.contour_ok = 0
        self.nearest_idx_maze = np.array([0, 0])
        
        # ball detection
        self.xball_cen = 0
        self.yball_cen = 0
        self.ball_coo = (0, 0, 0, 0)
        self.accuracyMode = 1
        self.slowMode = 0
        self.prevBall = [0, 0]
        self.flickIter = 0

        
        # initialize servos parameters
        self.servos = servo.servo()
        self.currentAngle1 = 90
        self.currentAngle2 = 90


        # vector interation offset
        self.vectIter = 0
        

    
    def getCanny(self, image):
        '''get black and white image displaying edges'''
        logging.debug("[getCanny]   Start")
        image[MASK_WALL[1]:MASK_WALL[3], MASK_WALL[0]:MASK_WALL[2]] = np.zeros(
                ((MASK_WALL[3]-MASK_WALL[1]), MASK_WALL[2]-MASK_WALL[0], 3)).astype(np.uint8)
        gray_image = cvtColor(image, COLOR_RGB2GRAY)
        blur_image = GaussianBlur(gray_image, (5, 5), 0)
        canny_image = Canny(blur_image, 50, 150)
        #imshow("masked area of pre-canny", canny_image)
        logging.debug("[getCanny]   End")
        return canny_image
        
        
    def getROI(self, image, delta, x0=0, y0=0, x1=0, y1=0):
        '''get Region Of Interest for a given image and dimensions. 
        Returns ROI image and the coo of the start'''
        logging.debug("[getROI]   Start")
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
        logging.debug("[getROI]   End")
        return image[y0_ROI:y1_ROI, x0_ROI:x1_ROI], x0_ROI, y0_ROI
    
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
                

    def getLeadLine(self, cannyImg, lineFrame):
        '''find the longest contour with respect to the largest are it encloses'''
        logging.debug("[getLeadLine]    Start")
        frame = np.copy(lineFrame)
        _, thresh = threshold(cannyImg, 128, 255, THRESH_BINARY)
        contours, _ = findContours(thresh, RETR_EXTERNAL, CHAIN_APPROX_NONE)
        self.contours = max(contours, key=contourArea)
        self.showLeadLine(lineFrame)
        
        # self.contour_ok = int(input("0 for not ok, 1 for ok: "))
        
        self.contour_ok = 1

        if self.contour_ok:
            logging.info("[getLeadLine]    The cotour has been accepted")
            self.contour_img = np.zeros((lineFrame.shape[0], lineFrame.shape[1]))
            for pixel in self.contours:    
                self.contour_img[pixel[0][1], pixel[0][0]] = 1
            self.contour_img_mask = np.ones_like(lineFrame)*255
            self.ogContourImg = np.copy(self.contour_img)
            if not self.getRollingWindow(frame):
                self.restart = 1
                return 0 
            self.maskContour = np.ones((self.contour_img.shape[0], self.contour_img.shape[1])).astype(np.uint8)
            self.restart = 0
            return 1
            
        logging.debug("[getLeadLine]    End")
        
            
    def showLeadLine(self, frame):
        '''show the the area the longest contour encloses'''
        drawContours(frame, [self.contours], -1, (0, 0, 255), FILLED)
        imshow('Lead line - longest line' ,frame)
        
        
    def detectHoles(self, frame, cannyImg):
        
        _, thresh = threshold(cannyImg, 128, 255, THRESH_BINARY)
        contours, _ = findContours(thresh, RETR_EXTERNAL, CHAIN_APPROX_NONE)
        if len(contours) !=0:
            holeContour = np.zeros(cannyImg.shape)
            holeCount = 0
            for contour in contours:
                x, y, w, h = boundingRect(contour)
                area = contourArea(contour)
                if ((area > HOLE_AREA_MIN) and (HOLE_WIDTH_MIN<w<HOLE_WIDTH_MAX) and (HOLE_HEIGHT_MIN<h<HOLE_HEIGHT_MAX)):
                    drawContours(frame, [contour], -1, (0, 255, 0), FILLED)
                    logging.debug("[detectHoles]    The coordinates of the holes: (%s, %s), (%s, %s), area: %i", x, y, x+w, y+h, area)
                    idx = contour.reshape(-1,2).T
                    holeContour[y + int(h/2), x + int(w/2)] = 1
                   
                    holeCount += 1
            logging.info("[detectHoles]    Number of holes detected: %i", holeCount)
            imshow("holes", frame)
            #print("hole count:",holeCount)
            return holeContour
        return np.ones([1, 1])
        
    def flick(self):
         ''''''
         if self.flickIter%FLICK_OFFSET==0:
             if (self.prevBall[0]-STUCK_PIXELS<self.ball_coo[0]<self.prevBall[0]+STUCK_PIXELS) and (self.prevBall[1]-STUCK_PIXELS<self.ball_coo[1]<self.prevBall[1]+STUCK_PIXELS):
                 self.prevBall[0] = self.ball_coo[0]
                 self.prevBall[1] = self.ball_coo[1]
                 try:
                     self.runServo.terminate()
                
                 except AttributeError:
                     logging.warning("[processWindowImg]   Threading error") 

                 self.runServo = Process(
                    target=self.servos.moveBothServos, args=(
                    10, 70))        
              
                 self.runServo.start()
                 logging.error("SCANDI FLICK")
                 self.flickIter += 1
                 return 1
                 
             else:
                 self.prevBall[0] = self.ball_coo[0]
                 self.prevBall[1] = self.ball_coo[1]
                 
         self.flickIter += 1
         return 0
     
     
    @staticmethod
    @jit(nopython=True)    
    def dodgeHoles(holeContour, xball_cen, yball_cen, ball_coo):

        x0_ROI = ball_coo[0] - ROI_HOLES
        x1_ROI = ball_coo[0]+ball_coo[2] + ROI_HOLES
        y0_ROI = ball_coo[1] - ROI_HOLES
        y1_ROI = ball_coo[1]+ball_coo[3] + ROI_HOLES
        
        if x0_ROI<0:
            x0_ROI=0
        if x1_ROI>holeContour.shape[1]:
            x1_ROI = holeContour.shape[1] - 1
        if y0_ROI<0:
            y0_ROI=0
        if y1_ROI>holeContour.shape[0]:
            y1_ROI = holeContour.shape[0] - 1
        holesROI = holeContour[y0_ROI:y1_ROI, x0_ROI:x1_ROI]
        nonzero = np.argwhere(holesROI==1)
        if len(nonzero):
            nearest_idx = nonzero[0]
            ToHoleY  = yball_cen - nearest_idx[0] - y0_ROI
            ToHoleX  = xball_cen - nearest_idx[1] - x0_ROI

            return np.sign(ToHoleX)*25, np.sign(ToHoleY)*25
        else:
            return 0, 0     
        
   
    def threadBallDetect(self):
        while True:
            '''find a blue object of specific dimensions and area'''
            logging.debug("[detectBall]    Start")
            image = self.ballFrame
    
            self.ball_coo = ProcessImage.detectBall(image)
            
            if not self.ball_coo[3]:
                continue
            self.xball_cen = int(self.ball_coo[0] + (self.ball_coo[2]/2))
            self.yball_cen = int(self.ball_coo[1] + (self.ball_coo[3]/2))

            
            
    @staticmethod
    def detectBall(image, debug=False):
        '''find a blue object of specific dimensions and area'''
        logging.debug("[detectBall]    Start")
        image = np.copy(image)
        x_c, y_c, w_c, h_c = [0, 0, 0, 0]
        
        hsv = cvtColor(image, COLOR_BGR2HSV)
        mask = inRange(hsv, BLUE_LOWER, BLUE_UPPER)
        if debug:
            imshow('DEBUG blue colour', mask)
        contours, _ = findContours(
            mask, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE)
        if len(contours) != 0:
            for contour in contours:
                x, y, w, h = boundingRect(contour)
                area = contourArea(contour)
                if ((area > BALL_AREA_MIN) and (w<BALL_WIDTH_MAX) and (h<BALL_HEIGHT_MAX)):
                    x_c = x
                    y_c = y
                    w_c = w
                    h_c = h
                    logging.warning("[detectBall]    The global coordinates of the ball: (%s, %s), (%s, %s), area: %i" , x, y, x+w, y+w, area)
                else:
                    logging.debug("[detectBall]    The ball hasn't been detected")
            logging.debug("[detectBall]    Exiting")
            return (x_c, y_c, w_c, h_c)

        else:
            return (0, 0, 0, 0)        
            

    
    def maskRoute(self):
        '''mask the route an object has travelled'''
        logging.debug("[maskRoute]    Start")
        
        x0 = self.xball_cen-(MASK_DELTA/self.accuracyMode)
        y0 = self.yball_cen-(MASK_DELTA/self.accuracyMode)
        x1 = self.xball_cen+(MASK_DELTA/self.accuracyMode)
        y1 = self.yball_cen+(MASK_DELTA/self.accuracyMode)

        if x0 <= 0:
            x0 = 0
        if y0 <= 0:
            y0 = 0
        if x1 >= self.contour_img.shape[1]:
            x1 = self.contour_img.shape[1] - 1
        if y1 >= self.contour_img.shape[0]:
            y1 = self.contour_img.shape[0] - 1

        self.contour_img[y0:y1, x0:x1] = 0
        self.contour_img_mask[y0:y1, x0:x1] = [0, 0, 0]       
        logging.debug("[maskRoute]    End")
   
   
    def unmaskRoute(self):
        '''unmask the route an object has travelled'''
        logging.debug("[unmaskRoute]    Start")
        x0 = self.xball_cen-MASK_DELTA
        y0 = self.yball_cen-MASK_DELTA
        x1 = self.xball_cen+MASK_DELTA
        y1 = self.yball_cen+MASK_DELTA

        if x0 <= 0:
            x0 = 0
        if y0 <= 0:
            y0 = 0
        if x1 >= self.contour_img.shape[1]:
            x1 = self.contour_img.shape[1] - 1
        if y1 >= self.contour_img.shape[0]:
            y1 = self.contour_img.shape[0] - 1        

        contourROI, contX0, contY0 = self.getROI(
            self.ogContourImg, 0, x0, y0, x1, y1)
        self.contour_img[y0:y1, x0:x1] = contourROI

        self.contour_img_mask[y0:y1, x0:x1] = [1, 1, 1] 
        logging.info("[unmaskRoute]    The past route has been unmasked")
        logging.debug("[unmaskRoute]    End")
    

    @staticmethod
    @jit(nopython=True)
    def getVector(contour, ball_coo, vectIter, nearest_idx_maze):
        '''Find the vector between the closest line position and the current position of the object'''
        # ROI for the vectoring
        x0_ROI = ball_coo[0] - ROI_VECT
        x1_ROI = ball_coo[0]+ball_coo[2] + ROI_VECT
        y0_ROI = ball_coo[1] - ROI_VECT
        y1_ROI = ball_coo[1]+ball_coo[3] + ROI_VECT
        if x0_ROI<0:
            x0_ROI=0
        if x1_ROI>contour.shape[1]:
            x1_ROI = contour.shape[1] - 1
        if y0_ROI<0:
            y0_ROI=0
        if y1_ROI>contour.shape[0]:
            y1_ROI = contour.shape[0] - 1
        contourROI = contour[y0_ROI:y1_ROI, x0_ROI:x1_ROI]
        
        xball_cen = int(ball_coo[0] + (ball_coo[2]/2))
        yball_cen = int(ball_coo[1] + (ball_coo[3]/2)) 
        
        nonzero = np.argwhere(contourROI==1)
 
        if vectIter%VECT_ITER_OFFSET!=0:
            pid_reset = 0
            vectorY = nearest_idx_maze[0]-yball_cen 
            vectorX = nearest_idx_maze[1]-xball_cen 
            vectIter += 1
            return vectorX, vectorY, nearest_idx_maze, vectIter, pid_reset

        elif len(nonzero):
            distances = np.sqrt(
                np.square(nonzero[:,0] - yball_cen + y0_ROI) \
                    + np.square(nonzero[:,1] - xball_cen + x0_ROI))
            nearest_idx = nonzero[np.argmin(distances)]
            nearest_idx_maze = np.array([nearest_idx[0] + y0_ROI, nearest_idx[1] + x0_ROI])
            vectorY = nearest_idx[0]-yball_cen + y0_ROI
            vectorX = nearest_idx[1]-xball_cen + x0_ROI
            vectIter += 1
            pid_reset = 1
            return vectorX, vectorY, nearest_idx_maze, vectIter, pid_reset

        return 0, 0, nearest_idx_maze, 0, 0
        


        
    def getRollingWindow(self, frame):
        logging.debug("[getRollingWindow]    Start")
        self.ball_coo = ProcessImage.detectBall(frame)
        if self.ball_coo[3] ==0:
            return 0
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

                return 1
            
            distances = np.sqrt(
                np.square(nonzero[:,0] - y_pos + y0_ROI) \
                    + np.square(nonzero[:,1] - x_pos + x0_ROI))
            nearest_idx = nonzero[np.argmin(distances)]
            frame = arrowedLine(frame, (x_pos, y_pos), 
                                (nearest_idx[1]+ x0_ROI, nearest_idx[0] + y0_ROI),
                                (0, 255, 0), 2) 
  
            x_pos = nearest_idx[1] + x0_ROI
            y_pos = nearest_idx[0] + y0_ROI
            nodeList.append([y_pos, x_pos])

    @staticmethod
    @jit(nopython=True)
    def windowGuts(xball_cen, yball_cen, nodes):
        accuracyMode = 1
        distances = np.sqrt(
                np.square(nodes[0] - yball_cen) \
                    + np.square(nodes[1]-xball_cen))
        nearestNodeIdx = np.argmin(distances)
        nearest_idx = nodes.T[nearestNodeIdx]
        if (nearestNodeIdx+VIS_WINDOW_SIZE+WINDOW_OFFSET+1)>=len(nodes.T):
            visableNodes = nodes.T[nearestNodeIdx+WINDOW_OFFSET:-1].reshape(-1,2).T
        else:
            abs_Y_dist = abs(nodes.T[nearestNodeIdx+VIS_WINDOW_SIZE][0]-nearest_idx[0])
            abs_X_dist = abs(nodes.T[nearestNodeIdx+VIS_WINDOW_SIZE][1]-nearest_idx[1])
            if (abs_Y_dist<HIGH_ACC_TH) and (abs_X_dist<HIGH_ACC_TH):
                accuracyMode = 2
            if abs_Y_dist<MEDIUM_ACC_TH and abs_X_dist<MEDIUM_ACC_TH:
                accuracyMode = 1.5
            visableNodes = nodes.T[nearestNodeIdx+WINDOW_OFFSET:nearestNodeIdx+VIS_WINDOW_SIZE].reshape(-1,2).T
        return visableNodes, accuracyMode
        
    def theySeeMeRollin(self):
        ''''''
        logging.debug("[moveRollingWindow]    Initilised")
        
        while True:
            if (not self.xball_cen) and (not self.yball_cen):
                continue
            maskContour = np.zeros((self.contour_img.shape[0], self.contour_img.shape[1]))
            visableNodes, accuracyMode = ProcessImage.windowGuts(self.xball_cen, self.yball_cen, self.nodes)
            maskContour[visableNodes[0], visableNodes[1]] = 1
            self.maskContour = ndimage.binary_dilation(maskContour, iterations=NODE_WINDOW).astype(np.uint8)
            logging.info("[moveRollingWindow]    Mask calculated")
    
        
    def moveRollingWindow(self):
        ''''''
        logging.debug("[moveRollingWindow]    Initilised")
        
        while True:
            if not self.xball_cen:
                continue
            
            logging.debug("[moveRollingWindow]    Start")
            maskContour = np.zeros((self.contour_img.shape[0], self.contour_img.shape[1]))
            self.accuracyMode = 1
            distances = np.sqrt(
                    np.square(self.nodes[0] - self.yball_cen) \
                        + np.square(self.nodes[1]-self.xball_cen))

            nearestNodeIdx = np.argmin(distances)
            nearest_idx = self.nodes.T[nearestNodeIdx]
            logging.warning("The nearest node number: %i, last node: %i, total lenght: %i", 
                            nearestNodeIdx, nearestNodeIdx+VIS_WINDOW_SIZE, len(self.nodes.T))
            if (nearestNodeIdx+VIS_WINDOW_SIZE+2)>=len(self.nodes.T):
                visableNodes = self.nodes.T[nearestNodeIdx+2:-1].reshape(-1,2).T
            else:
                abs_Y_dist = abs(self.nodes.T[nearestNodeIdx+VIS_WINDOW_SIZE][0]-nearest_idx[0])
                abs_X_dist = abs(self.nodes.T[nearestNodeIdx+VIS_WINDOW_SIZE][1]-nearest_idx[1])
                if (abs_Y_dist<HIGH_ACC_TH) and (abs_X_dist<HIGH_ACC_TH):
                    logging.info("[moveRollingWindow]    Double accuracy mode")
                    self.accuracyMode = 2
                if abs_Y_dist<MEDIUM_ACC_TH and abs_X_dist<MEDIUM_ACC_TH:
                    logging.info("[moveRollingWindow]    Accuracy mode")
                    self.accuracyMode = 1.5
                visableNodes = self.nodes.T[nearestNodeIdx+2:nearestNodeIdx+VIS_WINDOW_SIZE].reshape(-1,2).T
                logging.warning("ACCURACY MODE: %f", self.accuracyMode) 

            maskContour[visableNodes[0], visableNodes[1]] = 1
            self.maskContour = ndimage.binary_dilation(maskContour, iterations=NODE_WINDOW).astype(np.uint8)
            logging.info("[moveRollingWindow]    Mask calculated")
            
        
    @staticmethod
    @jit(nopython=True)   
    def windowMasking(windowContour, windowFrame, xball_cen, yball_cen, accuracyMode):
        x0 = xball_cen-int(MASK_DELTA/accuracyMode)
        y0 = yball_cen-int(MASK_DELTA/accuracyMode)
        x1 = xball_cen+int(MASK_DELTA/accuracyMode)
        y1 = yball_cen+int(MASK_DELTA/accuracyMode)

        if x0 <= 0:
            x0 = 0
        if y0 <= 0:
            y0 = 0
        if x1 >= windowContour.shape[1]:
            x1 = windowContour.shape[1] - 1
        if y1 >= windowContour.shape[0]:
            y1 = windowContour.shape[0] - 1

        windowContour[y0:y1, x0:x1] = 0
        
        windowFrame[y0:y1, x0:x1] = np.array([0, 0, 0]).astype(np.uint8)       
        return windowContour, windowFrame
 
    @staticmethod
    def moveWindowNoThread(contour_img, nodes, xball_cen, yball_cen):
        ''''''
        maskContour = np.zeros((contour_img.shape[0], contour_img.shape[1]))
        accuracyMode = 1
        distances = np.sqrt(
                np.square(nodes[0] - yball_cen) \
                    + np.square(nodes[1]-xball_cen))
        nearestNodeIdx = np.argmin(distances)
        nearest_idx = nodes.T[nearestNodeIdx]
        if (nearestNodeIdx+VIS_WINDOW_SIZE+2)>=len(nodes.T):
            visableNodes = nodes.T[nearestNodeIdx+1:-1].reshape(-1,2).T
        else:
            abs_Y_dist = abs(nodes.T[nearestNodeIdx+VIS_WINDOW_SIZE][0]-nearest_idx[0])
            abs_X_dist = abs(nodes.T[nearestNodeIdx+VIS_WINDOW_SIZE][1]-nearest_idx[1])
            if (abs_Y_dist<HIGH_ACC_TH) and (abs_X_dist<HIGH_ACC_TH):
                accuracyMode = 30
            if abs_Y_dist<MEDIUM_ACC_TH and abs_X_dist<MEDIUM_ACC_TH:
                accuracyMode = 15
            visableNodes = nodes.T[nearestNodeIdx+1:nearestNodeIdx+VIS_WINDOW_SIZE].reshape(-1,2).T
        maskContour[visableNodes[0], visableNodes[1]] = 1
        maskContour = ndimage.binary_dilation(maskContour, iterations=NODE_WINDOW).astype(np.uint8)
        return maskContour
            


    def showMaskedVector(self, frame, vector_coo, antiHoleVector):
        '''show the image with a vector and masked route undertaken by the object'''
        frame = np.bitwise_and(frame, self.contour_img_mask)
        frame = arrowedLine(frame, (self.xball_cen, self.yball_cen), 
                                (self.xball_cen+vector_coo[0], self.yball_cen+vector_coo[1]),
                                (0, 255, 0), 3)
        if antiHoleVector[0] and antiHoleVector[1]:
            frame = arrowedLine(frame, (self.xball_cen, self.yball_cen), 
                                    (self.xball_cen+int(antiHoleVector[0]), self.yball_cen+int(antiHoleVector[1])),
                                    (0, 0, 255), 3)
            frame = arrowedLine(frame, (self.xball_cen, self.yball_cen), 
                                    (self.xball_cen+vector_coo[0]+int(antiHoleVector[0]), 
                                    self.yball_cen+vector_coo[1]+int(antiHoleVector[1])),
                                    (255, 0, 0), 3)
        imshow('Vector and masked', frame)
        return frame


    def showBallVector(self, frame, vector_coo, antiHoleVector):
        '''show the image with a vector'''

        frame = rectangle(frame, (self.ball_coo[0],self.ball_coo[1]), 
                          (self.ball_coo[0]+self.ball_coo[2],
                           self.ball_coo[1]+self.ball_coo[3]), 
                          (255, 0, 0),  1)
        frame = arrowedLine(frame, (self.xball_cen, self.yball_cen), 
                                (self.xball_cen+vector_coo[0], self.yball_cen+vector_coo[1]),
                                (0, 255, 0), 3)
        if antiHoleVector[0] and antiHoleVector[1]:
            frame = arrowedLine(frame, (self.xball_cen, self.yball_cen), 
                                    (self.xball_cen+int(antiHoleVector[0]), self.yball_cen+int(antiHoleVector[1])),
                                    (0, 0, 255), 3)
            frame = arrowedLine(frame, (self.xball_cen, self.yball_cen), 
                                    (self.xball_cen+vector_coo[0]+int(antiHoleVector[0]), 
                                    self.yball_cen+vector_coo[1]+int(antiHoleVector[1])),
                                    (255, 0, 0), 3)
        imshow("Vector and Ball", frame)



    def processWindowImg(self, frame):
        '''Run the processing software'''
        logging.debug("[processWindowImg]    Start")
        frame, _, _ = ProcessImage.jitROI(frame, 0,
                                                MAZE_ROI[0], MAZE_ROI[1], 
                                                MAZE_ROI[2], MAZE_ROI[3]) 
        frame1 = np.copy(frame)
        self.ballFrame = np.copy(frame)
        cannyImg= self.getCanny(frame1)
    
        
        if self.contour_ok==0:    
            lineFrame = np.copy(frame)
            if not self.getLeadLine(cannyImg, lineFrame):
                return np.array([])
            self.holeContour = self.detectHoles(lineFrame, cannyImg)

        if self.contour_ok:  
            if self.restart:
                return np.array([])
            # imshow("Original frame", frame)
            self.ball_coo = (0, 0, 0, 0)
            count = 0
            while self.ball_coo[3]==0:
                if count==2:
                    return np.array([])
                image = self.ballFrame
                self.ball_coo = ProcessImage.detectBall(image)
                self.xball_cen = int(self.ball_coo[0] + (self.ball_coo[2]/2))
                self.yball_cen = int(self.ball_coo[1] + (self.ball_coo[3]/2))
                count += 1
            
            if not self.xball_cen and self.yball_cen:
                logging.warning("[processWindowImg]    Ball not detected")
                return np.array([])
            
            if abs(self.nodes[0][-1]-self.yball_cen)<COMPLETE_TH and abs(self.nodes[1][-1]-10-self.xball_cen)<COMPLETE_TH:
                print("DONE DONE DONE DONE")
                return np.array([0])
            flickEnable = self.flick()

            maskContour = np.zeros((self.contour_img.shape[0], self.contour_img.shape[1]))
            visableNodes, accuracyMode = ProcessImage.windowGuts(self.xball_cen, self.yball_cen, self.nodes)
            maskContour[visableNodes[0], visableNodes[1]] = 1
            self.maskContour = ndimage.binary_dilation(maskContour, iterations=NODE_WINDOW).astype(np.uint8)
            
            # create a mask for the RGB frame
            maskFrame = np.repeat(self.maskContour, 3).reshape(self.contour_img.shape[0], self.contour_img.shape[1], 3) * 255
            windowFrame = np.bitwise_and(frame, maskFrame.astype(np.uint8)) # get windowed frame

            
            # get windowed lead line

            windowContour = np.copy(self.ogContourImg).astype(np.uint8)

            # imshow("mask", maskFrame)

            windowContour = np.bitwise_and(windowContour, self.maskContour)

            logging.warning("ACCURACY MODE: %f", accuracyMode)
            # mask the balls postion
            windowContour, windowFrame = ProcessImage.windowMasking(windowContour, windowFrame, self.xball_cen, self.yball_cen, accuracyMode)
            Vx, Vy, self.nearest_idx_maze, self.vectIter, pid_reset = ProcessImage.getVector(
                windowContour, self.ball_coo, self.vectIter, self.nearest_idx_maze)

            if not Vx and not Vy:
                return np.array([])
            antiHoleVectorX, antiHoleVectorY = ProcessImage.dodgeHoles(
                self.holeContour, self.xball_cen, self.yball_cen, self.ball_coo)
            logging.warning("[processWindowImg]   The raw ball vector, x: %i, y: %i", Vx, Vy)
            logging.warning("[processWindowImg]   Inti-hole vector, x: %i, y: %i", antiHoleVectorX, antiHoleVectorY)
            finalAngle1, finalAngle2 =self.servos.PositionPDController(
                        Vx+antiHoleVectorX, Vy+antiHoleVectorY, pid_reset)
            
            if not flickEnable:       
                try:
                    self.runServo.terminate()
                
                except AttributeError:
                    logging.warning("[processWindowImg]   Threading error") 

                self.runServo = Process(
                    target=self.servos.moveBothServos, args=(
                        finalAngle1, finalAngle2,))        
                self.currentAngle1 = finalAngle1
                self.currentAngle2 = finalAngle2
              
                self.runServo.start()
            
            logging.warning("[processWindowImg]    The net vector (%i, %i)", Vx+antiHoleVectorX, Vy+antiHoleVectorY)
            frameArrowed = self.showMaskedVector(windowFrame, [Vx, Vy], [antiHoleVectorX, antiHoleVectorY])
            # self.showBallVector(frame, [Vx, Vy], [antiHoleVectorX, antiHoleVectorY])
            return frameArrowed     
        logging.debug("[processWindowImg]   End")       



    
def live_feedV2():
    vid = VideoCapture(0)
    image = ProcessImage()
    condition = True
    current_t = 0 
    timestamp_start = time.time()
    # while(condition):
    #     condition = int(input('give 0: '))
   
    while(True):
        ret, frame = vid.read()
        if current_t>=3:
            complete = len(image.processWindowImg(frame))
            # print(complete)
            if complete == 1:
                break
        else:
            current_t = time.time()-timestamp_start
        if waitKey(1) & 0xFF == ord('q'):
            break
        
    vid.release()
    destroyAllWindows()

live_feedV2()

def videoV2():
    cap = VideoCapture('WIN_20230206_16_20_55_Pro.mp4')
    # servos = servo.servo()
    image = ProcessImage()

    while(cap.isOpened()):
        _, frame = cap.read()
        img_75 = resize(frame, None, fx = 0.5, fy = 0.5)
        imshow("OG frame", img_75)
        # ROI_image_rgb, canny_image = get_ROI(img_75, 0, 0, 1000, 1000)
        # line_image, lines, combo_image = get_lines(canny_image, img_75)
        # ball_line_maze_image, lines_data, ball_data, canny_image,image_gray = process_image(img_75)
        # cv2.imshow("combo_image after display_lines", ball_line_maze_image)
        #time.sleep(0.1)
        image.processWindowImg(img_75)
        #time.sleep(0.1)
        if waitKey(1) & 0xFF == ord('q'):
            break
# videoV2()

#if __name__ == "__main__":
    # maze = multiprocessing.Process(target=live_feedV2(), args=())
    #maze = threading.Thread(target=live_feedV2(), args=())
    #maze.start()
    # live_feedV2()


'''
NOTES

add detect holes in a loop so we make sure that all holes are detected




'''


