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
        self.prevBall = [0, 0]
        self.accuracyMode = 1

        
        # initialize servos parameters
        self.servos = servo.servo()
        self.currentAngle1 = 90
        self.currentAngle2 = 90

        # vector interation offset
        self.vectIter = 0
        self.pid_reset = 1

    

        
 
 
            
    @staticmethod
    def detectBall(image, debug=False):
        '''find a blue object of specific dimensions and area'''
        logging.debug("[detectBall]    Start")
        image = np.copy(image)
        x_c, y_c, w_c, h_c = [0, 0, 0, 0]
        
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, BLUE_LOWER, BLUE_UPPER)
        if debug:
            cv2.imshow('DEBUG blue colour', mask)
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
        



      

    def testPID(self, frame, y_coo, x_coo):
        frame1 = np.copy(frame)
        ball_coo = ProcessImage.detectBall(frame, debug=False)

        if not ball_coo[0]:
            #cv2.imshow("dfeg", frame)
            return 0
        self.xball_cen = int(ball_coo[0]+(ball_coo[2]/2))
        self.yball_cen = int(ball_coo[1]+(ball_coo[3]/2))
        
        if self.pid_reset == 0 and (self.xball_cen<73 or self.xball_cen>520):
            self.pid_reset = 2 #this disables the x axis derivative gain only
        
        finalAngle1, finalAngle2 =self.servos.PositionPDController(
                        x_coo-self.xball_cen, y_coo-self.yball_cen, self.pid_reset)
        self.pid_reset = 0
                    
        try:
            self.runServo.terminate()
            
        except AttributeError:
            pass
        self.runServo = multiprocessing.Process(
            target=self.servos.moveBothServos, args=(
                finalAngle1, finalAngle2))                  
        self.runServo.start()
        frame = self.showBallVector(frame, ball_coo, (x_coo-self.xball_cen, y_coo-self.yball_cen))
        cv2.imshow("Vector and Ball", frame)


    


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

    path_coords = [[88,150],[98,183],[125,181],[158,170],[183,169],[206,179],[235,197],[245,230],
    [246, 231], [235,255],[208, 268],[192,270],[170,264],[139,261],[120,282],[105,314],[100,337],[101,371]]
    count = 0
    index = 0
    while(True):
        # Capture the video frame
        # by frame
        if count%70 == 0: #divisible by 100 i.e. 100 loops have passed
            index = index + 1
        coords = path_coords[index]
        ret, frame = vid.read()
        if current_t>=3:
            image.testPID(frame, 375, 100)
            #image.testPID(frame, coords[1], coords[0])
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
