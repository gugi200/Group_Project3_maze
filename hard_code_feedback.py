import time
import servo
import speed_solve_lib as vp
from cv2 import (cvtColor, GaussianBlur, Canny, COLOR_RGB2GRAY, imshow, 
threshold, findContours, THRESH_BINARY, RETR_EXTERNAL, CHAIN_APPROX_NONE, 
rectangle, resize, waitKey, destroyAllWindows, VideoCapture, arrowedLine, 
CHAIN_APPROX_SIMPLE, inRange,contourArea, drawContours, COLOR_BGR2HSV, FILLED, boundingRect)
#MAZE_ROI = [68-10, 30-20, 513+45, 406+20]# x0, y0, x1, y1 please use these values if the values are changed the coordinates will be off

servos = servo.servo()

def top_left():
    servos.moveBothServos(0, 180)
    time.sleep(0.1)

def top_right():
    servos.moveBothServos(180, 180)
    time.sleep(0.1)
    
def bottom_left():
    servos.moveBothServos(0, 0)
    time.sleep(0.1)
    
def bottom_right():
    servos.moveBothServos(180, 0)
    time.sleep(0.1)

def easy_maze_motion(ball_coo,no_motion_count,done):
    done = 0
    if (ball_coo[0] in range(285,325)) and (ball_coo[1] in range(15,45)):
        bottom_left()
        print("checkpoint 1")
        
    if (ball_coo[0] in range(75,85)) and (ball_coo[1] in range(200,230)):
        top_right()
    if (ball_coo[0] in range(225,250)) and (ball_coo[1] in range(115,135)):
        bottom_right()
        time.sleep(0.5)
    if (ball_coo[0] in range(230,250)) and (ball_coo[1] in range(270,290)):
        top_left()
        time.sleep(1)
    if (ball_coo[0] in range(65,85)) and (ball_coo[1] in range(225,245)):    
        bottom_left()
        print("top")
    if (ball_coo[0] in range(75,95)) and (ball_coo[1] in range(248,268)):
        servos.moveBothServos(120, 0)
        time.sleep(1.25)
        print("bottom")
    if (ball_coo[0] in range(115,135)) and (ball_coo[1] in range(320,340)):
        bottom_left()
    if (ball_coo[0] in range(90,110)) and (ball_coo[1] in range(380,400)):
        top_right()
    if (ball_coo[0] in range(210,230)) and (ball_coo[1] in range(305,325)):
        bottom_right()
    if (ball_coo[0] in range(265,290)) and (ball_coo[1] in range(365,385)):
        top_right()
    if (ball_coo[0] in range(367,387)) and (ball_coo[1] in range(232,252)):
        bottom_left()
    if (ball_coo[0] in range(270,290)) and (ball_coo[1] in range(272,292)):
        top_left()
    if (ball_coo[0] in range(265,285)) and (ball_coo[1] in range(152,172)):
        top_right()
    if (ball_coo[0] in range(430,450)) and (ball_coo[1] in range(100,125)):
        top_left()
    if (ball_coo[0] in range(340,360)) and (ball_coo[1] in range(20,40)):
        top_right()
    if (ball_coo[0] in range(495,520)) and (ball_coo[1] in range(38,58)):
        bottom_right()
        time.sleep(0.5)
    if (ball_coo[0] in range(515,535)) and (ball_coo[1] in range(100,120)):
        bottom_left()
    if (ball_coo[0] in range(470,490)) and (ball_coo[1] in range(116,136)):
        bottom_right()
    if (ball_coo[0] in range(515,535)) and (ball_coo[1] in range(210,230)):
        bottom_left()
    if (ball_coo[0] in range(472,492)) and (ball_coo[1] in range(270,290)):
        bottom_right()
    if (ball_coo[0] in range(508,528)) and (ball_coo[1] in range(384,404)):
        bottom_left()
    if (ball_coo[0] in range(445,465)) and (ball_coo[1] in range(408,428)):
        top_left()
    if (ball_coo[0] in range(395,415)) and (ball_coo[1] in range(315,335)):
        bottom_left()
        done = 1
        
    else:
        no_motion_count += 1
        print(no_motion_count)
        if no_motion_count == 200:
            servos.flick(90,90,0.015)    	   
            no_motion_count = 0
    
    return no_motion_count, done
    
    
def medium_maze_motion(ball_coo,no_motion_count,done):
    #if (ball_coo[0] in range(110,390)) and (ball_coo[1] in range(10,176)):
        if (ball_coo[0] in range(340,390)) and (ball_coo[1] in range(10,35)):
            top_left()
            print("checkpoint 1")
        if (ball_coo[0] in range(145,170)) and (ball_coo[1] in range(35,65)):
            servos.moveBothServos(80,0)
            time.sleep(0.1)
            print("checkpoint 2")
        if (ball_coo[0] in range(110,140)) and (ball_coo[1] in range(140,160)):
            servos.moveBothServos(150,0)
            time.sleep(0.1)
            print("checkpoint 3")
     #   no_motion_count = 0
    
    #if (ball_coo[0] in range(65,200)) and (ball_coo[1] in range(215,370)):
        if (ball_coo[0] in range(165,185)) and (ball_coo[1] in range(235,260)):
            top_left()
            print("checkpoint 4")
        if (ball_coo[0] in range(65,90)) and (ball_coo[1] in range(220,240)):
            bottom_left()
            print("checkpoint 5")
        if (ball_coo[0] in range(90,115)) and (ball_coo[1] in range(335,355)):
            bottom_right()
            
        if (ball_coo[0] in range(110,130)) and (ball_coo[1] in range(345,365)):
            top_right()
            print("checkpoint 6")
     #   no_motion_count = 0
    
    #if (ball_coo[0] in range(159,435)) and (ball_coo[1] in range(247,390)):
        if (ball_coo[0] in range(240,270)) and (ball_coo[1] in range(270,290)):
                bottom_right()
                print("checkpoint 7")
        if (ball_coo[0] in range(363,380)) and (ball_coo[1] in range(345,358)):
                servos.moveBothServos(180,120)
                time.sleep(1)
                top_right()
                time.sleep(0.5)
                bottom_left()
                time.sleep(0.5)
                
                print("checkpoint 8")
        if (ball_coo[0] in range(400,420)) and (ball_coo[1] in range(345,370)):
                bottom_right()
                print("checkpoint 9")
                time.sleep(0.5)
                bottom_left()
                done = 1
        if (ball_coo[0] in range(418,438)) and (ball_coo[1] in range(392,415)):
                bottom_left()
                done = 1
            
     #       no_motion_count = 0
            
        else:
    	    no_motion_count += 1
    	    print(no_motion_count)
    	    if no_motion_count == 200:
    	       servos.flick(90,90,0.01)
    	   
    	       no_motion_count = 0
    	   
        return no_motion_count, done
        

def get_centre_pt(ball_coo):
    ball_x = ball_coo[0] + (ball_coo[2]/2)
    ball_y = ball_coo[1] + (ball_coo[3]/2)
    return (ball_x , ball_y)

def prevent_0(ball_coo):
    if ball_coo[0] == 0 and ball_coo[1] == 0:
        ball_coo[0] = 10000
        ball_coo[1] = 10000
    return [ball_coo[0],ball_coo[1],ball_coo[2],ball_coo[3]]

# def maze_difficulty(image,frame):
#     cannyImg = image.getCanny(frame)
#     holeCount, holeCountour = image.detectHolesNEW(frame, cannyImg)
#     return holeCount

def holeCount(vid,image):
    holeCountList = []
    for x in range(75):
        ret,frame = vid.read()
        cannyImg = image.getCanny(frame)
        holeCount, holeCountour = image.detectHolesNEW(frame, cannyImg)
        holeCountList.append(holeCount)    
    return max(holeCountList)
    
def solve_easy(frame,no_motion_count,done):
    ball_coo = vp.ProcessImage.detectBall(frame)
    ball_coo = list(ball_coo)
    ball_coo = prevent_0(ball_coo)
    ball_centre_pt = get_centre_pt(ball_coo)
    no_motion_count,done = easy_maze_motion(ball_centre_pt,no_motion_count,done)
    print(ball_centre_pt)
    #imshow("frame",frame)
    return no_motion_count,done

def solve_medium(frame,no_motion_count,done):
    ball_coo = vp.ProcessImage.detectBall(frame)
    ball_coo = list(ball_coo)
    ball_coo = prevent_0(ball_coo)
    ball_centre_pt = get_centre_pt(ball_coo)
    no_motion_count,done = medium_maze_motion(ball_centre_pt,no_motion_count,done)
    print(ball_centre_pt)
    #imshow("frame",frame)
    return no_motion_count,done


    
def live_feedV2_hardcode():
    vid = vp.VideoCapture(0)
    #servos = servo.servo()
    image = vp.ProcessImage()
    condition = 0
    difficulty = 0
    done = 0
    #numHoleList = []
    current_t = 0 
    no_motion_count = 0
    
    #while(condition == 0):
     #   condition = int(input('Input maze difficulty. 1 for easy, 2 for medium'))
      #  difficulty = condition
        
    timestamp_start = time.time()
    
    while(True):
        if current_t>3:
            # for x in range(10):      
            #    ret, frame = vid.read()
            #    #imshow("looking for holes", frame)
            #    numHole = maze_difficulty(image, frame)
            #    numHoleList.append(numHole)
               
            # numHole = max(numHoleList)
            numHole = holeCount(vid, image)
            print("final hole count:",numHole)
            if numHole >= 6 and numHole <= 8:
                difficulty = 1
            else:
                difficulty = 2
            break
        
        else:
            current_t = time.time()-timestamp_start

        
    

    #while(True):
     #   if current_t >=3:
      #      for x in range(10):      
       #         ret, frame = vid.read()
        #        numHole = maze_difficulty(image, frame)
         #       print(numHole)
          #      holeCountSum += numHole            
           # numHole = holeCountSum/10
            #break
        #else:
         #   current_t = time.time()-timestamp_start
            # the 'q' button is set as the
        # quitting button you may use any
        # desired button of your choice
        #if waitKey(1) & 0xFF == ord('q'):
         #   break
        
    #print("holecount:",numHole)      
    
    while(True):
        # Capture the video frame
        # by frame
        ret, frame = vid.read()
        if done == 1:
            print("done")
            break
        elif current_t>=3:
            #if numHole <= 8:
             #   pass
            #if numHole>8 and numHole <= 21:
            if difficulty == 1:
                no_motion_count,done = solve_easy(frame, no_motion_count,done)
            elif difficulty == 2:
                no_motion_count,done = solve_medium(frame, no_motion_count,done)
        
        
        
        else:
            current_t = time.time()-timestamp_start
            # the 'q' button is set as the
        # quitting button you may use any
        # desired button of your choice
        if waitKey(1) & 0xFF == ord('q'):
            break
        
    # After the loop release the cap object
    vid.release()
    # Destroy all the windows
    vp.destroyAllWindows()
            
live_feedV2_hardcode()        

    
