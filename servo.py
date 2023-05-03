from periphery import PWM
import time
from synDef import *
import logging

class servo():

    def __init__(self): #initiate the pwm pins for 2 servos, the pwm objects are named pwm1 and pwm2 and are the name of the servos
        self.pwm1 = PWM(0,0)  #pin33
        self.pwm2 = PWM(4,0)  #pin7
        self.pwm1.frequency = 50
        self.pwm2.frequency = 50
        self.pwm1.polarity = "normal"
        self.pwm2.polarity = "normal"
        self.servoPos_Standalone(self.pwm1,90,1) #sets servos to 0 degrees with max delay, can be other angles
        self.servoPos_Standalone(self.pwm2,90,1)
        self.currentAngle1 = 90   #initialises current position
        self.currentAngle2 = 90
	
	#Control Loop Variables
        self.error_pos_x_prev = 0 #both for derivative control
        self.error_pos_y_prev = 0
        
        self.error_time_prev = 0
        
        self.error_total_x = 0
        self.error_total_y = 0
        
        
        # self.x_speed_prev = 0
        # self.y_speed_prev = 0

        
        # addition
        self.prevAngle1 = 90
        self.prevAngle2 = 90
       

    def servoPos_Standalone(self,servo,angle,delay): #move select servo to a specified angle with a certain delay
        #if angle > 180:
         #   angle = 180
        #elif angle < 0:
         #   angle = 0
        duty_cycle = (0.125-0.025)/180*angle + 0.025
        servo.duty_cycle = duty_cycle
        servo.enable()
        time.sleep(delay)
        servo.disable()

    def servoPos(self, servo,angle): #set servo duty cycle and enable output, needs to go with moveBothServos() to work, if need standalone servo function use the commented out version
        if angle > 180:
            angle = 180
        elif angle < 0:
            angle = 0
        duty_cycle = (0.125-0.025)/180*angle + 0.025
        servo.duty_cycle = duty_cycle
        servo.enable()


    def optimumDelay(self,currentAngle,finalAngle): #the numbers are potentially wrong, need experimenting
        delay = abs(currentAngle - finalAngle)*0.0025
        return delay


    def moveBothServos(self,finalAngle1,finalAngle2):
        logging.info("the final %f", finalAngle1)
        
        #delay1 = self.optimumDelay(self.currentAngle1, finalAngle1)
        #delay2 = self.optimumDelay(self.currentAngle2, finalAngle2)
        delay1 = 0.07  #mock values
        delay2 = 0.06
        if delay1 >= delay2: 
            self.servoPos(self.pwm1,finalAngle1)
            self.servoPos(self.pwm2,finalAngle2)
            time.sleep(delay1)
        else:
            self.servoPos(self.pwm2,finalAngle2)
            self.servoPos(self.pwm1,finalAngle1)
            time.sleep(delay2)

        self.pwm1.disable()  
        self.pwm2.disable()
        self.currentAngle1 = finalAngle1  #updates the currentAngle everytime after moving
        self.currentAngle2 = finalAngle2
        
    def flick(self,finalAngle1,finalAngle2,delay):
        
        #delay1 = self.optimumDelay(self.currentAngle1, finalAngle1)
        #delay2 = self.optimumDelay(self.currentAngle2, finalAngle2)
        self.servoPos(self.pwm1,finalAngle1)
        self.servoPos(self.pwm2,finalAngle2)    
        time.sleep(delay)    
        self.pwm1.disable()  
        self.pwm2.disable()
        


    def moveBothServosDelayed(self, finalAngle1, finalAngle2, currentAngle1, currentAngle2):
        self.servoPos(self.pwm1,currentAngle1)
        self.servoPos(self.pwm2,currentAngle2)
        delay1 = self.optimumDelay(currentAngle1, finalAngle1)
        delay2 = self.optimumDelay(currentAngle2, finalAngle2)
        deltaAngle1 = (finalAngle1 - currentAngle1)/NUM_DELTA
        deltaAngle2 = (finalAngle2 - currentAngle2)/NUM_DELTA
        for _ in range(NUM_DELTA):
            if delay1 >= delay2: 
                self.servoPos(self.pwm1,currentAngle1 + deltaAngle1)
                self.servoPos(self.pwm2,currentAngle2 + deltaAngle2)
                time.sleep(delay1/NUM_DELTA)
            else:
                self.servoPos(self.pwm1, currentAngle1 + deltaAngle1)
                self.servoPos(self.pwm2, currentAngle2 + deltaAngle2)
                time.sleep(delay2/NUM_DELTA)
            print("the current angle: ", currentAngle1, " ", currentAngle2, "the final angle: ", finalAngle1," ", finalAngle2)

            currentAngle1 += deltaAngle1  
            currentAngle2 += deltaAngle2
            time.sleep(SOFT_DELAY_MS/1000)

        self.pwm1.disable()
        self.pwm2.disable()

 
    def tiltBoard(self, desired_velocity1, desired_velocity2, measured_velocity1, measured_velocity2, forward_loop_gain, feedback_gain):
        error_velocity1 = desired_velocity1 - (measured_velocity1*feedback_gain)
        error_velocity2 = desired_velocity2*1.5 - (measured_velocity2*feedback_gain)
        
        finalAngle1 = 90 + (error_velocity1 * forward_loop_gain)
        finalAngle2 = 90 - (error_velocity2 * forward_loop_gain)
        
        if finalAngle1 > 180:
            finalAngle1 = 180
        elif finalAngle1 < 0:
            finalAngle1=0
            
        if finalAngle2 > 180:
            finalAngle2 = 180
        elif finalAngle2 < 0:
            finalAngle2=0
        print("x and y motor angles: " + str(finalAngle1) + " " + str(finalAngle2))
        self.moveBothServos(finalAngle1, finalAngle2)
        # self.moveBothServosDelayed(finalAngle1, finalAngle2)
        

    def PositionPDController(self,error_pos_x, error_pos_y, reset):
        #CALC TIME BETWEEN LOOPS
        error_time = time.time() #current time related to the current error vector
        h = (error_time - self.error_time_prev)*10 #difference in time between the current error vector and the previous error vector
        if reset == 1: #deals with initial value of h on first loop (add: or h>2000 for unlikely wrap-around scenario?)
        	h=0
        #print("h= ", h) #print to check it's working
        
        
        #CALC DERIVATIVE TERM
        if reset == 0: #normal operation:
             derivative_term_x = (error_pos_x - self.error_pos_x_prev)/h #calculate derivative terms
             derivative_term_y = (error_pos_y - self.error_pos_y_prev)/h
             MULTIPLIER = 1
        elif reset == 1: # First loop of vector pointing to a new coordinate. 
             derivative_term_x = 0
             derivative_term_y = 0
             MULTIPLIER = JOLT_IMPULSE
        elif reset == 2: # Ball is close to the left or right edge. Prevent positive feedback tilting problem.
             derivative_term_x = 0
             derivative_term_y = (error_pos_y - self.error_pos_y_prev)/h
             MULTIPLIER = 1
        elif reset == 3: # Ball is close to the top or bottom edge. Prevent positive feedback tilting problem
             derivative_term_x = (error_pos_x - self.error_pos_x_prev)/h
             derivative_term_y = 0
             MULTIPLIER = 1
        elif reset == 4: # Ball is close to the top or bottom edge. Prevent positive feedback tilting problem
             derivative_term_x = (error_pos_x - self.error_pos_x_prev)/h
             derivative_term_y = (error_pos_y - self.error_pos_y_prev)/h
             MULTIPLIER = SMALL_JOLT
	
	#print to check stuff
        #print("error_pos_y", error_pos_y)
        #print("self.error_pos_prev", self.error_pos_y_prev) 


        #CALC INTEGRAL TERMS 
        self.error_total_x = self.error_total_x + (error_pos_x*h) #calculate integral terms
        self.error_total_y = self.error_total_y + (error_pos_y*h)


	#IMPLEMENT PID        
        finalAngle1 = 90 + ( KPX*((error_pos_x*MULTIPLIER) + TDX*derivative_term_x + (1/TIX)*self.error_total_x) )
        finalAngle2 = 90 - ( KPY*((error_pos_y*MULTIPLIER) + TDY*derivative_term_y + (1/TIY)*self.error_total_y) )
        logging.info("proportional term (x, y): %f %f", KPX*error_pos_x, KPY*error_pos_y)
        logging.info("derivaive term (x, y): %f %f",KPX*TDX*derivative_term_x, KPY*TDY*derivative_term_y)
        logging.info("intergral term (x, y): %f %f",KPX*(1/TIX)*self.error_total_x, KPY*(1/TIY)*self.error_total_y)
        logging.info("PID term x: %f",( 90 + ( KPX*(error_pos_x + TDX*derivative_term_x + (1/TIX)*self.error_total_x) )))
        logging.info("PID term y: %f", (90 - ( KPY*(error_pos_y + TDY*derivative_term_y + (1/TIY)*self.error_total_y) )))
        
        
        #UDPATE CLASS ATTRIBUTES
        self.error_pos_x_prev = error_pos_x
        self.error_pos_y_prev = error_pos_y
        self.error_time_prev = error_time
        
        
        #Limit max/minimum value of angle
        if finalAngle1 > 180:
            finalAngle1 = 180
        elif finalAngle1 < 0:
            finalAngle1=0
            
        if finalAngle2 > 180:
            finalAngle2 = 180
        elif finalAngle2 < 0:
            finalAngle2=0
            
	
	#print motor angles for debugging
        #print("x and y motor angles: " + str(finalAngle1) + " " + str(finalAngle2))
        
	#return the desired angle of the board
        return finalAngle1, finalAngle2
        
              

#pwm1,pwm2,currentAngle1,currentAngle2 = servoInit()
#servoPos_Standalone(pwm1,0,1)
#servos = servo()
#servos.moveBothServos(90,90)
#pwm2 = PWM(4,0)
#pwm1 = PWM(0,0)
#x = 0
#time.sleep(1)
#while(1):

#    servos.servoPos(pwm2,0)
#    time.sleep(0.7)
#    servos.servoPos(pwm2,180)
#    time.sleep(0.7)
#servos.moveBothServos(90,90)
#servos.moveBothServos(40,40)
#servos.moveBothServos(130,130)
#servoPos_w_Speed(pwm1,180,0,0.001)
#time.sleep(2)
#servoPos(pwm1,180,1)
#servoPos(pwm1,30,1)
#servoPos(pwm1,120,1)
#servos.pwm1.close()  #unexport the pwm channels
#servos.pwm2.close()
