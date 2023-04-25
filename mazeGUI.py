from tkinter import (RIGHT, Label, Scrollbar, Frame, Tk, ttk, NORMAL, DISABLED, 
Canvas, BOTH,VERTICAL, LEFT, Y, Button, Menu, Scale, messagebox, Toplevel, StringVar, IntVar, NW, TOP)
#import tk
#from matplotlib.animation import FuncAnimation, PillowWriter
import time
import os
import signal
import matplotlib
from matplotlib.figure import Figure
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import ( FigureCanvasTkAgg, NavigationToolbar2Tk)
import threading
import multiprocessing
from tkVideoPlayer import TkinterVideo
import cv2
import sys
from subprocess import Popen, call
import PIL #import ImageTk, Image
# from video_processing_test2 import live_feedV2
# import video_processing_test3
# import hard_code


class Window():
    
    def __init__(self, root, figure):
        
        self.root = root
        self.root.geometry('500x550')
        self.root.title('Maze Solver - Group F')
        self.root.configure(bg='#ffffff')
        self.figure = figure

    def createMenuFrame(self, canvas):
        '''Creates the menu bar'''
        
        # creates the frame
        menuFrame = Frame(canvas, background='#ededed')
        # creates menu bar
        mainMenu = Menu(menuFrame)

        # adds dropbar in view
        viewMenu = Menu(mainMenu, tearoff=False, )
        viewMenu.add_command(label='One column', command=self.mainFrameOneColumn)
        viewMenu.add_command(label='Two columns', command=self.mainFrameTwoColumns)
        mainMenu.add_cascade(label='View', menu=viewMenu)

        # mainMenu.add_command(label='Load', command=self.load)
        mainMenu.add_command(label='Restart', command=self.restartProgram)
        # mainMenu.add_command(label='Clear data')
        mainMenu.add_command(label='Exit', command=self.exitProgram)

        self.root.config(menu=mainMenu)
        
        # self.startWindow()
        
    def restartProgram(self):
        # call([sys.executable, os.path.realpath(__file__)] + sys.argv[1:])
        os.execl(sys.executable, sys.executable, *sys.argv)
    
    def exitProgram(self):
        # self.digitalProc.terminate()
        # self.mazeFast.terminate()
        # self.mazePID.terminate()
        self.root.destroy()
        sys.exit(0)
        
    def createMainFrame(self):
        '''creates main frame to attach all widgets'''
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=1)
        self.createMenuFrame(self.root)
        
        self.dataFrame = self.createDataFrame()
        # camLabel = self.createCamLabel()
        self.timerFrame = self.createTimerFrame()
        # animFrame = self.createAnimationFrame()

        self.dataFrame.grid(column=0, row=1, padx=10, pady=10, sticky='nswe')
        # camLabel.grid(column=0, row=2, padx=5, pady=5)
        self.timerFrame.grid(column=0, row=0, padx=10, pady=10, sticky='nswe')
        # animFrame.grid(column=0, row=2, padx=10, pady=10, sticky='we')
        
    def mainFrameOneColumn(self):
        self.root.geometry('500x550')
        self.root.columnconfigure(0, weight=1)
        self.root.columnconfigure(1, weight=0)
        self.root.rowconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=1)
        self.dataFrame.grid(column=0, row=1, padx=10, pady=10, sticky='nswe')
        # camLabel.grid(column=0, row=2, padx=5, pady=5)
        self.timerFrame.grid(column=0, row=0, padx=10, pady=10, sticky='nswe')
        # animFrame.grid(column=0, row=2, padx=10, pady=10, sticky='we')
        
    def mainFrameTwoColumns(self):
        self.root.geometry('730x270')
        self.root.columnconfigure(0, weight=1)
        self.root.columnconfigure(1, weight=1)
        self.root.rowconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=0)
        # self.createMenuFrame(self.root)
        
        self.dataFrame.grid(column=1, row=0, padx=10, pady=10, sticky='nswe')
        # camLabel.grid(column=0, row=2, padx=5, pady=5)
        self.timerFrame.grid(column=0, row=0, padx=10, pady=10, sticky='nswe')
        # animFrame.grid(column=0, row=2, padx=10, pady=10, sticky='we')
        
    def createCanvas(self, mainFrame):
        '''creates canvases'''
        
        canvas = Canvas(mainFrame)
        canvas.pack(side=LEFT, fill=BOTH, expand=1)
        return canvas 
    
    def createAnimationFrame(self): 
        frame = Frame(self.root, background='#ffffff', highlightbackground='#000000', 
            highlightthickness=1, width=5000, height=5000, padx=5, pady=5)
        canvas = FigureCanvasTkAgg(self.figure, frame)
        canvas.draw()
        canvas.get_tk_widget().pack(side=TOP, fill=BOTH, expand=True)
        return frame
       
    def createCamLabel(self):
        label = Label(self.root, background='#ffffff', padx=5, pady=5)
        return label
    
    def createDataFrame(self):
        '''frame for displaying parameters'''

        frame = Frame(self.root, background='#ffffff', highlightbackground='#000000', 
            highlightthickness=1, padx=10, pady=10)
        # canvas.create_window((0, 0), window=frame, anchor='nw')

        self.addLabel(frame)
        return frame
    
    def createTimerFrame(self):
        frame = Frame(self.root, background='#ffffff', highlightbackground='#000000', 
            highlightthickness=1, padx=10, pady=10)
        
        self.addTimerLabel(frame)
        return frame
        
    def createVideoFrame(self, canvas):
        frame = Frame(canvas, background='#ffffff', highlightbackground='#000000', 
            highlightthickness=1, padx=10, pady=10)
        self.videoplayer = TkinterVideo(master=frame, scaled=True)
        self.videoplayer.load(r"openingBanner.mp4")
        self.videoplayer.play()
        self.videoplayer.pack(side=TOP, fill=BOTH, expand=True)
        return frame
        
    def run(self):
        dataFrame = self.DataFrame
        self.addAnimation()
        self.addLabel(dataFrame=dataFrame)
        
        
    def addAnimation(self):
        '''adding animation to the AnimateFrame'''
        canvas = FigureCanvasTkAgg(self.figure, self.mainFrame)
        canvas.draw()
        canvas.get_tk_widget().pack(side=TOP, fill=BOTH, expand=True)
        
        
    def toggleTimer(self):
        if self.stopTimer:  #Timer running
            self.stopTimer = False
            self.timerButton.config(text="Stop timer")
            self.runTimer()
            
        else:   #Stop timer
            self.stopTimer = True
            self.timerButton.config(text="Start timer")
            self.timeTaken = self.timer.cget("text")
            self.L1.config(text=self.timeTaken)
            
    def runTimer(self):
        if self.stopTimer:
            return
        self.elapsed_time += 10
        minutes, seconds = divmod(self.elapsed_time // 1000, 60)
        milliseconds = self.elapsed_time % 1000
        self.timer.config(text=f"{minutes:02d}:{seconds:02d}:{milliseconds:03d}")
        self.timer.after(10, self.runTimer) #10ms
        
    def resetTimer(self):
        self.running = True
        self.elapsed_time = 0
        self.timer.config(text="00:00:000")
      
    def addTimerLabel(self, dataFrame):
        dataFrame.columnconfigure(0, weight=1)
        
        self.stopTimer = True
        self.elapsed_time = 0
        
        self.timer = Label(dataFrame, text="00:00:000", background='#ffffff', font=("Arial",40))
        self.timer.grid(column=0, row=2, padx=5, pady=5, sticky='nswe')
        self.timerButton = Button(dataFrame, text="Start timer", background='#8fce00', command=self.toggleTimer, font=("Segoe UI",10))
        self.timerButton.grid(column=0, row=0, padx=5, pady=5, sticky='nswe')
        self.resetButton = Button(dataFrame, text="Reset timer", background='#fc1833', command=self.resetTimer, font=("Segoe UI",10))
        self.resetButton.grid(column=0, row=1, padx=5, pady=5, sticky='nswe')
     
    def camera(self):
        vid = cv2.VideoCapture(0) 
        while(True):
            ret, frame = vid.read()
            cv2.imshow('frame', frame)
            
            # the 'q' button is set as the
            # quitting button you may use any
            # desired button of your choice
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        # After the loop release the cap object
        vid.release()
        # Destroy all the windows
        cv2.destroyAllWindows()
        
    
    def camButton(self):
        cam = threading.Thread(target=self.camera, args=())
        cam.start()
        
    def fastMaze(self, stop):
        # maze = video_processing_test2.ProcessImage()
        # maze = video_processing_test2.live_feedV2()
        while True:
            print('thread running')
            os.system("python video_processing_test2.py")
            if stop():
                os.execl(sys.executable, sys.executable, *sys.argv)
                break
        
    def fastMazeButton(self):
        # mazeFast = threading.Thread(target=self.fastMaze, args=())
        self.stop_thread1 = False
        self.mazeFast = threading.Thread(target=self.fastMaze, args=((lambda : self.stop_thread1,)))
        self.mazeFast.start()
        # with open("video_processing_test2.py", "r", encoding="utf-8") as file:
            # exec(file.read())
        #file = live_feedV2()
        
    def terminateFast(self):
        # self.mazeFast.terminate()
        self.stop_thread1 = True
        # self.mazeFast.join()
        print('thread killed')
        os.execl(sys.executable, sys.executable, *sys.argv)

    def PIDmaze(self, stop):
        # maze = video_processing_test3.live_feedV2()
        while True:
            # print('thread running')
            os.system("sudo python3 video_processing_test3.py")
            if stop():
                break
        
    def PIDmazeButton(self):
        self.stop_thread2 = False
        self.mazePID = threading.Thread(target=self.PIDmaze, args=(lambda : self.stop_thread2,))
        # self.mazePID = multiprocessing.Process(target=self.PIDmaze, args=())
        self.mazePID.start()
        
    def terminatePID(self):
        # self.mazePID.terminate()
        self.stop_thread2 = True
        os.execl(sys.executable, sys.executable, *sys.argv)
        
    def hardCode(self, stop):
        while True:
            # print('thread running')
            os.system("sudo python3 hard_code.py")
            if stop():
                break
    def hardCodeButton(self):
        self.stop_thread3 = False
        self.hardCoded = threading.Thread(target=self.hardCode, args=(lambda : self.stop_thread3,))
        self.hardCoded.start()
    
    def terminateHard(self):
        self.stop_thread3 = True
        os.execl(sys.executable, sys.executable, *sys.argv)
      
    def newTerminalFast(self):
        command = "sudo python3 video_processing_test2.py"
        # call(['cmd', '/c', 'start', 'cmd', '/k', command]) #this is for Windows
        Popen(['gnome-terminal', '--', 'bash', '-c', command]) 
        
    def newTerminalPID(self):
        command = "sudo python3 video_processing_test3.py"
        Popen(['gnome-terminal', '--', 'bash', '-c', command]) 
        
    def newTerminalCoded(self):
        command = "sudo python3 hard_code_feedback.py"
        Popen(['gnome-terminal', '--', 'bash', '-c', command]) 
    
    def newTerminalMedFast(self):
        command = "sudo python3 video_processing_test2_medium.py"
        Popen(['gnome-terminal', '--', 'bash', '-c', command]) 
      
        
    def addLabel(self, dataFrame):
        '''adding a label'''
        
        ### Labels for data frame
        dataFrame.columnconfigure(0, weight=1)
        # dataFrame.columnconfigure(1, weight=1) 
        
        B1 = Button(dataFrame, text='Solve easy maze with proportional controller', background='#ffffff', 
                    command=self.newTerminalFast, height=2, font=("Segoe UI",10), bd = 3) 
        B1.grid(column=0, row=1, padx=5, pady=5,sticky='we')
        
        B4 = Button(dataFrame, text='Solve medium maze with proportional controller ', background='#ffffff', 
                    command=self.newTerminalCoded, height=2, font=("Segoe UI",10), bd = 3)
        B4.grid(column=0, row=2, padx=5, pady=5, sticky='we')
        
        # B5 = Button(dataFrame, text='Stop maze with proportional controller', background='#ffffff')
        # B5.grid(column=1, row=0, padx=5, pady=5, sticky='we')
        
        B2 = Button(dataFrame, text='Solve maze with PID controller', background='#ffffff', 
                    command=self.newTerminalPID, height=2, font=("Segoe UI",10), bd = 3)
        B2.grid(column=0, row=3, padx=5, pady=5, sticky='we')
        
        # B4 = Button(dataFrame, text='Stop maze with PID controller', background='#ffffff', command=self.terminatePID)
        # B4.grid(column=1, row=1, padx=5, pady=5, sticky='we')
        
        B3 = Button(dataFrame, text='Solve maze using feedback from ball position', background='#ffffff', 
                    command=self.newTerminalCoded, height=2, font=("Segoe UI",10), bd = 3)
        B3.grid(column=0, row=4, padx=5, pady=5, sticky='we')   
        
        # B6 = Button(dataFrame, text='Stop maze with hard coding', background='#ffffff', command=self.terminateHard)
        # B6.grid(column=1, row=2, padx=5, pady=5, sticky='we')

        self.L1 = Label(dataFrame, text="Total time taken", background='#ffffff', font=("Segoe UI",10))
        self.L1.grid(column=0, row=5, padx=10, pady=10,sticky='nswe')
        
        # L2 = Label(dataFrame, text="Solving method options", background='#ffffff', font=("Segoe UI",13,'underline'))
        # L2.grid(column=0, row=0, padx=5, pady=5)
        
    def startWindow(self):       
        self.startWin = Toplevel(self.root)
        self.startWin.configure(bg='#ffffff')
        self.startWin.title("Welcome!")
        
        banner = self.createVideoFrame(self.startWin)
        banner.grid(column=0, row=0, padx=10, pady=10, sticky='we')
        
        button = Button(self.startWin, text='     START     ', background='#ffffff', command= self.showRootWindow)
        button.grid(column=0, row=1, padx=10, pady=10, sticky='we')
        
        self.root.withdraw()
        
    def showRootWindow(self):
        self.root.deiconify()   
        self.startWin.withdraw() 
    
def main():
    figure = Figure()
    root = Tk()
    gui = Window(root, figure)
    
    rootFrame = gui.createMainFrame()
    
    # animation = Plot_graph(figure)
    # animation = FuncAnimation(figure, animation.update, frames=200, interval=1000)
    root.mainloop()
    
    
if __name__ == '__main__':
    main()    
        
