import RPi.GPIO as GPIO
import time
import tkinter as tk
 
GPIO.setwarnings(False)
'''
IN1 - 21
IN2 - 22
IN3 - 23
IN4 - 24
'''

'''
Control 6 Motors Using L289N Motor Drivers
'''

def init():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(21,GPIO.OUT) #
    GPIO.setup(22,GPIO.OUT) #
    GPIO.setup(23,GPIO.OUT) #
    GPIO.setup(24,GPIO.OUT) #

def forward(tf):
    init()
    GPIO.output(21,True)
    GPIO.output(22,False)
    GPIO.output(23,False)
    GPIO.output(24,False)
    time.sleep(tf)
    GPIO.output(21,False)
    #GPIO.output(24,False)
    GPIO.cleanup()

def pivot_left(tf):
    init()
    GPIO.output(21,True)
    GPIO.output(22,True)
    GPIO.output(23,False)
    GPIO.output(24,False)
    time.sleep(tf)
    GPIO.output(22,False)
    GPIO.output(21,False)
    GPIO.cleanup()

def pivot_right(tf):
    init()
    GPIO.output(21,True)
    GPIO.output(22,False)
    GPIO.output(23,True)
    GPIO.output(24,False)
    time.sleep(tf)
    GPIO.output(21,False)
    GPIO.output(23,False)
    GPIO.cleanup()


def reverse(tf):
    init()
    GPIO.output(21,False)
    GPIO.output(22,False)
    GPIO.output(23,False)
    GPIO.output(24,True)
    time.sleep(tf)
    GPIO.output(24,False)
    #GPIO.output(24,False)
    GPIO.cleanup()

def key_input(event):
    #allowed_keys = ['w','a','s','d','e','q']
    init()
    print("Key: {}".format(event.char))
    key_press = event.char
    sleep_time = 0.25
    
    if key_press.lower() == 'w':
        forward(sleep_time)
        
    elif key_press.lower() == 's':
        reverse(sleep_time)
    
    elif key_press.lower() == 'a':
        pivot_left(sleep_time)
    
    elif key_press.lower() == 'd':
        pivot_right(sleep_time)
        
    elif key_press.lower() == 'q':
        GPIO.cleanup()
  
command = tk.Tk()
command.bind('<KeyPress>' , key_input)
command.mainloop()