__author__ = 'srkiyengar'



import dynamixel
import pygame
import joystick
from datetime import datetime
import logging
import logging.handlers

import serial
import time
import thread
import sys, optparse
import math
import string

import pwd
import os
import grp

DELTA_TICKS = 200
CAL_TICKS = 100
LOG_LEVEL = logging.DEBUG
LOG_FILENAME = 'Reflex_SF_movement' + datetime.now().strftime('%Y-%m-%d %H:%M:%S')

class reflex_sf():
    '''The class manages the calibration and movement of the fingers for pinch and grasp
    '''
    def __init__(self, usb_channel = '/dev/ttyUSB0', baudrate = 57600):
        dyn = dynamixel.USB2Dynamixel_Device(usb_channel, baudrate)
        l_limits = [0,13900,16700,14050, 16384]
        max_movement = 2300 # change this a value that travel more that half way for the finger to grasp
        u_limits = [0,l_limits[1]+ max_movement ,l_limits[2]-max_movement,l_limits[3]+ max_movement ,l_limits[4]-max_movement]
        self.finger = []
        self.finger.append(0) # finger starts with 1. Inserting 0 at the first list position
        for i in range(1,5,1):
            try:
                # using the USB2Dynamixel object try to send commands to each and receive information
                j= dynamixel.Robotis_Servo(dyn, i,"MX" )
            except:
                raise RuntimeError('Connection to Servo failure for servo number', i,'\n')
            temp = j.read_temperature()
            resol = j.read_resolution_divider()
            current_pos = j.read_current_position()
            goal_pos = j.get_goal_position()
            offset = j.read_offset()
            speed = 150
            j.set_speed(speed)
            if (i == 2 or i == 4):
                joint_state = -1
                u_limits[i] = l_limits[i] - max_movement
            else :
                joint_state = 1
                u_limits[i] = l_limits[i] + max_movement
            max_torque = j.read_max_torque()
            set_torque = j.read_set_torque()
            finger_parameters = {"servo":j, "temperature": temp, "resolution_divider": resol, "initial_position": current_pos,
                                 "goal_position":goal_pos,"multi_turn_offset":offset, "moving_speed":speed, "direction": 1,
                                 "lower_limit":l_limits[i],"upper_limit":u_limits[i],"rotation":joint_state,
                                 "max_torque":max_torque, "set_torque":set_torque}
            self.finger.append(finger_parameters)


    def is_finger_within_encoder_lower_limit(self, id, new_position):
        p = self.finger[id]["lower_limit"]
        rotation_mode = self.finger[id]["rotation"]
        if rotation_mode == 1:
           if new_position < p:
               my_logger.debug('Cannot exceed Finger{} start point {}'.format(id,p))
               return 0
           else:
                return 1
        elif rotation_mode == -1:
           if new_position > p:
               my_logger.debug('Cannot exceed Finger{} start point {}'.format(id,p))
               return 0
           else:
                return 1
        else:
            my_logger.debug('Finger{} - Rotation value {} not -1 or +1'.format(id,rotation_mode))
            return 0

    def finger_current_position(self,id):
        while (self.finger[id]["servo"].is_moving()):
            pass
        p = self.finger[id]["servo"].read_current_position()
        my_logger.info('Finger{} - Current Position {}'.format(id,p))
        return p

    def finger_load(self,id):
        load, rotation = self.finger[id]["servo"].read_and_convert_raw_load()
        return load, rotation

    def move_finger_delta(self, id, move_direction,increment): # direction +1 = finger closing; -1 = finger opening
        p = self.finger_current_position(id)
        q = self.finger[id]["rotation"]
        q = move_direction*q
        new_position = p + q*increment
        if self.is_finger_within_encoder_lower_limit(id,new_position) == 1:
            my_logger.info('Finger{} - Moving From Position {} to Position {}'.format(id,p,new_position))
            z = self.finger[id]["servo"].set_goal_position(new_position) # return data to make the program wait
            #time.sleep(2) # temp - modify based on speed and tick to give time for the motor to move
            p = self.finger_current_position(id)
        else:
            my_logger.info('Outside Limit Finger{} - Move From Position {} to Position {}'.format(id,p,new_position))
        return p

    def send_finger_to_start_position(self,id):
        p = self.finger_current_position(id)
        new_position = self.finger[id]["lower_limit"]
        my_logger.info('Moving Finger{} From Position {} to Start Position {}'.format(id,p,new_position))
        z = self.finger[id]["servo"].set_goal_position(new_position)
        time.sleep(5) # temp - modify based on speed and tick to give time for the motor to move
        p = self.finger_current_position(id)
        return p

    def tighten_fingers(self):
        how_much = DELTA_TICKS
        tighten = 1
        for i in range(1,4,1): #1,2,3
            my_logger.info('Finger{} - Before Tightening'.format(i))
            load, rotation = self.finger_load(i)
            my_logger.info('---> Load: {} Direction: {}'.format(load,rotation))
            j = self.move_finger_delta(i,tighten,how_much)
            my_logger.info('Finger{} - After tightening'.format(i))
            load, rotation = self.finger_load(i)
            my_logger.info('---> Load: {} Direction: {}'.format(load,rotation))

    def loosen_fingers(self):
        how_much = DELTA_TICKS
        tighten = -1    # loosen
        for i in range(1,4,1): #1,2,3
            my_logger.info('Finger{} - Before Loosening'.format(i))
            load, rotation = self.finger_load(i)
            my_logger.info('---> Load: {} Direction: {}'.format(load,rotation))
            j = self.move_finger_delta(i,tighten,how_much)
            my_logger.info('Finger{} - After Loosening'.format(i))
            load, rotation = self.finger_load(i)
            my_logger.info('---> Load: {} Direction: {}'.format(load,rotation))

    def spread_finger_1_and_2(self):
        how_much = DELTA_TICKS
        tighten = 1 # Spread
        i = 4 #servo 4
        j = self.move_finger_delta(i,tighten,how_much)
        return j

    def close_finger_1_and_2(self):
        how_much = DELTA_TICKS
        tighten = -1    # Close
        i = 4 #servo 4
        j = self.move_finger_delta(i,tighten,how_much)
        return j

# Define some colors
BLACK    = (   0,   0,   0)
WHITE    = ( 255, 255, 255)

# This is a simple class that will help us print to the screen
# It has nothing to do with the joysticks, just outputing the
# information.
class TextPrint:
    def __init__(self):
        self.reset()
        self.font = pygame.font.Font(None, 20)

    def Screenprint(self, screen, textString):
        textBitmap = self.font.render(textString, True, BLACK)
        screen.blit(textBitmap, [self.x, self.y])
        self.y += self.line_height

    def reset(self):
        self.x = 10
        self.y = 10
        self.line_height = 15

    def indent(self):
        self.x += 10

    def unindent(self):
        self.x -= 10

    def Yspace(self):
        self.y += 10


if __name__ == '__main__':



    # Set up a logger with output level set to debug; Add the handler to the logger
    my_logger = logging.getLogger('MyLogger')
    my_logger.setLevel(LOG_LEVEL)
    handler = logging.handlers.RotatingFileHandler(LOG_FILENAME, maxBytes=2000000, backupCount=5)
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    handler.setFormatter(formatter)
    my_logger.addHandler(handler)
    # end of logfile preparation Log levels are debug, info, warn, error, critical


    palm = reflex_sf() # Reflex object ready

    my_logger.info('Reflex_SF object created')
    for i in range(1,5,1):
        lowest_position = palm.finger[i]["lower_limit"]
        highest_position = palm.finger[i]["upper_limit"]
        init_position = palm.finger[i]["initial_position"]
        max_torque_setting = palm.finger[i]["max_torque"]
        allowable_torque = palm.finger[i]["set_torque"]
        my_logger.info('--- Finger {}:'.format(i))
        my_logger.info('       Max Torque --- {}'.format(max_torque_setting))
        my_logger.info('       Allowable Torque --- {}'.format(allowable_torque))
        my_logger.info('       Lower Limit Position --- {}'.format(lowest_position))
        my_logger.info('       Upper Limit Position --- {}'.format(highest_position))
        my_logger.info('       Initial Position {}'.format(init_position))

    pygame.init()

    # Set the width and height of the screen [width,height]
    size = [500, 700]
    screen = pygame.display.set_mode(size)

    pygame.display.set_caption("Reflex_SF Commands")

    # Used to manage how fast the screen updates
    clock = pygame.time.Clock()

    # Get ready to print
    textPrint = TextPrint()

    j_device = joystick.ExtremeProJoystick()
    # Get count of joystick
    Buttons = []
    Button_Set = []   # to remember button states that change during a for loop of get event
    Num_Buttons = j_device.buttons
    Axes = []   #mainly for the screen display
    max_val = []
    min_val = []
    Num_Axes = j_device.axes
    Num_Hats =j_device.hats

    JoyAxes = []    #multi-dimensional list
    JoyAxes.append([])  #Axis A0
    JoyAxes.append([])  #Axis A1
    JoyAxes.append([])  #Axis A2
    JoyAxes.append([])  #Axis A3
    count =[0,0,0,0]    #count of joystick position values for + and - of each Axis A0 to 3 (ends with a zero)

    # Flags fo Joystick axis movement to decide Reflex_SF actions inconjunction with buttons
    A0plus = False
    A0minus = False
    A1plus = False
    A1minus = False
    A2plus = False
    A2minus = False
    A3plus = False
    A3minus = False


    for i in range (Num_Buttons):
        Buttons.append(0)
        Button_Set.append(0)
    for i in range (Num_Axes):
        Axes.append(0.00)
        max_val.append(0.5)
        min_val.append(-0.5)


    Hat = (0,0)

    A3pluslimit = 0.99
    A3minuslimit =-0.99
    A3state, A3previous_state = 0,0

    #Loop until the user clicks the close button.
    done = False


    # -------- Main Program Loop -----------
    while done==False:
        Button_Event = 0   #to check if pygame.event.get() encountered a button event
        screen.fill(WHITE)
        textPrint.reset()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                    done = True
            elif event.type == pygame.JOYAXISMOTION:
                position = event.dict['value']
                k = event.dict['axis']
                Axes[k] = position
                if k == 3: # treating Axis 3 as a special case with 3 position
                    if(position > A3pluslimit):
                        A3previous_state = A3state
                        A3state = 1
                        if A3state != A3previous_state:
                            A3plus = True
                            A3minus = False
                            my_logger.info("Joystick Axis 3  at +1")
                    elif(position<A3minuslimit):
                        A3previous_state = A3state
                        A3state = 2
                        if A3state != A3previous_state:
                            A3plus = False
                            A3minus = True
                            my_logger.info("Joystick Axis 3 at -1")
                    else:
                        A3previous_state = A3state
                        A3state = 3
                        if A3state != A3previous_state:
                            A3plus = False
                            A3minus = False
                else:
                    if position > max_val[k]:
                        max_val[k] = position
                        JoyAxes[k].append(position)
                        count[k] += 1
                    elif position < min_val[k]:
                        min_val[k] = position
                        JoyAxes[k].append(position)
                        count[k] += 1
                    if count[k] > 0 :   #detecting 0 crossing
                        if(position==0)and (JoyAxes[k][count[k]-1]) !=0:
                            if min_val[k] < -0.5:
                                min_val[k] = -0.5
                                if k == 0:
                                    A0minus = True
                                    my_logger.info("Joystick Axis 0 -ive movement")
                                elif k == 1:
                                    A1minus = True
                                    my_logger.info("Joystick Axis 1 -ive movement")
                                elif k == 2:
                                    pass  #not going to use A2 as A1 and A0 inadvertently produce A2 movement
                            if max_val[k] > +0.5:
                                max_val[k] = 0.5
                                if k == 0:
                                    A0plus = True
                                    my_logger.info("Joystick Axis 0 +ive movement")
                                elif k == 1:
                                    A1plus = True
                                    my_logger.info("Joystick Axis 1 +ive movement")
                                elif k ==2:
                                    pass  #not going to use A2 as A1 and A0 inadvertently produce A2 movement
            elif event.type == pygame.JOYBUTTONDOWN:
                Button_Event = 1
                i = event.dict['button']    # button number
                Buttons[i] = 1
                Button_Set[i] = 1
                my_logger.debug("Button {} pressed".format(i))
            elif event.type == pygame.JOYBUTTONUP:
                Button_Event = 1
                i = event.dict['button']
                Buttons[i] = 0
                my_logger.debug("Button {} released".format(i))
            elif event.type == pygame.JOYHATMOTION:
                Hat = event.dict['value']
                my_logger.debug("Hat value: {}".format(str(Hat)))
            else:
                pass # ignoring other event types

        textPrint.Screenprint(screen, "Joystick name: {}".format(j_device.name))
        textPrint.Yspace()
        textPrint.Screenprint(screen, "Number of Axes: {}".format(Num_Axes))
        textPrint.indent()
        for i in range(Num_Axes):
            textPrint.Screenprint(screen, "Axis {} value: {:>6.3f}".format(i, Axes[i]))
        textPrint.unindent()
        textPrint.Yspace()
        textPrint.Screenprint(screen, "Number of Buttons: {}".format(Num_Buttons))
        textPrint.indent()
        for i in range(Num_Buttons):
            textPrint.Screenprint(screen, "Button {:>2} value: {}".format(i,Buttons[i]))
        textPrint.unindent()
        textPrint.Yspace()
        textPrint.Screenprint(screen, "Number of Hats: {}".format(Num_Hats) )
        textPrint.indent()
        textPrint.Screenprint(screen, "Hat value: {}".format(str(Hat)) )
        textPrint.unindent()

        #--------------------------------------------------------------------------------------------------
        #Processing Button events
        #--------------------------------------------------------------------------------------------------
        if Button_Event == 1:
            if (Button_Set[1] == 1 and Button_Set[7] == 1):
                my_logger.info("Buttons 1 and 7 pressed")
                finger_id = 1
                direction = 1   #1 = tighten
                increment = CAL_TICKS
                palm.move_finger_delta(finger_id,direction,increment)

            if (Button_Set[1] == 1 and Button_Set[6] == 1):
                my_logger.info("Buttons 1 and 6 pressed")
                finger_id = 1
                direction = -1   #-1 = loosen
                increment = CAL_TICKS
                palm.move_finger_delta(finger_id,direction,increment)

            if (Button_Set[2] == 1 and Button_Set[7] == 1):
                my_logger.info("Buttons 2 and 7 pressed")
                finger_id = 2
                direction = 1   #1 = tighten
                increment = CAL_TICKS
                palm.move_finger_delta(finger_id,direction,increment)

            if (Button_Set[2] == 1 and Button_Set[6] == 1):
                my_logger.info("Buttons 2 and 6 pressed")
                finger_id = 2
                direction = -1   #-1 = loosen
                increment = CAL_TICKS
                palm.move_finger_delta(finger_id,direction,increment)

            if (Button_Set[3] == 1 and Button_Set[7] == 1):
                my_logger.info("Buttons 3 and 7 pressed")
                finger_id = 3
                direction = 1   #1 = tighten
                increment = CAL_TICKS
                palm.move_finger_delta(finger_id,direction,increment)

            if (Button_Set[3] == 1 and Button_Set[6] == 1):
                my_logger.info("Buttons 3 and 6 pressed")
                finger_id = 3
                direction = -1   #-1 = loosen
                increment = CAL_TICKS
                palm.move_finger_delta(finger_id,direction,increment)

            if (Button_Set[4] == 1 and Button_Set[7] == 1):
                my_logger.info("Buttons 4 and 7 pressed")
                finger_id = 4
                direction = 1   #1 = tighten
                increment = CAL_TICKS
                palm.move_finger_delta(finger_id,direction,increment)

            if (Button_Set[4] == 1 and Button_Set[6] == 1):
                my_logger.info("Buttons 4 and 6 pressed")
                finger_id = 4
                direction = -1   #-1 = loosen
                increment = CAL_TICKS
                palm.move_finger_delta(finger_id,direction,increment)

            # Commands to send to Calibrated positions
            if ((Button_Set[1] == 1) and (Button_Set[10] == 1)):
                my_logger.info("Buttons 1 and 10 pressed - Sending Finger 1 to initial position")
                finger_id = 1
                palm.send_finger_to_start_position(finger_id)
                Buttons[1] = 0
                Buttons[10] = 0

            if ((Button_Set[2] == 1) and (Button_Set[10] == 1)):
                my_logger.info("Buttons 2 and 10 pressed - Sending Finger 2 to initial position")
                finger_id = 2
                palm.send_finger_to_start_position(finger_id)
                # in case the buttonup event is not captured
                Buttons[2] = 0
                Buttons[10] = 0

            if ((Button_Set[3] == 1) and (Button_Set[10] == 1)):
                my_logger.info("Buttons 3 and 10 pressed - Sending Finger 3 to initial position")
                finger_id = 3
                palm.send_finger_to_start_position(finger_id)
                # in case the buttonup event is not captured
                Buttons[3] = 0
                Buttons[10] = 0

            if ((Button_Set[4] == 1) and (Button_Set[10] == 1)):
                my_logger.info("Buttons 4 and 10 pressed - Sending Finger 4 to initial position")
                finger_id = 4
                palm.send_finger_to_start_position(finger_id)
                # in case the buttonup event is not captured
                Buttons[4] = 0
                Buttons[10] = 0

            for i in range(Num_Buttons):
                if Buttons[i] == 0:
                    Button_Set[i] = 0
                    #my_logger.info("Button {} Buttons {} and Button_Set {}".format(i,Buttons[i],Button_Set[i]))

        #--------------------------------------------------------------------------------------------------
        # end of Button event Processing
        #--------------------------------------------------------------------------------------------------

        # Joystick Processing

        if A3plus == True:
            if A1plus == True:
                my_logger.info("Tighten Fingers - A3plus and A1plus True")
                a = palm.tighten_fingers()
                A1plus = False
            elif A1minus == True:
                my_logger.info("Loosen Fingers - A3plus and A1minus True")
                b = palm.loosen_fingers()
                A1minus = False
        elif A3minus == True:
            if A0plus == True:
                my_logger.info("Spread Finger 1 and 2 apart - A3minus and A0plus True")
                h = palm.spread_finger_1_and_2()
                A0plus = False
            elif A0minus == True:
                my_logger.info("Bring Finger 1 and 2 together - A3minus and A0minus True")
                h = palm.close_finger_1_and_2()
                A0minus = False
        else:
            A0plus = False
            A0minus = False
            A1plus = False
            A1minus = False


        # reset flags

        A0plus = False
        A0minus = False
        A2plus = False
        A2minus = False
        #A3plus = False #They are more like switch positions
        #A3minus = False #They are more like switch positions
        # end of reset flags
    # ALL CODE TO DRAW SHOULD GO ABOVE THIS COMMENT

    # Go ahead and update the screen with what we've drawn.
        pygame.display.flip()

    # Limit to 20 frames per second OR 50 ms scan rate - 1000/20 = 50 ms Both display and checking of Joystick;
        clock.tick(20)

# Close the window and quit.
# If you forget this line, the program will 'hang' on exit if running from IDLE.

pygame.quit ()






