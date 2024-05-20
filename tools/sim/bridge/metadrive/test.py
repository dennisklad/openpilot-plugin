import vgamepad as vg
import time
from random import randrange

def press_circle(seconds):
    print("press")
    gamepad.press_button(vg.DS4_BUTTONS.DS4_BUTTON_CIRCLE)
    gamepad.update()
    time.sleep(seconds)
    
    print("release")
    gamepad.release_button(vg.DS4_BUTTONS.DS4_BUTTON_CIRCLE)
    gamepad.update()
    time.sleep(seconds)

def move_left_stick(x_pos, y_pos):
    gamepad.left_joystick_float(x_value, y_value)
    gamepad.update()
    time.sleep(2)
    
gamepad = vg.VDS4Gamepad()


while 1:
    inp = input('wasd')
    
    if inp == 'w':
        gamepad.left_joystick_float(0, -1)
        gamepad.update()
        
    if inp == 's':
        gamepad.left_joystick_float(0, 1)
        gamepad.update()
        
    if inp == 'a':
        gamepad.left_joystick_float(-1, 0)
        gamepad.update()
        
    if inp == 'd':
        gamepad.left_joystick_float(1, 0)
        gamepad.update()


while True:
    
    x_value = randrange(-10, 10) / 10
    y_value = randrange(-10, 10) / 10
    print(f"left-stick {x_value}; {y_value}")
    move_left_stick(x_value, y_value)
    
    print('1')
    gamepad.left_trigger_float(1)
    time.sleep(1)
    
    print('-1')
    gamepad.left_trigger_float(-1)
    time.sleep(1)
    
    print('1')
    gamepad.right_trigger_float(1)
    time.sleep(1)
    
    print('-1')
    gamepad.right_trigger_float(-1)
    time.sleep(1)
    
    press_circle(1)
    
    print('Button Trigger')
    gamepad.press_button(vg.DS4_BUTTONS.DS4_BUTTON_TRIGGER_LEFT)
    time.sleep(3)
    
    print('Button Trigger')
    gamepad.release_button(vg.DS4_BUTTONS.DS4_BUTTON_TRIGGER_LEFT)
    time.sleep(2)
    
    print('Button Trigger')
    gamepad.press_button(vg.DS4_BUTTONS.DS4_BUTTON_SHOULDER_LEFT)
    time.sleep(3)
    
    print('Button Trigger')
    gamepad.release_button(vg.DS4_BUTTONS.DS4_BUTTON_SHOULDER_LEFT)
    time.sleep(2)
