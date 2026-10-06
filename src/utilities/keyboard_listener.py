# keyboard_listener.py
from pynput import keyboard

s_toggle = False

def on_press(key):
    global s_toggle
    try:
        if key.char == 's':
            s_toggle = not s_toggle
            print(f"[KEY] 's' pressed → Sending {'ZERO' if s_toggle else 'NORMAL'} control")
    except AttributeError:
        pass

def start_keyboard_listener():
    listener = keyboard.Listener(on_press=on_press)
    listener.daemon = True
    listener.start()
    return listener
