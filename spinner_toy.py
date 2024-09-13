from machine import Pin, PWM, Timer
import time

# Initialize the buttons
trigger_button = Pin(17, Pin.IN, Pin.PULL_DOWN)
trigger_button_state = False
power_level_button = Pin(16, Pin.IN, Pin.PULL_DOWN)
power_level_button_state = False

# Initialize the servo PWM pin
pwm_servo = PWM(Pin(13))
pwm_servo.freq(50)

# Initialize the motor PWM pin
pwm_motor_speed = PWM(Pin(15))
pwm_motor_speed.freq(50)

# Initialize the LEDs
led_a = Pin(0, Pin.OUT)
led_b = Pin(1, Pin.OUT)
led_c = Pin(2, Pin.OUT)
led_d = Pin(3, Pin.OUT)
led_e = Pin(4, Pin.OUT)
leds = [led_a, led_b, led_c, led_d, led_e]

# Servo positions
MIN = 600000
VAL = 850000
MAX = 1000000

# Timer Values in _ms
MOTOR_ON = 1000 # Motor on for this long
SERVO_ACTIVATE = 600 # Time after which the servo moves
SERVO_RETURN = 200 # Time from above when the servo moves back

# List of Tuples for power levels
power_levels = [('low', 4600), ('low-mid', 4800), ('mid', 5000), ('mid-high', 5250), ('high', 5500)]

# Initial servo position
pwm_servo.duty_ns(MIN)

# Initial level for power of motor
current_level = 0

'''
Function to light up LEDs
- First turns off all LEDS, then turns on the specified number.
'''
def light_up_leds(level):
    for led in leds:
        led.value(0)
    for i in range(level + 1):
        leds[i].value(1)

'''
Function to move the servo
'''
def move_servo(timer=None):
    pwm_servo.duty_ns(VAL)
    # Set another timer to move the servo back to MIN after SERVO_BACK time
    servo_return_timer.init(mode=Timer.ONE_SHOT, period=SERVO_RETURN, callback=lambda t: pwm_servo.duty_ns(MIN))

'''
Function to stop the motor
'''
def stop_motor(timer=None):
    pwm_motor_speed.duty_u16(3500)

'''
Function to run the motor
'''
def run_motor(level):
    motor_speed = power_levels[level][1]
    pwm_motor_speed.duty_u16(motor_speed)
    motor_timer.init(mode=Timer.ONE_SHOT, period=MOTOR_ON, callback=stop_motor)
    servo_timer.init(mode=Timer.ONE_SHOT, period=SERVO_ACTIVATE, callback=move_servo)

def trigger_button_IRQHandler(pin):
    global trigger_button_state
    trigger_button_state = True

def power_level_button_IRQHandler(pin):
    global power_level_button_state
    power_level_button_state = True

def trigger_released_IRQHandler(pin):
    global trigger_button_state
    trigger_button_state = True

trigger_button.irq(trigger=Pin.IRQ_FALLING, handler=trigger_button_IRQHandler)
power_level_button.irq(trigger=Pin.IRQ_FALLING, handler=power_level_button_IRQHandler)

trigger_button.irq(trigger=Pin.IRQ_RISING, handler=trigger_released_IRQHandler)

# Set non-active motor speed
pwm_motor_speed.duty_u16(3500)
#time.sleep(2)

# Create timer objects
motor_timer = Timer(-1)
servo_timer = Timer(-1)
servo_return_timer = Timer(-1)

alternate_mode = True
light_up_leds(current_level)

while True:
    if alternate_mode:
        while trigger_button_state:
            pwm_motor_speed.duty_u16(power_levels[current_level][1])
            if not trigger_button.value():
                trigger_button_state = False
                move_servo()
                time.sleep(0.1)
                stop_motor()
                break  # Exit the inner while loop, not the main loop

    if power_level_button_state:
        current_level = (current_level + 1) % len(leds)
        light_up_leds(current_level)
        print(power_levels[current_level][1])
        power_level_button_state = False
        time.sleep(0.1)

    if trigger_button_state:
        run_motor(current_level)
        trigger_button_state = False
        time.sleep(0.1)

    time.sleep(0.1)