from machine import Pin, PWM, Timer
import time

# Initialize the buttons
trigger_button = Pin(5, Pin.IN, Pin.PULL_DOWN)
trigger_button_state = False
power_level_button = Pin(7, Pin.IN, Pin.PULL_DOWN)
power_level_button_state = False
alt_mode_button = Pin(6, Pin.IN, Pin.PULL_DOWN)
alt_mode_button_state = False

# Initialize the servo PWM pin
pwm_servo = PWM(Pin(26))
pwm_servo.freq(50)

# Initialize the motor PWM pin
pwm_motor_speed = PWM(Pin(29))
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

# Counter for ALT_MODE LED effect
current_led = 0

# LED Count Direction
forward = True

# Set if toy mode is in alternate mode
alternate_mode = False

ALT_LIGHT_DELAY = 150  # milliseconds

# Debounce variables
DEBOUNCE_TIME = 200  # 50 ms debounce time
last_trigger_time = 0
last_power_level_time = 0
last_alt_mode_time = 0

def light_up_leds(level):
    for led in leds:
        led.value(0)
    for i in range(level + 1):
        leds[i].value(1)

def alt_light_effect():
    global current_led, forward
    for led in leds:
        led.value(0)
    leds[current_led].value(1)
    current_led += 1 if forward else -1
    if current_led == len(leds) - 1 or current_led == 0:
        forward = not forward
    time.sleep_ms(ALT_LIGHT_DELAY)  # Add delay to slow down the effect

def move_servo(timer=None):
    pwm_servo.duty_ns(VAL)
    servo_return_timer.init(mode=Timer.ONE_SHOT, period=SERVO_RETURN, callback=lambda t: pwm_servo.duty_ns(MIN))

def stop_motor(timer=None):
    pwm_motor_speed.duty_u16(3500)

def run_motor(level):
    motor_speed = power_levels[level][1]
    pwm_motor_speed.duty_u16(motor_speed)
    motor_timer.init(mode=Timer.ONE_SHOT, period=MOTOR_ON, callback=stop_motor)
    servo_timer.init(mode=Timer.ONE_SHOT, period=SERVO_ACTIVATE, callback=move_servo)

def debounce(last_time):
    current_time = time.ticks_ms()
    if time.ticks_diff(current_time, last_time) > DEBOUNCE_TIME:
        return True, current_time
    return False, last_time

def trigger_button_IRQHandler(pin):
    global trigger_button_state, last_trigger_time
    debounced, last_trigger_time = debounce(last_trigger_time)
    if debounced:
        trigger_button_state = True

def power_level_button_IRQHandler(pin):
    global power_level_button_state, last_power_level_time
    debounced, last_power_level_time = debounce(last_power_level_time)
    if debounced:
        power_level_button_state = True

def alt_mode_button_IRQHandler(pin):
    global alt_mode_button_state, alternate_mode, last_alt_mode_time
    debounced, last_alt_mode_time = debounce(last_alt_mode_time)
    if debounced:
        alternate_mode = not alternate_mode
        print("Now in ALT_MODE" if alternate_mode else "No longer in ALT_MODE")
        alt_mode_button_state = True

trigger_button.irq(trigger=Pin.IRQ_FALLING, handler=trigger_button_IRQHandler)
power_level_button.irq(trigger=Pin.IRQ_FALLING, handler=power_level_button_IRQHandler)
alt_mode_button.irq(trigger=Pin.IRQ_FALLING, handler=alt_mode_button_IRQHandler)

# Set non-active motor speed
pwm_motor_speed.duty_u16(3500)

# Create timer objects
motor_timer = Timer(-1)
servo_timer = Timer(-1)
servo_return_timer = Timer(-1)

light_up_leds(current_level)

while True:
    if alternate_mode:
        alt_light_effect()
        if trigger_button_state:
            light_up_leds(current_level)
            pwm_motor_speed.duty_u16(power_levels[current_level][1])
            while trigger_button.value():
                time.sleep(0.01)
            trigger_button_state = False
            move_servo()
            time.sleep(0.1)
            stop_motor()
    else:
        light_up_leds(current_level)  # Add this to ensure LEDs show correct level when not in alt mode

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

    time.sleep(0.01)