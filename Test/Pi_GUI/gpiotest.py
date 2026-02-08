import RPi.GPIO as GPIO
import time

# Set GPIO mode
GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)

# Create PWM object on GPIO 18, 50Hz frequency
pwm = GPIO.PWM(18, 50)  # 50Hz for servo control
pwm.start(0)

try:
    print("Testing PWM output...")
    # Gradually increase duty cycle from 0% to 100%
    for dc in range(0, 101, 10):
        pwm.ChangeDutyCycle(dc)
        print(f"Duty Cycle: {dc}%")
        time.sleep(0.5)
finally:
    # Clean up GPIO resources
    pwm.stop()
    GPIO.cleanup()
    print("Test complete")

