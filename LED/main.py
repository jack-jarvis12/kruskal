from gpiozero import PWMLED, Button
from signal import pause

# Define the PWMLED and Button pins
led = PWMLED(17)  # GPIO pin 17 for PWMLED
button = Button(3)  # GPIO pin 3 for Button (updated)

# State tracking for the LED mode
pulsing = True

def toggle_led():
	global pulsing
	pulsing = not pulsing
	if not pulsing:
		led.value = 0.1
	else:
		led.pulse(fade_in_time=0.4, fade_out_time=0.6)  # Full brightness (on)

# Bind button press to the toggle function
button.when_pressed = toggle_led

toggle_led()
pause()  # Keep the program running
