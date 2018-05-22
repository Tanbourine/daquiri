import time
import os
import RPi.GPIO as GPIO
import ADS122C04_i2c as ads

ADC_ADR = 0x40
LED1 = 20
LED2 = 21
LED3 = 22


def setup():
    print("Begin setup.....")
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(LED1, GPIO.OUT)
    GPIO.setup(LED2, GPIO.OUT)
    GPIO.setup(LED3, GPIO.OUT)

    GPIO.output(LED1, GPIO.LOW)
    GPIO.output(LED2, GPIO.LOW)
    GPIO.output(LED3, GPIO.LOW)
    print(".....setup complete!")

def cleanup():
    print("\n-----------------\n")
    print("GPIO pins cleaned up!")
    GPIO.cleanup()

def blink(pin, repeat, delay):
    for i in range(repeat):
        GPIO.output(pin, GPIO.HIGH)
        time.sleep(delay)

        GPIO.output(pin, GPIO.LOW)
        time.sleep(delay)


def main():
    adc = ads.ADS122C04_i2c(ADC_ADR)
    delay = 0.1
    low = 0.3
    medium = 0.5
    high = 1
    alarm = 1.5

    setup()

    try:
        while(1):
            voltage = adc.read_conversion()
            os.system('clear')
            adc.print_voltage()

            if voltage > 0 and voltage < low:
                GPIO.output(LED1, GPIO.LOW)
                GPIO.output(LED2, GPIO.LOW)
                GPIO.output(LED3, GPIO.LOW)

            elif voltage > low and voltage < medium:
                GPIO.output(LED1, GPIO.HIGH)
                GPIO.output(LED2, GPIO.LOW)
                GPIO.output(LED3, GPIO.LOW)

            elif voltage > medium and voltage < high:
                GPIO.output(LED1, GPIO.HIGH)
                GPIO.output(LED2, GPIO.HIGH)
                GPIO.output(LED3, GPIO.LOW)

            elif voltage > high and voltage < alarm:
                GPIO.output(LED1, GPIO.HIGH)
                GPIO.output(LED2, GPIO.HIGH)
                GPIO.output(LED3, GPIO.HIGH)
            else:
                GPIO.output(LED1, GPIO.HIGH)
                GPIO.output(LED2, GPIO.HIGH)
                blink(LED3, 3, 0.1)

            time.sleep(delay)
    except:
        cleanup()





if __name__ == "__main__":
    main()

