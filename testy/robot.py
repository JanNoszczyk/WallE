# https://www.electronicshub.org/raspberry-pi-l298n-interface-tutorial-control-dc-motor-l298n-raspberry-pi/
import RPi.GPIO as GPIO
from IOPi import IOPi      
from time import sleep


class Motor:
    def __init__(self, bus, en, inA, inB):
        self.bus = bus
        self.en = en
        self.inA = inA
        self.inB = inB

        self.speed = 100

        # Setup digital pins
        self.bus.set_pin_direction(self.inA, 0)
        self.bus.set_pin_direction(self.inB, 0)
        self.bus.write_pin(self.inA, 0)
        self.bus.write_pin(self.inB, 0)

        # Setup PWM pin
        GPIO.setup(self.en, GPIO.OUT)
        self.pwm = GPIO.PWM(self.en, 1000)
        self.pwm.start(self.speed)
    
    def forward(self, power):
        self.pwm.ChangeDutyCycle(power)
        self.bus.write_pin(self.inA, 1)
        self.bus.write_pin(self.inB, 0)
        
    def backward(self, power):
        self.pwm.ChangeDutyCycle(power)
        self.bus.write_pin(self.inA, 0)
        self.bus.write_pin(self.inB, 1)
    
    def stop(self):
        self.bus.write_pin(self.inA, 0)
        self.bus.write_pin(self.inB, 0)


class WallE:
    def __init__(self):
        self.enA = 17
        self.in1, self.in2 = 5, 3
        # Left motors
        self.enB = 27
        self.in3, self.in4 = 7, 9

        self.bus = IOPi(0x21)
        GPIO.setmode(GPIO.BCM)

        self.motor_right = Motor(self.bus, self.enA, self.in1, self.in2)
        self.motor_left = Motor(self.bus, self.enB, self.in3, self.in4)
        

    def set_speeds(self, power_left, power_right):
        power_left = (power_left)
        power_right = (power_right)

        if power_left < 0:
            self.motor_left.backward(-power_left)
        else:
            self.motor_left.forward(power_left)

        if power_right < 0:
            self.motor_right.backward(-power_right)
        else:
            self.motor_right.forward(power_right)


    def stop_motors(self):
        self.motor_left.stop()
        self.motor_right.stop()





# class MotorController:
#     def __init__(self):
#         # Right motors
#         self.enA = 17
#         self.in1, self.in2 = 5, 3
#         # Left motors
#         self.enB = 27
#         self.in3, self.in4 = 7, 9

#         self.speed = 100
#         self.turn_ratio = 0.8

#         self.bus = IOPi(0x21)

#         GPIO.setmode(GPIO.BCM)

#         # Right Motors 
#         self.bus.set_pin_direction(self.in1, 0)
#         self.bus.set_pin_direction(self.in2, 0)
#         self.bus.write_pin(self.in1, 0)
#         self.bus.write_pin(self.in2, 0)

#         GPIO.setup(self.enA,GPIO.OUT)
#         self.p_right = GPIO.PWM(self.enA, 1000)
#         self.p_right.start(self.speed)

#         # Left Motors
#         self.bus.set_pin_direction(self.in3, 0)
#         self.bus.set_pin_direction(self.in4, 0)
#         self.bus.write_pin(self.in3, 0)
#         self.bus.write_pin(self.in4, 0)

#         GPIO.setup(self.enB,GPIO.OUT)
#         self.p_left = GPIO.PWM(self.enB, 1000)
#         self.p_left.start(self.speed)

        
    
#     def set_speed(self, speed_right, speed_left):
#         self.p_right.ChangeDutyCycle(speed_right)
#         self.p_left.ChangeDutyCycle(speed_left)
    
#     def forward(self):
#         self.set_speed(self.speed, self.speed)
#         self.bus.write_pin(self.in1, 1)
#         self.bus.write_pin(self.in2, 0)
#         self.bus.write_pin(self.in3, 1)
#         self.bus.write_pin(self.in4, 0)

#     def reverse(self):
#         self.set_speed(self.speed, self.speed)
#         self.bus.write_pin(self.in1, 0)
#         self.bus.write_pin(self.in2, 1)
#         self.bus.write_pin(self.in3, 0)
#         self.bus.write_pin(self.in4, 1)

#     def stop(self):
#         self.bus.write_pin(self.in1, 0)
#         self.bus.write_pin(self.in2, 0)
#         self.bus.write_pin(self.in3, 0)
#         self.bus.write_pin(self.in4, 0)
    
#     def turn_left(self):
#         s = self.speed*self.turn_ratio
#         self.set_speed(self.speed, s)
        
#         self.bus.write_pin(self.in1, 1)
#         self.bus.write_pin(self.in2, 0)
#         self.bus.write_pin(self.in3, 1)
#         self.bus.write_pin(self.in4, 0)
    
#     def turn_right(self):
#         s = self.speed*self.turn_ratio
#         self.set_speed(s, self.speed)

#         self.bus.write_pin(self.in1, 1)
#         self.bus.write_pin(self.in2, 0)
#         self.bus.write_pin(self.in3, 1)
#         self.bus.write_pin(self.in4, 0)

# m = MotorController()
# m.forward()
# sleep(2)
# # m.reverse()
# # sleep(10)
# m.turn_left()
# sleep(2)
# m.turn_right()
# sleep(2)
# m.stop()

# GPIO.cleanup()

        
