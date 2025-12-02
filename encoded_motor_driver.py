from motor_driver import MotorDriver
from machine import Pin, Timer
from math import pi

class EncodedMotorDriver(MotorDriver):
    def __init__(self, driver_ids, encoder_ids) -> None:
        # driver_ids must be: (pwm_id, in1_id, in2_id)
        super().__init__(*driver_ids)
        
        self.enc_a_pin = Pin(encoder_ids[0], Pin.IN)
        self.enc_b_pin = Pin(encoder_ids[1], Pin.IN)
        
        # Quadrature interrupt handlers (4x decoding)
        self.enc_a_pin.irq(
            trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self._update_counts_a
        )
        self.enc_b_pin.irq(
            trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self._update_counts_b
        )
        
        self._enc_a_val = self.enc_a_pin.value()
        self._enc_b_val = self.enc_b_pin.value()
        self.encoder_counts = 0
        self.prev_counts = 0
        self.meas_ang_vel = 0.0
        self.meas_lin_vel = 0.0
        
        # Physical Constants
        self.meas_radius = 0.02  # m
        self.gear_ratio = 98.5    # i
        self.cpr = 28             # CPR
        self.vel_meas_freq = 100  # Hz
        
        # Timer for velocity measurement
        self.vel_meas_timer = Timer(
            freq=self.vel_meas_freq, mode=Timer.PERIODIC, callback=self.measure_velocity,
        )

    def _update_counts_a(self, pin):
        # Quadrature decoding logic for Pin A
        self._enc_a_val = pin.value()
        if self._enc_a_val == 1:
            self.encoder_counts += 1 if self._enc_b_val == 0 else -1
        else:
            self.encoder_counts -= 1 if self._enc_b_val == 0 else -1

    def _update_counts_b(self, pin):
        # Quadrature decoding logic for Pin B
        self._enc_b_val = pin.value()
        if self._enc_b_val == 1:
            self.encoder_counts -= 1 if self._enc_a_val == 0 else -1
        else:
            self.encoder_counts += 1 if self._enc_a_val == 0 else -1

    def reset_encoder_counts(self):
        self.encoder_counts = 0

    def measure_velocity(self, timer):
        # Calculates angular and linear velocity based on change in counts
        delta_counts = self.encoder_counts - self.prev_counts
        self.prev_counts = self.encoder_counts
        
        counts_per_sec = delta_counts * self.vel_meas_freq
        orig_rev_per_sec = counts_per_sec / self.cpr
        orig_rad_per_sec = orig_rev_per_sec * 2 * pi
        
        self.meas_ang_vel = orig_rad_per_sec / self.gear_ratio
        self.meas_lin_vel = self.meas_ang_vel * self.meas_radius


# TEST
if __name__ == "__main__":  # Test only the encoder part
    from time import sleep
    
    # SETUP
    # Motor A: driver_ids=(9, 11, 10), encoder_ids=(16, 17)
    # Motor B: driver_ids=(15, 13, 14), encoder_ids=(18, 19)
    
    # Instantiate EncodedMotorDriver for Channel A
    emd_a = EncodedMotorDriver(
        driver_ids=(7, 9, 8),
        encoder_ids=(16, 17),
    )  # channel A motor, encoder's green and yellow on GP16 and GP17
    
    # Instantiate EncodedMotorDriver for Channel B (your original motor)
    emd_b = EncodedMotorDriver(
        driver_ids=(15, 13, 14),
        encoder_ids=(18, 19),
    )  # channel B motor, encoder's green and yellow on GP18 and GP19
    
    STBY = Pin(12, Pin.OUT)
    STBY.off()

    # LOOP
    STBY.on()  # enable motor driver
    
    # Forwardly ramp up and down for BOTH motors
    print("Ramping Up Both Motors Forward")
    for i in range(100):
        duty_cycle = (i + 1) / 100
        
        emd_a.forward(duty_cycle)
        emd_b.forward(duty_cycle)
        
        print(f"DC: {int(duty_cycle*100)}%")
        print(f"Motor A: Ang Vel={emd_a.meas_ang_vel:.2f} rad/s, Lin Vel={emd_a.meas_lin_vel:.2f} m/s")
        print(f"Motor B: Ang Vel={emd_b.meas_ang_vel:.2f} rad/s, Lin Vel={emd_b.meas_lin_vel:.2f} m/s")
        
        sleep(4 / 100)
        
    print("Ramping Down Both Motors Forward")
    for i in reversed(range(100)):
        duty_cycle = (i + 1) / 100
        
        emd_a.forward(duty_cycle)
        emd_b.forward(duty_cycle)
        
        print(f"DC: {int(duty_cycle*100)}%")
        print(f"Motor A: Ang Vel={emd_a.meas_ang_vel:.2f} rad/s, Lin Vel={emd_a.meas_lin_vel:.2f} m/s")
        print(f"Motor B: Ang Vel={emd_b.meas_ang_vel:.2f} rad/s, Lin Vel={emd_b.meas_lin_vel:.2f} m/s")
        
        sleep(4 / 100)
        
    # Backwardly ramp up and down for BOTH motors
    print("Ramping Up Both Motors Backward")
    for i in range(100):
        duty_cycle = (i + 1) / 100
        
        emd_a.backward(duty_cycle)
        emd_b.backward(duty_cycle)
        
        print(f"DC: {int(duty_cycle*100)}%")
        print(f"Motor A: Ang Vel={emd_a.meas_ang_vel:.2f} rad/s, Lin Vel={emd_a.meas_lin_vel:.2f} m/s")
        print(f"Motor B: Ang Vel={emd_b.meas_ang_vel:.2f} rad/s, Lin Vel={emd_b.meas_lin_vel:.2f} m/s")
        
        sleep(4 / 100)
        
    print("Ramping Down Both Motors Backward")
    for i in reversed(range(100)):
        duty_cycle = (i + 1) / 100
        
        emd_a.backward(duty_cycle)
        emd_b.backward(duty_cycle)
        
        print(f"DC: {int(duty_cycle*100)}%")
        print(f"Motor A: Ang Vel={emd_a.meas_ang_vel:.2f} rad/s, Lin Vel={emd_a.meas_lin_vel:.2f} m/s")
        print(f"Motor B: Ang Vel={emd_b.meas_ang_vel:.2f} rad/s, Lin Vel={emd_b.meas_lin_vel:.2f} m/s")
        
        sleep(4 / 100)

    # Terminate
    emd_a.stop()
    emd_b.stop()

    print("Test Complete")
