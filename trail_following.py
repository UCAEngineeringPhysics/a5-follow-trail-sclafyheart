from machine import Pin
from math import pi, radians
import time 

from encoded_motor_driver import EncodedMotorDriver

# Assisted by Gemini for project development and debugging.

class TrailFollowerRobot:
    
    AXLE_LENGTH_M = 0.115          # L (Distance between wheel centers) 
    DEFAULT_SPEED = 0.4
    SPIN_SPEED = 0.35
    STRAIGHT_P_GAIN = 0.005       # Proportional gain for straight driving correction

    def __init__(self, emd_l: EncodedMotorDriver, emd_r: EncodedMotorDriver):
        self.motor_l = emd_l
        self.motor_r = emd_r
        
        self.R = emd_l.meas_radius
        self.I = emd_l.gear_ratio
        self.CPR = emd_l.cpr
        
        print(f"Robot Initialized (R={self.R}m, L={self.AXLE_LENGTH_M}m)")
        
    def _calculate_straight_counts(self, distance_m):
        """
        Calculates the target encoder counts for straight movement.
        Formula: C = (d / (2 * pi * r)) * i * CPR
        """
        # 2 * pi * r is correctly grouped 
        revolutions = distance_m / (2 * pi * self.R)
        return round(revolutions * self.I * self.CPR)

    def _calculate_spin_counts(self, angle_rad):
        """
        Calculates the target encoder counts for a spin turn.
        Formula: C = (theta * L) / (4 * pi * r) * i * CPR
        """
        revolutions = (angle_rad * self.AXLE_LENGTH_M) / (4 * pi * self.R)
        return round(revolutions * self.I * self.CPR)

    def _drive_to_counts(self, target_counts_l, target_counts_r, base_speed, straight_correction):
        self.motor_l.reset_encoder_counts()
        self.motor_r.reset_encoder_counts()
        
        dir_l = self.motor_l.forward if target_counts_l >= 0 else self.motor_l.backward
        dir_r = self.motor_r.forward if target_counts_r >= 0 else self.motor_r.backward
        
        abs_target_l = abs(target_counts_l)
        abs_target_r = abs(target_counts_r)
        
        # Start motors at base speed immediately
        dir_l(base_speed)
        dir_r(base_speed)
        
        initial_time = time.ticks_ms()
        
        while abs(self.motor_l.encoder_counts) < abs_target_l or abs(self.motor_r.encoder_counts) < abs_target_r:
            
            speed_l, speed_r = base_speed, base_speed
            
            if straight_correction:
                # PID correction for driving straight
                error = abs(self.motor_l.encoder_counts) - abs(self.motor_r.encoder_counts)
                correction = error * self.STRAIGHT_P_GAIN
                speed_l = max(0, min(1.0, base_speed - correction))
                speed_r = max(0, min(1.0, base_speed + correction))
            
            final_speed_l = speed_l if abs(self.motor_l.encoder_counts) < abs_target_l else 0
            final_speed_r = speed_r if abs(self.motor_r.encoder_counts) < abs_target_r else 0

            # Execute commands
            (dir_l(final_speed_l) if final_speed_l > 0 else self.motor_l.stop())
            (dir_r(final_speed_r) if final_speed_r > 0 else self.motor_r.stop())
            
            time.sleep(0.01)

        self.motor_l.stop()
        self.motor_r.stop()
        print("\nSegment movement complete.")
        
    def drive_straight(self, distance_m, speed=DEFAULT_SPEED):
        counts = self._calculate_straight_counts(distance_m)
        target_counts = counts if distance_m >= 0 else -counts
        self._drive_to_counts(target_counts, target_counts, speed, straight_correction=True)

    def spin_turn(self, angle_deg, direction, speed=SPIN_SPEED):
        angle_rad = radians(abs(angle_deg))
        counts = self._calculate_spin_counts(angle_rad)
        
        if direction.lower() == 'ccw' or direction.lower() == 'left':
            target_l, target_r = -counts, counts
        elif direction.lower() == 'cw' or direction.lower() == 'right':
            target_l, target_r = counts, -counts
        else:
            print("Invalid spin direction.")
            return

        self._drive_to_counts(target_l, target_r, speed, straight_correction=False)
        
    def pause(self, seconds):
        if seconds > 0:
            print(f"Pausing for {seconds} seconds...")
            time.sleep(seconds)

    # Optimized Helper Methods for Sequence
    def _drive_segment(self, distance_m, checkpoint_name, pause_s):
        print(f"Driving {distance_m}m to {checkpoint_name}.")
        self.drive_straight(distance_m)
        print(f"Reached {checkpoint_name}.")
        self.pause(pause_s)

    def _turn_segment(self, angle_deg, direction, pause_s):
        print(f"Turning {angle_deg} degrees {direction.upper()}.")
        self.spin_turn(angle_deg, direction)
        print("Turn complete.")
        self.pause(pause_s)

    # Main Trail Sequence
    def run_trail(self):
        print("--- Starting Trail Following Sequence ---")
        
        STBY_PIN_ID = 12
        STBY = Pin(STBY_PIN_ID, Pin.OUT)
        STBY.on() # Enable motor driver
        print(f"STBY Pin {STBY_PIN_ID} set HIGH.")

        # DIAGNOSTIC MOTOR TEST
        print("\n--- Diagnostic Motor Spin Test (0.5s at 0.9 speed) ---")
        self.motor_l.reset_encoder_counts()
        self.motor_r.reset_encoder_counts()
        
        # Both motors to move 
        self.motor_l.forward(0.9) 
        self.motor_r.backward(0.9)
        time.sleep(0.5)
        self.motor_l.stop()
        self.motor_r.stop()

        l_counts = abs(self.motor_l.encoder_counts)
        r_counts = abs(self.motor_r.encoder_counts)
        print(f"Test Result: L Counts={l_counts}, R Counts={r_counts}")
        
        # Diagnostic must pass for BOTH motors.
        if l_counts < 5 or r_counts < 5: 
            print("DIAGNOSTIC FAILED: Encoders are not registering movement on one or both motors.")
            STBY.off()
            return # Halt execution if test fails
        
        print("DIAGNOSTIC PASSED: Motors are spinning and encoders are working. Starting trail...")
        
        # 1. CP4 -> CP1 (0.75m, Stop 3s)
        self._drive_segment(0.75, "Checkpoint 1", 3)

        # 2. Spin 90 deg CCW (Stop 1s)
        self._turn_segment(90, 'ccw', 1)

        # 3. CP1 -> CP2 (0.5m, Stop 3s)
        self._drive_segment(0.5, "Checkpoint 2", 3)
        
        # 4. Spin 90 deg CW (Stop 1s)
        self._turn_segment(90, 'cw', 1)
        
        # 5. CP2 -> CP3 (0.5m, Stop 3s)
        self._drive_segment(0.5, "Checkpoint 3", 3)

        # 6. Spin 70 deg CCW (Stop 1s)
        self._turn_segment(70, 'ccw', 1)

        # 7. CP3 -> CP4 (0.5m, Shutdown)
        self._drive_segment(0.5, "Checkpoint 4", 0)
        
        # Shutdown sequence
        print("--- Trail Following Sequence Complete. Robot Shut Down. ---")
        STBY.off()
        self.motor_l.stop()
        self.motor_r.stop()


# Main Execution Block

if __name__ == "__main__":
    
    # Motor A (Left): driver_ids=(7, 9, 8), encoder_ids=(16, 17) 
    emd_a = EncodedMotorDriver(
        driver_ids=(7, 9, 8),
        encoder_ids=(16, 17),
    )
    
    # Motor B (Right): driver_ids=(15, 13, 14), encoder_ids=(18, 19) 
    emd_b = EncodedMotorDriver(
        driver_ids=(15, 13, 14),
        encoder_ids=(18, 19),
    )
    
    robot = TrailFollowerRobot(emd_a, emd_b) 
    # robot = TrailFollowerRobot(emd_b, emd_a)
    
    robot.run_trail()
