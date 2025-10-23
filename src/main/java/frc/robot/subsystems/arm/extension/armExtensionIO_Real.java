package frc.robot.subsystems.arm.extension;

import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import frc.robot.Constants.ArmConstants;

public class armExtensionIO_Real implements armExtensionIO {
        private PWMTalonSRX motor; //TODO should be TalonSRX since its being ran with CAN not PWM.

        @Override
        public void updateInputs(armExtensionIOInputs inputs) {
           motor = new PWMTalonSRX(ArmConstants.kArmExtend);
        }

        @Override
        public void setSpeed(double dutycyc) {
           motor.set(dutycyc);
        }
}
