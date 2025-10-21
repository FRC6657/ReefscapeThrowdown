package frc.robot.subsystems.arm.extension;

import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import frc.robot.Constants.ArmConstants;

public class armExtensionIO_Real implements armExtensionIO {
        private PWMTalonSRX motor;

        @Override
        public void updateInputs(armExtensionIOInputs inputs) {
           motor = new PWMTalonSRX(ArmConstants.kArmExtend);
        }

        @Override
        public void setSpeed(double dutycyc) {
           motor.set(dutycyc);
        }
}
