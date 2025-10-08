package frc.robot.subsystems.armextension;

import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import frc.robot.Constants.ArmConstants;

public class ArmExtensionIO_Real implements ArmExtensionIO {
        private PWMTalonSRX motor;

        @Override
        public void updateInputs(ArmExtensionIOInputs inputs) {
           motor = new PWMTalonSRX(ArmConstants.kArmExtend);
        }

        @Override
        public void setSpeed(double dutycyc) {
           motor.set(dutycyc);
        }
}
