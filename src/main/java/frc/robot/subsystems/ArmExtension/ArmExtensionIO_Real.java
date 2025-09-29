package frc.robot.subsystems.armextension;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import frc.robot.Constants.ArmConstants;

public class ArmExtensionIO_Real implements ArmExtensionIO {
        private PWMSparkMax motor;

        @Override
        public void updateInputs(ArmExtensionIOInputs inputs) {
                motor = new PWMSparkMax(ArmConstants.kArmExtend);
        }

        @Override
        public void go(double dutycyc) {
            motor.set(dutycyc);
        }
}
