package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {

    private final PWMVictorSPX clawMotor;

    public Claw(int clawChannel) {
        clawMotor = new PWMVictorSPX(clawChannel);
    }

    // Turns on claw motor. Positive power out-takes, negative power intakes
    public void activateClaw(double power) {
        clawMotor.set(MathUtil.applyDeadband(power, 0.02));
    }

}