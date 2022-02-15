package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {

    private final PWMVictorSPX clawMotor;

    public Claw(int clawChannel) {
        clawMotor = new PWMVictorSPX(clawChannel);
    }

    public void activateClaw(double power) {
        clawMotor.set(power);
    }

}