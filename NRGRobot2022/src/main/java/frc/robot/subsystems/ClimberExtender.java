package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberExtender extends SubsystemBase {
    // Large extending pistons controlled on one solenoid.
    private final DoubleSolenoid extenderPistons = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);

    public enum State {
        UP, DOWN;
    }

    public State getState() {
        return extenderPistons.get() == Value.kForward ? State.DOWN : State.UP;
    }

    public void setState(State state) {
        extenderPistons.set(state == State.UP ? Value.kForward : Value.kReverse);
    }

    public void toggleState() {
        setState(getState() == State.UP ? State.DOWN : State.UP);
    }

}