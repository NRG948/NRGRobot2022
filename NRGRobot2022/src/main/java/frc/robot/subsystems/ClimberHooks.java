package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.preferences.RobotPreferencesLayout;
import frc.robot.Constants.ClimberConstants;

public class ClimberHooks extends SubsystemBase {

    private final DoubleSolenoid piston1;
    private final DoubleSolenoid piston2;
    private final DigitalInput beamBreak1;
    private final DigitalInput beamBreak2;

    public enum State {
        OPEN, // kForward position
        CLOSED; // kReverse position
    }

    public enum HookSelection {
        HOOK_1, HOOK_2;
    }

    /** Creates a new ClimberHooks subsystem. **/
    public ClimberHooks() {
        // piston1 = new DoubleSolenoid(ClimberConstants.PH_ID, PneumaticsModuleType.REVPH, 2, 3);
        piston1 = null;

        piston2 = new DoubleSolenoid(ClimberConstants.PH_ID, PneumaticsModuleType.REVPH, 9, 5);

        // The beam breaks will read TBD(true/false) when it engages the bar
        beamBreak1 = new DigitalInput(6);
        beamBreak2 = new DigitalInput(5);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    /** Returns true iff the climber hook is latched on a bar. */
    public boolean isBarDetected(HookSelection hook) {
        if (hook.equals(HookSelection.HOOK_1)) {
            return !beamBreak1.get();
        } else {
            return !beamBreak2.get();
        }
    }

    /** Returns the current state of a climber hook piston. */
    public State getState(HookSelection hook) {
        return getPiston(hook).get() == Value.kForward ? State.OPEN : State.CLOSED;
    }

    /** Sets the state of a climber hook piston. */
    public void setState(State state, HookSelection hook) {
        getPiston(hook).set(state == State.OPEN ? Value.kForward : Value.kReverse);
    }

    /** Reverses the state of a climber hook piston. */
    public void toggleState(HookSelection hook) {
        setState(getState(hook) == State.OPEN ? State.CLOSED : State.OPEN, hook);
    }

    private DoubleSolenoid getPiston(HookSelection hook) {
        if (hook.equals(HookSelection.HOOK_1)) {
            return piston1;
        } else {
            return piston2;
        }
    }

    public void addShuffleboardLayout(ShuffleboardTab climberTab) {
        ShuffleboardLayout rotatorLayout = climberTab.getLayout("Hook", BuiltInLayouts.kGrid)
                .withPosition(2, 0)
                .withSize(2, 2);

        rotatorLayout.addBoolean("Beam Break 1", () -> !beamBreak1.get()).withWidget(BuiltInWidgets.kBooleanBox);
        rotatorLayout.addBoolean("Beam Break 2", () -> !beamBreak2.get()).withWidget(BuiltInWidgets.kBooleanBox);
    }
}