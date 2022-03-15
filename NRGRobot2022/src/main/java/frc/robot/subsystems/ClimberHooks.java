package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.preferences.RobotPreferencesValue;
import frc.robot.preferences.RobotPreferences.DoubleValue;
// import frc.robot.preferences.RobotPreferencesLayout;

public class ClimberHooks extends SubsystemBase {
    // TODO: Figure out column, row, width, and height for widget
    // @RobotPreferencesLayout(groupName = "ClimberModule", column = 2, row = 0, width = 2, height = 3, type = "Grid Layout")
    @RobotPreferencesValue
    public static DoubleValue climbingPower = new DoubleValue("ClimberModule", "Climbing Power", 0.5);

    private final PWMVictorSPX climberMotor;
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
        climberMotor = new PWMVictorSPX(0); // TODO: change this to a Falcon on CANbus
        piston1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
        piston2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);

        // The beam breaks will read TBD(true/false) when it engages the bar
        beamBreak1 = new DigitalInput(5);
        beamBreak2 = new DigitalInput(6);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    /** Runs the climber motor with a power set via a preferences value. */
    public void rotateMotor() {
        climberMotor.set(climbingPower.getValue());
    }

    /** Stops the climber motor. */
    public void stopMotor() {
        climberMotor.stopMotor();
    }

    // Do not use this method. Similar code needs to be placed into Climber commands.
    public void rotateTimedMotor(double d) {
        Timer time = new Timer();
        time.start();
        while (!time.hasElapsed(d)) {
            climberMotor.set(climbingPower.getValue());
        }
        time.stop();
    }

    /** Returns true iff the climber hook is latched on a bar. */
    public boolean isHookLatched(HookSelection hook) {
        if (hook.equals(HookSelection.HOOK_1)) {
            return beamBreak1.get();
        } else {
            return beamBreak2.get();
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
}