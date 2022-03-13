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
        public static DoubleValue climbingPower = new DoubleValue("ClimberModule", "Climbing Power", -0.1);

    private final PWMVictorSPX climberMotor; 
    private final DoubleSolenoid piston1;
    private final DoubleSolenoid piston2;
    private final DigitalInput limitSwitch1;
    private final DigitalInput limitSwitch2;

    public enum State {
        OPEN, CLOSED;
    }

    public enum PistonSelection {
        PISTON_1, PISTON_2;
    }

    /** Creates a new ExampleSubsystem. **/
    public ClimberHooks() {
        climberMotor = new PWMVictorSPX(0);
        piston1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
        piston2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);
        limitSwitch1 = new DigitalInput(5);
        limitSwitch2 = new DigitalInput(6);
    }

    @Override
    public void periodic() {
    // This method will be called once per scheduler run

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation

    }

    public DoubleSolenoid getPiston(PistonSelection p) {
        if(p.equals(PistonSelection.PISTON_1)) {
            return piston1;
        }
        else {
            return piston2;
        }
    }

    public DoubleSolenoid getP1() {
        return piston1;
    }

    public DoubleSolenoid getP2() {
        return piston2;
    }

    public DigitalInput getLimitSwitch1() {
        return limitSwitch1;
    }

    public DigitalInput getLimitSwitch2() {
        return limitSwitch2;
    }

    public void rotateMotor() {
        climberMotor.set(climbingPower.getValue());
    }

    public void stopMotor() {
        climberMotor.set(0);
    }

    public void rotateTimedMotor(double d) {
        Timer time = new Timer();
        time.start();
        while (!time.hasElapsed(d)) {
          climberMotor.set(climbingPower.getValue());
        }
        time.stop();
    }

    /** Returns the current state of the acquirer pistons */
    public State getState(DoubleSolenoid p) {
        return p.get() == Value.kForward ? State.OPEN : State.CLOSED;
    }

    public void setState(State state, DoubleSolenoid p) {
        p.set(state == State.OPEN ? Value.kForward : Value.kReverse);
    }

    public void toggleState(DoubleSolenoid p) {
        setState(getState(p) == State.OPEN ? State.CLOSED : State.OPEN, p);
    }
}