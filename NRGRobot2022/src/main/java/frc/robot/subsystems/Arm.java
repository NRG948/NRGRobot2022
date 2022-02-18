
package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ArmConstants;
import frc.robot.preferences.RobotPreferencesLayout;
import frc.robot.preferences.RobotPreferencesValue;
import frc.robot.preferences.RobotPreferences.BooleanValue;
import frc.robot.preferences.RobotPreferences.DoubleValue;

@RobotPreferencesLayout(groupName = "Arm", column = 3, row = 0, width = 2, height = 3, type = "Grid Layout")
public class Arm extends ProfiledPIDSubsystem {
    @RobotPreferencesValue
    public static final DoubleValue levelAngleOffset = new DoubleValue("Arm", "levelAngleOffset", 85.88);
    @RobotPreferencesValue
    public static final DoubleValue stowedAngle = new DoubleValue("Arm", "stowedAngle", 100);
    @RobotPreferencesValue
    public static final DoubleValue restingAngle = new DoubleValue("Arm", "restingAngle", -20);
    @RobotPreferencesValue
    public static final BooleanValue enableTab = new BooleanValue("Arm", "enableTab", false);
    @RobotPreferencesValue
    public static final DoubleValue kP = new DoubleValue("Arm", "kP", 1.0);
    @RobotPreferencesValue
    public static final DoubleValue kS = new DoubleValue("Arm", "kS", 1.0);
    @RobotPreferencesValue
    public static final DoubleValue kG = new DoubleValue("Arm", "kG", 1.0);
    @RobotPreferencesValue
    public static final DoubleValue kV = new DoubleValue("Arm", "kV", 0.5);
    @RobotPreferencesValue
    public static final DoubleValue kA = new DoubleValue("Arm", "kA", 0.1);

    private final DigitalInput restingPositionLimitSwitch = new DigitalInput(ArmConstants.kRestingPosChannel);
    private final DigitalInput scoringPositionLimitSwitch = new DigitalInput(ArmConstants.kScoringPosChannel);
    private final PWMVictorSPX m_motor = new PWMVictorSPX(ArmConstants.kMotorPort);
    private final DutyCycleEncoder m_encoder = new DutyCycleEncoder(ArmConstants.kEncoderChannel);
    private final ArmFeedforward m_feedforward = new ArmFeedforward(
            kS.getValue(), kG.getValue(), kV.getValue(), kA.getValue());

    /** Create a new ArmSubsystem. */
    public Arm() {
        super(new ProfiledPIDController(
                ArmConstants.kP,
                0,
                0,
                new TrapezoidProfile.Constraints(
                        ArmConstants.kMaxVelocityRadPerSecond,
                        ArmConstants.kMaxAccelerationRadPerSecSquared)),
                0);

        // Configure the encoder for REV-11-1271 Through Bore Encoder duty cycle and to
        // report angle in radians.
        m_encoder.setDutyCycleRange(ArmConstants.kEncoderMinimumDutyCycle, ArmConstants.kEncoderMaximumDutyCycle);
        m_encoder.setDistancePerRotation(ArmConstants.kEncoderDistancePerRotation);

        // Initialize the goal state to the arm's current position.
        setGoal(getRadians());
    }

    @Override
    protected void useOutput(double output, TrapezoidProfile.State setpoint) {
        // Calculate the feedforward from the sepoint
        double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
        // Add the feedforward to the PID output to get the motor output
        final double batteryVoltage = RobotController.getBatteryVoltage();

        m_motor.set((output + feedforward) / batteryVoltage);
    }

    @Override
    protected double getMeasurement() {
        return getRadians();
    }

    /** Returns the arm's current angle in radians. */
    private double getRadians() {
        return Math.toRadians(levelAngleOffset.getValue()) - m_encoder.getDistance();
    }

    /** Returns whether the arm is at its resting/acquiring position. */
    public boolean isAtRestingPosition() {
        return !restingPositionLimitSwitch.get() || getRadians() < Math.toRadians(restingAngle.getValue());
    }

    /** Returns whether the arm is at its stowed position. */
    public boolean isAtStowedPosition() {
        return !scoringPositionLimitSwitch.get() || getRadians() > Math.toRadians(stowedAngle.getValue());
    }

    /** Adds a tab to the Shuffleboard for Arm subsystem debugging. */
    public void addShuffleboardTab() {
        if (!enableTab.getValue()) {
            return;
        }

        ShuffleboardTab armTab = Shuffleboard.getTab("Arm");
        ShuffleboardLayout layout = armTab.getLayout("Arm", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 3);
        layout.addNumber("Angle", () -> Math.toDegrees(getRadians()));
        layout.addBoolean("Resting", () -> isAtRestingPosition()).withWidget(BuiltInWidgets.kBooleanBox);
        layout.addBoolean("Stowed", () -> isAtStowedPosition()).withWidget(BuiltInWidgets.kBooleanBox);
    }
}