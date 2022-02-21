
package frc.robot.subsystems;

import java.util.Map;

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
import frc.robot.utilities.ShuffleboardUtils;

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

    // The default values for the feed forward gain were estimated using http://reca.lc.
    // (https://www.reca.lc/arm?armMass=%7B%22s%22%3A15%2C%22u%22%3A%22lbs%22%7D&comLength=%7B%22s%22%3A21.5%2C%22u%22%3A%22in%22%7D&currentLimit=%7B%22s%22%3A40%2C%22u%22%3A%22A%22%7D&efficiency=100&endAngle=%7B%22s%22%3A45%2C%22u%22%3A%22deg%22%7D&iterationLimit=10000&motor=%7B%22quantity%22%3A1%2C%22name%22%3A%22MiniCIM%22%7D&ratio=%7B%22magnitude%22%3A100%2C%22ratioType%22%3A%22Reduction%22%7D&startAngle=%7B%22s%22%3A-23%2C%22u%22%3A%22deg%22%7D)
    @RobotPreferencesValue
    public static final DoubleValue kS = new DoubleValue("Arm", "kS", 1.0 /* V */);
    @RobotPreferencesValue
    public static final DoubleValue kG = new DoubleValue("Arm", "kG", 3.10 /* V */);
    @RobotPreferencesValue
    public static final DoubleValue kV = new DoubleValue("Arm", "kV", 1.96 /* V*s/rad */);
    @RobotPreferencesValue
    public static final DoubleValue kA = new DoubleValue("Arm", "kA", 0.17 /* V*s^2/rad */);

    private final DigitalInput restingPositionLimitSwitch = new DigitalInput(ArmConstants.kRestingPosChannel);
    private final DigitalInput scoringPositionLimitSwitch = new DigitalInput(ArmConstants.kScoringPosChannel);
    private final PWMVictorSPX m_motor = new PWMVictorSPX(ArmConstants.kMotorPort);
    private final DutyCycleEncoder m_encoder = new DutyCycleEncoder(ArmConstants.kEncoderChannel);
    private final ArmFeedforward m_feedforward = new ArmFeedforward(
            kS.getValue(), kG.getValue(), kV.getValue(), kA.getValue());

    /** Create a new ArmSubsystem. */
    public Arm() {
        super(new ProfiledPIDController(
                kP.getValue(),
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

    public void setMotorVoltage(double motorVoltage) {
        m_motor.setVoltage(motorVoltage);
    }

    public void stopMotor () {
        m_motor.stopMotor();
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
    public double getRadians() {
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

        ShuffleboardLayout control = armTab.getLayout("Control", BuiltInLayouts.kList).withPosition(2, 0).withSize(2, 2);
        ShuffleboardUtils.addNumberSlider(control, "Arm Motor", 0.0, voltage -> setMotorVoltage(voltage))
            .withProperties(Map.of("Min", -12.0, "Max", 12.0, "Block increment", 0.05));
    }
}