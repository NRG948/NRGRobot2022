// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.preferences.RobotPreferencesLayout;
import frc.robot.preferences.RobotPreferencesValue;
import frc.robot.preferences.RobotPreferences.DoubleValue;
import frc.robot.utilities.ShuffleboardUtils;
  @RobotPreferencesLayout(groupName = "SwerveModule", column = 2, row = 0, width = 2, height = 3, type = "Grid Layout")

public class SwerveModule extends SubsystemBase {
  @RobotPreferencesValue
  public static DoubleValue driveP = new DoubleValue("SwerveModule", "driveP", 1.0);
  @RobotPreferencesValue
  public static DoubleValue driveKs = new DoubleValue("SwerveModule", "driveKs", 1.0);
  @RobotPreferencesValue
  public static DoubleValue driveKv = new DoubleValue("SwerveModule", "driveKv", 3.0);
  @RobotPreferencesValue
  public static DoubleValue driveKa = new DoubleValue("SwerveModule", "driveKa", 0);
  @RobotPreferencesValue
  public static DoubleValue turnP = new DoubleValue("SwerveModule", "turnP", 7.0);
  @RobotPreferencesValue
  public static DoubleValue turnKs = new DoubleValue("SwerveModule", "turnKs", 1.0);
  @RobotPreferencesValue
  public static DoubleValue turnKv = new DoubleValue("SwerveModule", "turnKv", 0.5);
  @RobotPreferencesValue
  public static DoubleValue turnKa = new DoubleValue("SwerveModule", "turnKa", 0);

  private static final double WHEEL_RADIUS = 0.047625; // Meters
  private static final int ENCODER_RESOLUTION = 2048; // Steps per Rev
  private static final double DRIVE_GEAR_RATIO = 8.14; // Gear ratio
  private static final double DRIVE_PULSES_PER_METER = (ENCODER_RESOLUTION * DRIVE_GEAR_RATIO)
      / (2 * WHEEL_RADIUS * Math.PI); // pulses per meter

  private static final double MODULE_MAX_ANGULAR_VELOCITY = SwerveDrive.MAX_ANGULAR_SPEED;
  private static final double MODULE_MAX_ANGULAR_ACCELERATION = 2 * Math.PI; // radians per second squared

  private final TalonFX driveMotor;
  private final TalonFX turningMotor;

  private final CANCoder turningEncoder;

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController drivePIDController = new PIDController(driveP.getValue(), 0, 0);

  // Gains are for example purposes only - must be determined for your own robot!
  private final ProfiledPIDController turningPIDController = new ProfiledPIDController(
      turnP.getValue(),
      0,
      0,
      new TrapezoidProfile.Constraints(
          MODULE_MAX_ANGULAR_VELOCITY, MODULE_MAX_ANGULAR_ACCELERATION));

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(
      driveKs.getValue(), driveKv.getValue(), driveKa.getValue());
  private final SimpleMotorFeedforward turnFeedforward = new SimpleMotorFeedforward(
      turnKs.getValue(), turnKv.getValue(), turnKa.getValue());

  private SwerveModuleState desiredState = new SwerveModuleState(0, new Rotation2d(0));

  private String moduleName;

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
   * and turning encoder.
   *
   * @param driveMotorChannel    CAN ID of the drive motor.
   * @param turningMotorChannel  CAN ID of the turning motor.
   * @param turningEncodeChannel CAN ID of the turning encoder
   */
  public SwerveModule(int driveMotorChannel, int turningMotorChannel, int turningEncodeChannel, String moduleName) {
    driveMotor = new TalonFX(driveMotorChannel);
    turningMotor = new TalonFX(turningMotorChannel);

    turningEncoder = new CANCoder(turningEncodeChannel);

    this.moduleName = moduleName;

    turningEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    turningPIDController.reset(Math.toRadians(turningEncoder.getAbsolutePosition()));

  }

  /** Resets the module. */
  public void reset() {
    stopMotors();
    turningPIDController.reset(Math.toRadians(turningEncoder.getAbsolutePosition()));
  }

  /** Returns the current state of the module. */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getWheelVelocity(), getWheelRotation2d());
  }

  /** Returns wheel velocity in meters per second. */
  public double getWheelVelocity() {
    // talonFX reports velocity in pulses per 100ms; multiply by 10 to convert to
    // seconds
    return (driveMotor.getSelectedSensorVelocity() * 10) / DRIVE_PULSES_PER_METER;
  }

  /** Returns the distance the wheel has travelled in meters. */
  public double getWheelDistance() {
    return driveMotor.getSelectedSensorPosition() / DRIVE_PULSES_PER_METER;
  }

  /** Returns the module state set by the last call to setDesiredState. */
  public SwerveModuleState getDesiredState() {
    return desiredState;
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees

    this.desiredState = desiredState;
    Rotation2d currentAngle = getWheelRotation2d();
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, currentAngle);

    // Calculate the drive output from the drive PID controller.
    final double driveOutput = drivePIDController.calculate(getWheelVelocity(), state.speedMetersPerSecond);

    final double driveFeedforward = this.driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput = turningPIDController.calculate(currentAngle.getRadians(), state.angle.getRadians());

    final double turnFeedforward = this.turnFeedforward.calculate(turningPIDController.getSetpoint().velocity);
    final double batteryVoltage = RobotController.getBatteryVoltage();

    driveMotor.set(ControlMode.PercentOutput, (driveOutput + driveFeedforward) / batteryVoltage);
    turningMotor.set(ControlMode.PercentOutput, (turnOutput + turnFeedforward) / batteryVoltage);
  }

  /** Stops the drive and turn motors */
  public void stopMotors() {
    driveMotor.set(ControlMode.PercentOutput, 0);
    turningMotor.set(ControlMode.PercentOutput, 0);

  }

  /**
   * Returns the current wheel angle in degrees. The range of values is
   * [-180..180].
   */
  public double getWheelAngle() {
    return turningEncoder.getAbsolutePosition();
  }

  /** Returns the current whell angle as a Rotation2d object. */
  public Rotation2d getWheelRotation2d() {
    return Rotation2d.fromDegrees(getWheelAngle());
  }

  /** Sets the driver motor power. */
  private void setDriveMotorPower(double power) {
    driveMotor.set(ControlMode.PercentOutput, power);
  }

  /** Sets the turn motor power. */
  private void setTurnMotorPower(double power) {
    turningMotor.set(ControlMode.PercentOutput, power);
  }

  /** Adds module widgets to the specified Shuffleboard tab. */
  public ShuffleboardLayout addShuffleBoardLayout(ShuffleboardTab tab) {
    ShuffleboardLayout layout = tab.getLayout(moduleName, BuiltInLayouts.kList);

    ShuffleboardUtils.addNumberSlider(layout, "Drive Motor", 0.0, power -> setDriveMotorPower(power));
    ShuffleboardUtils.addNumberSlider(layout, "Turn Motor", 0.0, power -> setTurnMotorPower(power));

    layout.add("Rotation", new Sendable() {

      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Gyro");
        builder.addDoubleProperty("Value", () -> getWheelAngle(), null);
      }

    }).withWidget(BuiltInWidgets.kGyro).withPosition(0, 0);

    return layout;
  }
}
