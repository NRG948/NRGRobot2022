// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {

  /*
   * 
   * Forward motion of drive wheel is the directions your fingers curl
   * when your thumb points in the direction of the bolt.
   * 
   */

  /* Swerve Module helper class */

  // Absolute position of encoders when the module wheel is pointed in the +X
  // direction
  double zeroFrontLeft = -2.197;
  double zeroFrontRight = 129.990;
  double zeroBackLeft = 88.066;
  double zeroBackRight = -70.048;

  private class Module {

    private static final double WHEEL_RADIUS = 0.047625; // Meters
    private static final int ENCODER_RESOLUTION = 2048; // Steps per Rev
    private static final double DRIVE_GEAR_RATIO = 8.14; // Gear ratio
    private static final double DRIVE_PULSES_PER_METER = (ENCODER_RESOLUTION * DRIVE_GEAR_RATIO)
        / (2 * WHEEL_RADIUS * Math.PI); // pulses per
    // meter

    private static final double MODULE_MAX_ANGULAR_VELOCITY = SwerveDrive.MAX_ANGULAR_SPEED;
    private static final double MODULE_MAX_ANGULAR_ACCELERATION = 2 * Math.PI; // radians per second squared

    private final TalonFX m_driveMotor;
    private final TalonFX m_turningMotor;

    // private final Encoder m_driveEncoder;
    private final CANCoder m_turningEncoder;

    /* TODO: Tune P ID for drive and turning PID controllers */
    // Gains are for example purposes only - must be determined for your own robot!
    private final PIDController m_drivePIDController = new PIDController(1, 0, 0);

    // Gains are for example purposes only - must be determined for your own robot!
    private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
        7,
        0,
        0,
        new TrapezoidProfile.Constraints(
            MODULE_MAX_ANGULAR_VELOCITY, MODULE_MAX_ANGULAR_ACCELERATION));

    // Gains are for example purposes only - must be determined for your own robot!
    private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
    private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

    private SwerveModuleState desiredState = new SwerveModuleState(0, new Rotation2d(0));

    private double turnPIDOutput = 0;
    private double turnFeedForwardOutput = 0;

    /**
     * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
     * and turning encoder.
     *
     * @param driveMotorChannel    CAN ID of the drive motor.
     * @param turningMotorChannel  CAN ID of the turning motor.
     * @param turningEncodeChannel CAN ID of the turning encoder
     */
    public Module(int driveMotorChannel, int turningMotorChannel, int turningEncodeChannel

    ) {
      m_driveMotor = new TalonFX(driveMotorChannel);
      m_turningMotor = new TalonFX(turningMotorChannel);

      m_turningEncoder = new CANCoder(turningEncodeChannel);

      m_turningEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
      // m_turningEncoder.config
      // Limit the PID Controller's input range between -pi and pi and set the input
      // to be continuous.
      m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
      m_turningPIDController.reset(Math.toRadians(m_turningEncoder.getAbsolutePosition()));

    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
      return new SwerveModuleState(getDriveMotorVelocity(),
          Rotation2d.fromDegrees(m_turningEncoder.getAbsolutePosition()));
    }

    public double getDriveMotorVelocity() {
      return m_driveMotor.getSelectedSensorVelocity() / DRIVE_PULSES_PER_METER;

    }

    public double getDriveMotorPosition() {
      return m_driveMotor.getSelectedSensorPosition() / DRIVE_PULSES_PER_METER;
    }

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
      Rotation2d currentAngle = Rotation2d.fromDegrees(m_turningEncoder.getAbsolutePosition());
      SwerveModuleState state = SwerveModuleState.optimize(desiredState, currentAngle);

      // Calculate the drive output from the drive PID controller.
      final double driveOutput = m_drivePIDController.calculate(getDriveMotorVelocity(), state.speedMetersPerSecond);

      final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

      // Calculate the turning motor output from the turning PID controller.
      final double turnOutput = m_turningPIDController.calculate(currentAngle.getRadians(), state.angle.getRadians());

      final double turnFeedforward = m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);
      final double batteryVolatage = RobotController.getBatteryVoltage();

      turnFeedForwardOutput = turnFeedforward;
      turnPIDOutput = turnOutput;

      m_driveMotor.set(ControlMode.PercentOutput, (driveOutput + driveFeedforward) / batteryVolatage);
      m_turningMotor.set(ControlMode.PercentOutput, (turnOutput + turnFeedforward) / batteryVolatage);
    }

    // Stops the Driving and Turning Motors
    public void stopMotors() {
      m_driveMotor.set(ControlMode.PercentOutput, 0);
      m_turningMotor.set(ControlMode.PercentOutput, 0);

    }

    public double getFeedForwardOutput() {
      return turnFeedForwardOutput;
    }

    public double getTurnPIDOutput() {
      return turnPIDOutput;
    }

    public double getAbsolutePosition() {
      return m_turningEncoder.getAbsolutePosition();
    }

    public double getRelativePosition() {
      return m_turningEncoder.getPosition();
    }

    public void setDriveMotorPower(double power) {
      m_driveMotor.set(ControlMode.PercentOutput, power);
    }

    public void setTurnMotorPower(double power) {
      m_turningMotor.set(ControlMode.PercentOutput, power);

    }
  }

  public static final double MAX_SPEED = 3.0; // 3 meters per second
  public static final double MAX_ANGULAR_SPEED = Math.PI; // 1/2 rotation per second
  public static final double MAX_ACCELERATION = 1.0; // TODO: find Max acceleration in meters per second squared

  public static double currentMaxSpeed = MAX_SPEED;
  public static double currentMaxAngularSpeed = MAX_ANGULAR_SPEED;

  // X and Y swaped
  private final Translation2d m_frontLeftLocation = new Translation2d(0.34925, 0.24765);
  private final Translation2d m_frontRightLocation = new Translation2d(0.34925, -0.24765);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.34925, 0.24765);
  private final Translation2d m_backRightLocation = new Translation2d(-0.34925, -0.24765);

  private final Module m_frontLeft = new Module(1, 2, 9);
  private final Module m_frontRight = new Module(3, 4, 10);
  private final Module m_backLeft = new Module(7, 8, 12);
  private final Module m_backRight = new Module(5, 6, 11);

  private final AHRS m_ahrs = new AHRS(SerialPort.Port.kMXP);

  public final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, getRotation2d());

  public SwerveDrive() {
    m_ahrs.reset();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    xSpeed = MathUtil.applyDeadband(xSpeed, 0.02) * currentMaxSpeed;
    ySpeed = MathUtil.applyDeadband(ySpeed, 0.02) * currentMaxSpeed;
    rot = MathUtil.applyDeadband(rot, 0.02) * currentMaxAngularSpeed;

    var swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    setModuleStates(swerveModuleStates);
  }

  public void setMaxSpeed(double speed) {
    currentMaxSpeed = MathUtil.clamp(speed, 0, MAX_SPEED);

  }

  public void setMaxAngularSpeed(double angularSpeed) {
    currentMaxAngularSpeed = MathUtil.clamp(angularSpeed, 0, MAX_ANGULAR_SPEED);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        getRotation2d(),
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_backLeft.getState(),
        m_backRight.getState());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, getRotation2d());
  }

  @Override
  public void periodic() {
    updateOdometry();
  }

  /** Returns the current orientation of the robot as a Rotation2d object */
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(-m_ahrs.getAngle());
  }

  /** Returns the current pose of the robot as a Pose2d object */
  public Pose2d getPose2d() {
    return m_odometry.getPoseMeters();
  }

  // Get the Absolute Turning Encoder Position of a Swerve Module
  public double getAbsoluteTurningEncoderPosition(int index) {
    switch (index) {
      case 0:
        return m_frontLeft.getAbsolutePosition();
      case 1:
        return m_frontRight.getAbsolutePosition();
      case 2:
        return m_backLeft.getAbsolutePosition();
      case 3:
        return m_backRight.getAbsolutePosition();
    }
    return 0;
  }

  // Get the Relative Turning Encoder Position of a Swerve Module
  public double getRelativeTurningEncoderPosition(int index) {
    switch (index) {
      case 0:
        return m_frontLeft.getRelativePosition();
      case 1:
        return m_frontRight.getRelativePosition();
      case 2:
        return m_backLeft.getRelativePosition();
      case 3:
        return m_backRight.getRelativePosition();
    }
    return 0;
  }

  // Sets the Swerve Module State of a Swerve Module
  public void setModuleState(int index, double speed, double angle) {
    Rotation2d rotation = Rotation2d.fromDegrees(angle);
    SwerveModuleState state = new SwerveModuleState(speed, rotation);
    switch (index) {
      case 0:
        m_frontLeft.setDesiredState(state);
        break;
      case 1:
        m_frontRight.setDesiredState(state);
        break;
      case 2:
        m_backLeft.setDesiredState(state);
        break;
      case 3:
        m_backRight.setDesiredState(state);
        break;
    }

  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, MAX_SPEED);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_backLeft.setDesiredState(desiredStates[2]);
    m_backRight.setDesiredState(desiredStates[3]);
  }

  // Stops all Swerve Drive Motors
  public void stopMotor() {
    m_frontLeft.stopMotors();
    m_frontRight.stopMotors();
    m_backLeft.stopMotors();
    m_backRight.stopMotors();

  }

  public void initShuffleboardTab() {
    ShuffleboardTab swerveDriveTab = Shuffleboard.getTab("Swerve Drive");

    ShuffleboardLayout swerveOdometry = swerveDriveTab.getLayout("Odometry", BuiltInLayouts.kGrid)
        .withPosition(0, 0)
        .withSize(2, 3);

    swerveOdometry.addNumber("Gyro", () -> getRotation2d().getDegrees());
    swerveOdometry.addNumber("X", () -> getPose2d().getX());
    swerveOdometry.addNumber("Y", () -> getPose2d().getY());
    swerveOdometry.addNumber("FR Encoder", () -> m_frontRight.getDriveMotorPosition());
    swerveOdometry.addNumber("FL Encoder", () -> m_frontLeft.getDriveMotorPosition());
    swerveOdometry.addNumber("BR Encoder", () -> m_backRight.getDriveMotorPosition());
    swerveOdometry.addNumber("BL Encoder", () -> m_backLeft.getDriveMotorPosition());

    ShuffleboardLayout virtualGearBox = swerveDriveTab.getLayout("Swerve Speed Controller", BuiltInLayouts.kGrid)
        .withPosition(0, 2)
        .withSize(2, 2);

    Map<String, Object> maxSpeedSliderProperties = new HashMap<>();
    maxSpeedSliderProperties.put("Min", 0);
    maxSpeedSliderProperties.put("Max", MAX_SPEED);

    virtualGearBox.add("Max Speed", 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(maxSpeedSliderProperties)
        .withPosition(0, 0)
        .getEntry()
        .addListener(
            (event) -> setMaxSpeed(event.getEntry().getDouble(MAX_SPEED)),
            EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    Map<String, Object> maxAngularSpeedSliderProperties = new HashMap<>();
    maxAngularSpeedSliderProperties.put("Min", 0);
    maxAngularSpeedSliderProperties.put("Max", MAX_ANGULAR_SPEED);

    virtualGearBox.add("Max Angular Speed", 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(maxAngularSpeedSliderProperties)
        .withPosition(1, 0)
        .getEntry()
        .addListener(
            (event) -> setMaxAngularSpeed(event.getEntry().getDouble(MAX_ANGULAR_SPEED)),
            EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    ShuffleboardLayout swerveDriveTester = swerveDriveTab.getLayout("Drive Tester", BuiltInLayouts.kGrid)
        .withPosition(2, 0)
        .withSize(2, 2);

    swerveDriveTester.add("Left Front", 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withPosition(0, 0)
        .getEntry()
        .addListener(
            (event) -> m_frontLeft.setDriveMotorPower(event.getEntry().getDouble(0)),
            EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    swerveDriveTester.add("Front Right", 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withPosition(1, 0)
        .getEntry()
        .addListener(
            (event) -> m_frontRight.setDriveMotorPower(event.getEntry().getDouble(0)),
            EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    swerveDriveTester.add("Back Left", 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withPosition(0, 1)
        .getEntry()
        .addListener(
            (event) -> m_backLeft.setDriveMotorPower(event.getEntry().getDouble(0)),
            EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    swerveDriveTester.add("Back Right", 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withPosition(1, 1)
        .getEntry()
        .addListener(
            (event) -> m_backRight.setDriveMotorPower(event.getEntry().getDouble(0)),
            EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    ShuffleboardLayout swerveTurnTester = swerveDriveTab.getLayout("Turn Tester", BuiltInLayouts.kGrid)
        .withPosition(4, 0)
        .withSize(2, 2);

    swerveTurnTester.add("Left Front", 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withPosition(0, 0)
        .getEntry()
        .addListener(
            (event) -> m_frontLeft.setTurnMotorPower(event.getEntry().getDouble(0)),
            EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    swerveTurnTester.add("Front Right", 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withPosition(1, 0)
        .getEntry()
        .addListener(
            (event) -> m_frontRight.setTurnMotorPower(event.getEntry().getDouble(0)),
            EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    swerveTurnTester.add("Back Left", 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withPosition(0, 1)
        .getEntry()
        .addListener(
            (event) -> m_backLeft.setTurnMotorPower(event.getEntry().getDouble(0)),
            EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    swerveTurnTester.add("Back Right", 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withPosition(1, 1)
        .getEntry()
        .addListener(
            (event) -> m_backRight.setTurnMotorPower(event.getEntry().getDouble(0)),
            EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    ShuffleboardTab swerveModuleTesting = Shuffleboard.getTab("Swerve Module Testing");

    ShuffleboardLayout frontLeftModuleState = swerveModuleTesting
        .getLayout("Left Front Swerve Module States", BuiltInLayouts.kGrid)
        .withPosition(0, 0)
        .withSize(1, 2);
    frontLeftModuleState.addNumber("LF Desired Rotation", () -> m_frontLeft.getDesiredState().angle.getDegrees());
    frontLeftModuleState.addNumber("LF Desired Speed", () -> m_frontLeft.getDesiredState().speedMetersPerSecond);

    ShuffleboardLayout frontRightModuleState = swerveModuleTesting
        .getLayout("right Front Swerve Module States", BuiltInLayouts.kGrid)
        .withPosition(1, 0)
        .withSize(1, 2);
    frontRightModuleState.addNumber("RF Desired Rotation", () -> m_frontRight.getDesiredState().angle.getDegrees());
    frontRightModuleState.addNumber("RF Desired Speed", () -> m_frontRight.getDesiredState().speedMetersPerSecond);

    ShuffleboardLayout backLeftModuleState = swerveModuleTesting
        .getLayout("Left Back Swerve Module States", BuiltInLayouts.kGrid)
        .withPosition(2, 0)
        .withSize(1, 2);
    backLeftModuleState.addNumber("LB Desired Rotation", () -> m_backLeft.getDesiredState().angle.getDegrees());
    backLeftModuleState.addNumber("LB Desired Speed", () -> m_backLeft.getDesiredState().speedMetersPerSecond);

    ShuffleboardLayout backRightModuleState = swerveModuleTesting
        .getLayout("Right Back Swerve Module States", BuiltInLayouts.kGrid)
        .withPosition(3, 0)
        .withSize(1, 2);
    backRightModuleState.addNumber("RB Desired Rotation", () -> m_backRight.getDesiredState().angle.getDegrees());
    backRightModuleState.addNumber("RB Desired Speed", () -> m_backRight.getDesiredState().speedMetersPerSecond);

    ShuffleboardLayout absoluteEncoderValues = swerveDriveTab.getLayout("Encoders", BuiltInLayouts.kList)
        .withPosition(2, 0)
        .withSize(2, 3);
    absoluteEncoderValues.addNumber("Front Left", () -> m_frontLeft.getAbsolutePosition());
    absoluteEncoderValues.addNumber("Front Right", () -> m_frontRight.getAbsolutePosition());
    absoluteEncoderValues.addNumber("Back Left", () -> m_backLeft.getAbsolutePosition());
    absoluteEncoderValues.addNumber("Back Right", () -> m_backRight.getAbsolutePosition());
  }
}
