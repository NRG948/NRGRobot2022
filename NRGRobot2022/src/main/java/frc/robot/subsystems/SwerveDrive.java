// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;

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
  double zeroFrontLeft = -2.197;
  double zeroFrontRight = 129.990;
  double zeroBackLeft = 88.066;
  double zeroBackRight = -70.048;




  private class Module {

    private static final double kWheelRadius = 0.047625;
    private static final int kEncoderResolution = 2048;
    private static final double kDrivePulsesPerMeter = kEncoderResolution / (2 * kWheelRadius * Math.PI); // pulses per
                                                                                                          // meter

    private static final double kModuleMaxAngularVelocity = SwerveDrive.kMaxAngularSpeed;
    private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

    private final TalonFX m_driveMotor;
    private final TalonFX m_turningMotor;

    // private final Encoder m_driveEncoder;
    private final CANCoder m_turningEncoder;

    /* TODO: Tune PID for drive and turning PID controllers */
    // Gains are for example purposes only - must be determined for your own robot!
    private final PIDController m_drivePIDController = new PIDController(1, 0, 0);

    // Gains are for example purposes only - must be determined for your own robot!
    private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
        1,
        0,
        0,
        new TrapezoidProfile.Constraints(
            kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

    // Gains are for example purposes only - must be determined for your own robot!
    private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
    private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

    /**
     * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
     * and turning encoder.
     *
     * @param driveMotorChannel    CAN ID of the drive motor.
     * @param turningMotorChannel  CAN ID of the turning motor.
     * @param turningEncodeChannel
     */
    public Module(
        int driveMotorChannel,
        int turningMotorChannel,
        int turningEncodeChannel

    ) {
      m_driveMotor = new TalonFX(driveMotorChannel);
      m_turningMotor = new TalonFX(turningMotorChannel);

      m_turningEncoder = new CANCoder(turningEncodeChannel);

      m_turningEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

      // Limit the PID Controller's input range between -pi and pi and set the input
      // to be continuous.
      m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
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
      return m_driveMotor.getSelectedSensorVelocity() / kDrivePulsesPerMeter;

    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {

      // Optimize the reference state to avoid spinning further than 90 degrees
      Rotation2d currentAngle = Rotation2d.fromDegrees(m_turningEncoder.getAbsolutePosition());
      SwerveModuleState state = SwerveModuleState.optimize(desiredState, currentAngle);

      // Calculate the drive output from the drive PID controller.
      final double driveOutput = m_drivePIDController.calculate(getDriveMotorVelocity(), state.speedMetersPerSecond);

      final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

      // Calculate the turning motor output from the turning PID controller.
      final double turnOutput = m_turningPIDController.calculate(currentAngle.getRadians(), state.angle.getRadians());

      final double turnFeedforward = m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

      final double batteryVolatage = RobotController.getBatteryVoltage();

      m_driveMotor.set(ControlMode.PercentOutput, (driveOutput + driveFeedforward) / batteryVolatage);
      m_turningMotor.set(ControlMode.PercentOutput, (turnOutput + turnFeedforward) / batteryVolatage);
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

  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

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

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
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
    var swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
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

  /** Returns the current orientation of the robot as a Rotation2d object */
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(m_ahrs.getAngle());
  }

  /** Returns the current pose of the robot as a Pose2d object */
  public Pose2d getPose2d() {
    return m_odometry.getPoseMeters();
  }

  public void initShuffleboardTab() {
    ShuffleboardTab swerveDriveTab = Shuffleboard.getTab("Swerve Drive");

    ShuffleboardLayout swerveKinematics = swerveDriveTab.getLayout("Odemetry", BuiltInLayouts.kGrid)
      .withPosition(0, 0)
      .withSize(2, 2);

      swerveKinematics.addNumber("Gyro", () -> getRotation2d().getDegrees())
        .withPosition(0, 0)
        .withSize(2, 1);
      swerveKinematics.addString("Position", () -> { var pose = getPose2d(); return "X: " + pose.getX() + " Y: " + pose.getY(); })
        .withPosition(0, 1)
        .withSize(2, 1);

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

    ShuffleboardLayout absoluteEncoderValues = swerveDriveTab.getLayout("Encoders", BuiltInLayouts.kList)
        .withPosition(6, 0)
        .withSize(2, 3);
    absoluteEncoderValues.addNumber("Front Left", () -> m_frontLeft.getAbsolutePosition());
    absoluteEncoderValues.addNumber("Front Right", () -> m_frontRight.getAbsolutePosition());
    absoluteEncoderValues.addNumber("Back Left", () -> m_backLeft.getAbsolutePosition());
    absoluteEncoderValues.addNumber("Back Right", () -> m_backRight.getAbsolutePosition());
  }
}
