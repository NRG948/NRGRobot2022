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
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.preferences.RobotPreferencesLayout;
import frc.robot.preferences.RobotPreferencesValue;
import frc.robot.preferences.RobotPreferences.DoubleValue;

@RobotPreferencesLayout(groupName = "SwerveDrive", column = 0, row = 0, width = 2, height = 4)
public class SwerveDrive extends SubsystemBase {

  /*
   * 
   * Forward motion of drive wheel is the directions your fingers curl
   * when your thumb points in the direction of the bolt.
   * 
   */

  /* Swerve Module helper class */
  @RobotPreferencesLayout(groupName = "SwerveModule", column = 2, row = 0, width = 2, height = 4)
  public static class Module {
    @RobotPreferencesValue
    public static DoubleValue driveP = new DoubleValue("SwerveModule", "driveP", 1.0);
    @RobotPreferencesValue
    public static DoubleValue driveFeedForwardS = new DoubleValue("SwerveModule", "driveFeedForwardS", 1.0);
    @RobotPreferencesValue
    public static DoubleValue driveFeedForwardV = new DoubleValue("SwerveModule", "driveFeedForwardV", 3.0);
    @RobotPreferencesValue
    public static DoubleValue turnP = new DoubleValue("SwerveModule", "turnP", 7.0);
    @RobotPreferencesValue
    public static DoubleValue turnFeedForwardS = new DoubleValue("SwerveModule", "turnFeedForwardS", 1.0);
    @RobotPreferencesValue
    public static DoubleValue turnFeedForwardV = new DoubleValue("SwerveModule", "turnFeedForwardV", 0.5);

    private static final double WHEEL_RADIUS = 0.047625; // Meters
    private static final int ENCODER_RESOLUTION = 2048; // Steps per Rev
    private static final double DRIVE_GEAR_RATIO = 8.14; // Gear ratio
    private static final double DRIVE_PULSES_PER_METER = (ENCODER_RESOLUTION * DRIVE_GEAR_RATIO)
        / (2 * WHEEL_RADIUS * Math.PI); // pulses per meter

    private static final double MODULE_MAX_ANGULAR_VELOCITY = SwerveDrive.MAX_ANGULAR_SPEED;
    private static final double MODULE_MAX_ANGULAR_ACCELERATION = 2 * Math.PI; // radians per second squared

    private final TalonFX driveMotor;
    private final TalonFX turningMotor;

    // private final Encoder m_driveEncoder;
    private final CANCoder turningEncoder;

    /* TODO: Tune PID for drive and turning PID controllers */
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
    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(driveFeedForwardS.getValue(), driveFeedForwardV.getValue());
    private final SimpleMotorFeedforward turnFeedforward = new SimpleMotorFeedforward(turnFeedForwardS.getValue(), turnFeedForwardV.getValue());

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
    public Module(int driveMotorChannel, int turningMotorChannel, int turningEncodeChannel, String moduleName) {
      driveMotor = new TalonFX(driveMotorChannel);
      turningMotor = new TalonFX(turningMotorChannel);

      turningEncoder = new CANCoder(turningEncodeChannel);

      this.moduleName = moduleName;

      turningEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
      // m_turningEncoder.config
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

    /** Returns the current wheel angle in degrees. The range of values is [-180..180]. */
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

      layout.add("Drive Motor", 0)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .getEntry()
          .addListener(
              (event) -> this.setDriveMotorPower(event.getEntry().getDouble(0)),
              EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

      layout.add("Turn Motor", 0)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .getEntry()
          .addListener(
              (event) -> this.setTurnMotorPower(event.getEntry().getDouble(0)),
              EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

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

  public static final double MAX_SPEED = 3.0; // 3 meters per second
  public static final double MAX_ANGULAR_SPEED = Math.PI; // 1/2 rotation per second
  public static final double MAX_ACCELERATION = 2.0; // TODO: find Max acceleration in meters per second squared

  @RobotPreferencesValue
  public static final DoubleValue turnP = new DoubleValue("SwerveDrive", "turnP", 1.0);
  @RobotPreferencesValue
  public static final DoubleValue turnI = new DoubleValue("SwerveDrive", "turnI", 0);
  @RobotPreferencesValue
  public static final DoubleValue turnD = new DoubleValue("SwerveDrive", "turnD", 0);

  public static double currentMaxSpeed = MAX_SPEED;
  public static double currentMaxAngularSpeed = MAX_ANGULAR_SPEED;

  public static boolean turnToAngle = false;
  public static Rotation2d targetAngle = Rotation2d.fromDegrees(0);

  // X and Y swaped
  private final Translation2d m_frontLeftLocation = new Translation2d(0.3302, 0.2413);
  private final Translation2d m_frontRightLocation = new Translation2d(0.3302, -0.2413);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.3302, 0.2413);
  private final Translation2d m_backRightLocation = new Translation2d(-0.3302, -0.2413);

  private final Module m_frontLeft = new Module(1, 2, 9, "Front Left");
  private final Module m_frontRight = new Module(3, 4, 10, "Front Right");
  private final Module m_backLeft = new Module(7, 8, 12, "Back Left");
  private final Module m_backRight = new Module(5, 6, 11, "Back Right");

  private final AHRS m_ahrs = new AHRS(SerialPort.Port.kMXP);

  public final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, getRotation2d());

  
  public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
      SwerveDrive.MAX_SPEED, SwerveDrive.MAX_ACCELERATION);
  private final ProfiledPIDController thetaController = new ProfiledPIDController(
    turnP.getValue(), turnI.getValue(), turnD.getValue(), kThetaControllerConstraints);


  public SwerveDrive() {
    m_ahrs.reset();
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(Math.PI/36); // 5 degree tolerance
  }

  public void reset() {
    m_ahrs.reset();
    m_frontLeft.reset();
    m_frontRight.reset();
    m_backLeft.reset();
    m_backRight.reset();
    m_odometry.resetPosition(new Pose2d(), getRotation2d());
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
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean squareInputs) {
    if(squareInputs){
      xSpeed *= Math.abs(xSpeed);
      ySpeed *= Math.abs(ySpeed);
      rot *= Math.abs(rot);
    }

    if(turnToAngle){
      if(thetaController.atGoal()){
        disableTurnToAngle();
      }
      else{
        rot = calculateRotSpeed();
      }
    }

    xSpeed = MathUtil.applyDeadband(xSpeed, 0.02) * currentMaxSpeed;
    ySpeed = MathUtil.applyDeadband(ySpeed, 0.02) * currentMaxSpeed;
    rot = MathUtil.applyDeadband(rot, 0.02) * currentMaxAngularSpeed;

    var swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    setModuleStates(swerveModuleStates);
  }

  public double calculateRotSpeed(){

    return thetaController.calculate(getRotation2d().getRadians(), targetAngle.getRadians());
  }

  public void enableTurnToAngle(double angle){
    turnToAngle = true;
    targetAngle = Rotation2d.fromDegrees(angle);
  }
  public void disableTurnToAngle(){
    turnToAngle = false;
  }

  /** Sets the maximum drive speed. This value is clamped to the range [0..MAX_SPEED]. */
  public void setMaxSpeed(double speed) {
    currentMaxSpeed = MathUtil.clamp(speed, 0, MAX_SPEED);
  }

  /** Sets the maximum angular rotation speed. This value is clamped to the range [0..MAX_ANGULAR_SPEED]. */
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

  /** Sets the desired state of a swerve module. */
  @Deprecated(forRemoval = true)
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

  /** Sets the desired module states. */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, MAX_SPEED);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_backLeft.setDesiredState(desiredStates[2]);
    m_backRight.setDesiredState(desiredStates[3]);
  }

  // Stops all Swerve Drive Motors
  public void stopMotors() {
    m_frontLeft.stopMotors();
    m_frontRight.stopMotors();
    m_backLeft.stopMotors();
    m_backRight.stopMotors();
    turnToAngle = false;

  }

  /** Adds and initializes a Shufflboard tab for this subsystem. */
  public void initShuffleboardTab() {
    ShuffleboardTab swerveDriveTab = Shuffleboard.getTab("Swerve Drive");

    ShuffleboardLayout swerveOdometry = swerveDriveTab.getLayout("Odometry", BuiltInLayouts.kGrid)
        .withPosition(0, 0)
        .withSize(2, 3);

    swerveOdometry.addNumber("Gyro", () -> getRotation2d().getDegrees());
    swerveOdometry.addNumber("X", () -> getPose2d().getX());
    swerveOdometry.addNumber("Y", () -> getPose2d().getY());
    swerveOdometry.addNumber("FR Encoder", () -> m_frontRight.getWheelDistance());
    swerveOdometry.addNumber("FL Encoder", () -> m_frontLeft.getWheelDistance());
    swerveOdometry.addNumber("BR Encoder", () -> m_backRight.getWheelDistance());
    swerveOdometry.addNumber("BL Encoder", () -> m_backLeft.getWheelDistance());

    m_frontLeft.addShuffleBoardLayout(swerveDriveTab)
        .withPosition(2, 0)
        .withSize(2, 3);

    m_frontRight.addShuffleBoardLayout(swerveDriveTab)
        .withPosition(4, 0)
        .withSize(2, 3);

    m_backLeft.addShuffleBoardLayout(swerveDriveTab)
        .withPosition(2, 3)
        .withSize(2, 3);

    m_backRight.addShuffleBoardLayout(swerveDriveTab)
        .withPosition(4, 3)
        .withSize(2, 3);

    ShuffleboardLayout virtualGearBox = swerveDriveTab.getLayout("Swerve Speed Controller", BuiltInLayouts.kGrid)
        .withPosition(0, 3)
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
  }
}

