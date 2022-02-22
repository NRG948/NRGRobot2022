// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.List;
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
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.preferences.RobotPreferences.BooleanValue;
import frc.robot.preferences.RobotPreferences.DoubleValue;
import frc.robot.utilities.ShuffleboardUtils;
import frc.robot.commands.CharacterizeSwerveDrive;
import frc.robot.preferences.RobotPreferencesLayout;
import frc.robot.preferences.RobotPreferencesValue;

@RobotPreferencesLayout(groupName = "SwerveDrive", column = 0, row = 0, width = 1, height = 3)
public class SwerveDrive extends SubsystemBase {

  /*
   * 
   * Forward motion of drive wheel is the directions your fingers curl
   * when your thumb points in the direction of the bolt.
   * 
   */

  /* Swerve Module helper class */
  @RobotPreferencesLayout(groupName = "SwerveModule", column = 1, row = 0, width = 2, height = 4)
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
    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(driveFeedForwardS.getValue(),
        driveFeedForwardV.getValue());
    private final SimpleMotorFeedforward turnFeedforward = new SimpleMotorFeedforward(turnFeedForwardS.getValue(),
        turnFeedForwardV.getValue());

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

  public static final double MAX_SPEED = 3.0; // 3 meters per second
  public static final double MAX_ANGULAR_SPEED = Math.PI; // 1/2 rotation per second
  public static final double MAX_ACCELERATION = 2.0; // TODO: find Max acceleration in meters per second squared
  public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(
      SwerveDrive.MAX_SPEED, SwerveDrive.MAX_ACCELERATION);

  @RobotPreferencesValue
  public static final DoubleValue turnP = new DoubleValue("SwerveDrive", "turnP", 1.0);
  @RobotPreferencesValue
  public static final DoubleValue turnI = new DoubleValue("SwerveDrive", "turnI", 0);
  @RobotPreferencesValue
  public static final DoubleValue turnD = new DoubleValue("SwerveDrive", "turnD", 0);
  @RobotPreferencesValue
  public static final BooleanValue enableTab = new BooleanValue("SwerveDrive", "enableTab", false);

  public double currentMaxSpeed = MAX_SPEED;
  public double currentMaxAngularSpeed = MAX_ANGULAR_SPEED;

  public boolean turnToAngle = false;
  public Rotation2d targetAngle = Rotation2d.fromDegrees(0);

  // X and Y swaped
  private final Translation2d frontLeftLocation = new Translation2d(0.3302, 0.2413);
  private final Translation2d frontRightLocation = new Translation2d(0.3302, -0.2413);
  private final Translation2d backLeftLocation = new Translation2d(-0.3302, 0.2413);
  private final Translation2d backRightLocation = new Translation2d(-0.3302, -0.2413);

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);
  private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, getRotation2d());

  private final Module frontLeft = new Module(1, 2, 9, "Front Left");
  private final Module frontRight = new Module(3, 4, 10, "Front Right");
  private final Module backLeft = new Module(7, 8, 12, "Back Left");
  private final Module backRight = new Module(5, 6, 11, "Back Right");

  private final AHRS ahrs = new AHRS(SerialPort.Port.kMXP);

  private final ProfiledPIDController thetaController = new ProfiledPIDController(
      turnP.getValue(), turnI.getValue(), turnD.getValue(), THETA_CONTROLLER_CONSTRAINTS);

  public SwerveDrive() {
    ahrs.reset();
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(Math.PI / 36); // 5 degree tolerance
  }

  public void reset() {
    ahrs.reset();
    frontLeft.reset();
    frontRight.reset();
    backLeft.reset();
    backRight.reset();
    odometry.resetPosition(new Pose2d(), getRotation2d());
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
    if (squareInputs) {
      xSpeed *= Math.abs(xSpeed);
      ySpeed *= Math.abs(ySpeed);
      rot *= Math.abs(rot);
    }

    if (turnToAngle) {

      if (thetaController.atGoal()) {
        disableTurnToAngle();
      } else {
        rot = calculateRotSpeed();
      }
    }

    xSpeed = MathUtil.applyDeadband(xSpeed, 0.02) * currentMaxSpeed;
    ySpeed = MathUtil.applyDeadband(ySpeed, 0.02) * currentMaxSpeed;
    rot = MathUtil.applyDeadband(rot, 0.02) * currentMaxAngularSpeed;

    var swerveModuleStates = kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    setModuleStates(swerveModuleStates);
  }

  public double calculateRotSpeed() {

    return thetaController.calculate(getRotation2d().getRadians(), targetAngle.getRadians());
  }

  public void enableTurnToAngle(double angle) {
    turnToAngle = true;
    targetAngle = Rotation2d.fromDegrees(angle);
    thetaController.setGoal(targetAngle.getRadians());
  }

  public void disableTurnToAngle() {
    turnToAngle = false;
  }

  /**
   * Sets the maximum drive speed. This value is clamped to the range
   * [0..MAX_SPEED].
   */
  public void setMaxSpeed(double speed) {
    currentMaxSpeed = MathUtil.clamp(speed, 0, MAX_SPEED);
  }

  /**
   * Sets the maximum angular rotation speed. This value is clamped to the range
   * [0..MAX_ANGULAR_SPEED].
   */
  public void setMaxAngularSpeed(double angularSpeed) {
    currentMaxAngularSpeed = MathUtil.clamp(angularSpeed, 0, MAX_ANGULAR_SPEED);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    odometry.update(
        getRotation2d(),
        frontLeft.getState(),
        frontRight.getState(),
        backLeft.getState(),
        backRight.getState());
  }

  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(frontLeft.getState(), frontRight.getState(), backLeft.getState(),
        backRight.getState());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(pose, getRotation2d());
  }

  @Override
  public void periodic() {
    updateOdometry();
  }

  /** Returns the current orientation of the robot as a Rotation2d object */
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(-ahrs.getAngle());
  }

  /** Returns the current pose of the robot as a Pose2d object */
  public Pose2d getPose2d() {
    return odometry.getPoseMeters();
  }

  /** Sets the desired state of a swerve module. */
  @Deprecated(forRemoval = true)
  public void setModuleState(int index, double speed, double angle) {
    Rotation2d rotation = Rotation2d.fromDegrees(angle);
    SwerveModuleState state = new SwerveModuleState(speed, rotation);
    switch (index) {
      case 0:
        frontLeft.setDesiredState(state);
        break;
      case 1:
        frontRight.setDesiredState(state);
        break;
      case 2:
        backLeft.setDesiredState(state);
        break;
      case 3:
        backRight.setDesiredState(state);
        break;
    }
  }

  /** Sets the desired module states. */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, MAX_SPEED);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  // Stops all Swerve Drive Motors
  public void stopMotors() {
    frontLeft.stopMotors();
    frontRight.stopMotors();
    backLeft.stopMotors();
    backRight.stopMotors();
    turnToAngle = false;

  }

  /**
   * Generates a trajectory to be followed using
   * {@link edu.wpi.first.wpilibj2.command.SwerveControllerCommand}.
   * 
   * @param initialPose2d The initial robot pose.
   * @param waypoints     A list of waypoints through which the robot should
   *                      traverse.
   * @param finalPose2d   The final robot pose.
   * @param reversed      Set to true when the robot must follow the path
   *                      backwards.
   * 
   * @return The trajectory to be followed using
   *         {@link edu.wpi.first.wpilibj2.command.SwerveControllerCommand}.
   */
  public Trajectory generateTrajectory(
      Pose2d initialPose2d,
      List<Translation2d> waypoints,
      Pose2d finalPose2d,
      boolean reversed) {
    TrajectoryConfig config = new TrajectoryConfig(MAX_SPEED, MAX_ACCELERATION)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(kinematics)
        .setReversed(reversed);

    // An example trajectory to follow. All units in meters.
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        initialPose2d,
        waypoints,
        finalPose2d,
        config);

    return trajectory;
  }

  /** Adds and initializes a Shufflboard tab for this subsystem. */
  public void initShuffleboardTab() {
    if (!enableTab.getValue()) {
      return;
    }

    ShuffleboardTab swerveDriveTab = Shuffleboard.getTab("Swerve Drive");

    ShuffleboardLayout swerveOdometry = swerveDriveTab.getLayout("Odometry", BuiltInLayouts.kGrid)
        .withPosition(0, 0)
        .withSize(2, 3);

    swerveOdometry.addNumber("Gyro", () -> getRotation2d().getDegrees());
    swerveOdometry.addNumber("X", () -> getPose2d().getX());
    swerveOdometry.addNumber("Y", () -> getPose2d().getY());
    swerveOdometry.addNumber("FR Encoder", () -> frontRight.getWheelDistance());
    swerveOdometry.addNumber("FL Encoder", () -> frontLeft.getWheelDistance());
    swerveOdometry.addNumber("BR Encoder", () -> backRight.getWheelDistance());
    swerveOdometry.addNumber("BL Encoder", () -> backLeft.getWheelDistance());

    frontLeft.addShuffleBoardLayout(swerveDriveTab)
        .withPosition(2, 0)
        .withSize(2, 3);

    frontRight.addShuffleBoardLayout(swerveDriveTab)
        .withPosition(4, 0)
        .withSize(2, 3);

    backLeft.addShuffleBoardLayout(swerveDriveTab)
        .withPosition(2, 3)
        .withSize(2, 3);

    backRight.addShuffleBoardLayout(swerveDriveTab)
        .withPosition(4, 3)
        .withSize(2, 3);

    ShuffleboardLayout virtualGearBox = swerveDriveTab.getLayout("Swerve Speed Controller", BuiltInLayouts.kGrid)
        .withPosition(0, 3)
        .withSize(2, 2);

    Map<String, Object> maxSpeedSliderProperties = new HashMap<>();
    maxSpeedSliderProperties.put("Min", 0);
    maxSpeedSliderProperties.put("Max", MAX_SPEED);

    ShuffleboardUtils.addNumberSlider(virtualGearBox, "Max Speed", currentMaxSpeed, (max) -> setMaxSpeed(max))
        .withProperties(maxSpeedSliderProperties)
        .withPosition(0, 0);

    Map<String, Object> maxAngularSpeedSliderProperties = new HashMap<>();
    maxAngularSpeedSliderProperties.put("Min", 0);
    maxAngularSpeedSliderProperties.put("Max", MAX_ANGULAR_SPEED);

    ShuffleboardUtils
        .addNumberSlider(virtualGearBox, "Max Angular Speed", currentMaxAngularSpeed, (max) -> setMaxAngularSpeed(max))
        .withProperties(maxAngularSpeedSliderProperties)
        .withPosition(1, 0);

    ShuffleboardLayout commandLayout = swerveDriveTab.getLayout("Commands", BuiltInLayouts.kList)
        .withPosition(6, 0)
        .withSize(2, 2);
    commandLayout.add(new CharacterizeSwerveDrive(this));
  }
}
