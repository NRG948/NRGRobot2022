// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.CharacterizeSwerveDrive;
import frc.robot.preferences.RobotPreferences.BooleanValue;
import frc.robot.preferences.RobotPreferences.DoubleValue;
import frc.robot.preferences.RobotPreferencesLayout;
import frc.robot.preferences.RobotPreferencesValue;
import frc.robot.utilities.ShuffleboardUtils;

@RobotPreferencesLayout(groupName = "SwerveDrive", column = 0, row = 0, width = 2, height = 3, type = "Grid Layout")
public class SwerveDrive extends SubsystemBase {

  /*
   * 
   * Forward motion of drive wheel is the directions your fingers curl
   * when your thumb points in the direction of the bolt.
   * 
   */

  /* Swerve Module helper class */
  

  public static final double MAX_SPEED = 3.0; // 3 meters per second
  public static final double MAX_ANGULAR_SPEED = (3 * Math.PI) / 2; // 1/2 rotation per second
  public static final double MAX_ACCELERATION = 2.0; // TODO: find Max acceleration in meters per second squared
  public static final double MAX_AUTO_ANGULAR_SPEED = Math.PI / 2;
  public static final double MAX_AUTO_ANGULAR_ACCELERATION = (3 * Math.PI) / 2;
  public static final double MAX_AUTO_SPEED = 1.5;
  public static final double MAX_AUTO_ACCELERATION = 0.5;

  public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(
      SwerveDrive.MAX_AUTO_ANGULAR_SPEED, SwerveDrive.MAX_AUTO_ANGULAR_ACCELERATION);

  @RobotPreferencesValue
  public static final DoubleValue turnP = new DoubleValue("SwerveDrive", "turnP", 1.0);
  @RobotPreferencesValue
  public static final DoubleValue turnI = new DoubleValue("SwerveDrive", "turnI", 0);
  @RobotPreferencesValue
  public static final DoubleValue turnD = new DoubleValue("SwerveDrive", "turnD", 0);
  @RobotPreferencesValue
  public static final DoubleValue driveP = new DoubleValue("SwerveDrive", "driveP", 1.0);
  @RobotPreferencesValue
  public static final DoubleValue driveI = new DoubleValue("SwerveDrive", "driveI", 0);
  @RobotPreferencesValue
  public static final DoubleValue driveD = new DoubleValue("SwerveDrive", "driveD", 0);
  @RobotPreferencesValue
  public static final BooleanValue enableTab = new BooleanValue("SwerveDrive", "enableTab", false);

  public double currentMaxSpeed = MAX_SPEED;
  public double currentMaxAngularSpeed = MAX_ANGULAR_SPEED;

  public boolean turnToAngle = false;
  public Rotation2d targetAngle = Rotation2d.fromDegrees(0);

  // X and Y swaped
  public static final Translation2d FRONT_LEFT_LOCATION = new Translation2d(0.3302, 0.2413);
  public static final Translation2d FRONT_RIGHT_LOCATION = new Translation2d(0.3302, -0.2413);
  public static final Translation2d BACK_LEFT_LOCATION = new Translation2d(-0.3302, 0.2413);
  public static final Translation2d BACK_RIGHT_LOCATION = new Translation2d(-0.3302, -0.2413);

  private final AHRS ahrs = new AHRS(SerialPort.Port.kMXP);

  private final SwerveModule frontLeft = new SwerveModule(1, 2, 9, "Front Left");
  private final SwerveModule frontRight = new SwerveModule(3, 4, 10, "Front Right");
  private final SwerveModule backLeft = new SwerveModule(7, 8, 12, "Back Left");
  private final SwerveModule backRight = new SwerveModule(5, 6, 11, "Back Right");

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      FRONT_LEFT_LOCATION, FRONT_RIGHT_LOCATION, BACK_LEFT_LOCATION, BACK_RIGHT_LOCATION);
  private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(
    kinematics, getRotation2d(), getModulePositions());

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
    odometry.resetPosition(getRotation2d(), getModulePositions(), new Pose2d());
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition()
    };
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
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getPose2d().getRotation())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    setModuleStates(swerveModuleStates);
  }

  public double calculateRotSpeed() {

    return thetaController.calculate(getRotation2d().getRadians(), targetAngle.getRadians());
  }

  /**
   * Enable turning the robot to a specific angle
   * @param angle Desired angle in degrees
   */
  public void enableTurnToAngle(double angle) {
    turnToAngle = true;
    targetAngle = Rotation2d.fromDegrees(angle);
    thetaController.setGoal(targetAngle.getRadians());
  }

  public void disableTurnToAngle() {
    turnToAngle = false;
  }

  /** Returns the drive train kinematics. */
  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  /**
   * Returns the distance, in meters, from the center of the robot frame to the
   * wheels.
   */
  public double getModuleRadius() {
    return FRONT_LEFT_LOCATION.getNorm();
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
    odometry.update(getRotation2d(), getModulePositions());
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
    odometry.resetPosition(getRotation2d(), getModulePositions(), pose);
  }

  public void resetHeading() {
    Pose2d currentPos = getPose2d();
    Pose2d newPos2d = new Pose2d(currentPos.getTranslation(), new Rotation2d());
    this.resetOdometry(newPos2d);
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

  public double getHeadingDegrees() {
    return  getPose2d().getRotation().getDegrees();
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
    TrajectoryConfig config = new TrajectoryConfig(MAX_AUTO_SPEED, MAX_AUTO_ACCELERATION)
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

    swerveOdometry.addNumber("Heading", () -> getPose2d().getRotation().getDegrees());
    swerveOdometry.addNumber("X", () -> getPose2d().getX());
    swerveOdometry.addNumber("Y", () -> getPose2d().getY());
    swerveOdometry.addNumber("Gyro", () -> getRotation2d().getDegrees());
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
