// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.SwerveDrive;

public final class CommandUtils {
        public static final ExecutorService executor = Executors.newSingleThreadExecutor();
        // public static Field2d field;
        public static CommandBase newFollowWaypointsCommand(
                        SwerveDrive swerve,
                        Pose2d initialPose2d,
                        List<Translation2d> waypoints,
                        Pose2d finalPose2d,
                        boolean reversed) {
                // Create config for trajectory
                Trajectory trajectory = swerve.generateTrajectory(initialPose2d, waypoints, finalPose2d, reversed);
                // field = new Field2d();
                // SmartDashboard.putData(field);
                // field.getObject("traj").setTrajectory(trajectory);
                var thetaController = new ProfiledPIDController(
                                SwerveDrive.turnP.getValue(),
                                SwerveDrive.turnI.getValue(),
                                SwerveDrive.turnD.getValue(),
                                SwerveDrive.THETA_CONTROLLER_CONSTRAINTS);
                thetaController.enableContinuousInput(-Math.PI, Math.PI);

                SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                                trajectory,
                                swerve::getPose2d, // Functional interface to feed supplier
                                swerve.getKinematics(),

                                // Position controllers
                                new PIDController(SwerveDrive.driveP.getValue(), SwerveDrive.driveI.getValue(),
                                                SwerveDrive.driveD.getValue()),
                                new PIDController(SwerveDrive.driveP.getValue(), SwerveDrive.driveI.getValue(),
                                                SwerveDrive.driveD.getValue()),
                                thetaController,
                                swerve::setModuleStates,
                                swerve);

                // Run path following command, then stop at the end.
                return swerveControllerCommand.andThen(() -> swerve.stopMotors())
                                .andThen(() -> {
                                        Pose2d currPose = swerve.getPose2d();
                                        Translation2d currLocation = currPose.getTranslation();
                                        System.out.println(
                                                        String.format("END SwerveControllerCommand x: %f y: %f h: %f",
                                                                        currLocation.getX(), currLocation.getY(),
                                                                        currPose.getRotation().getDegrees()));
                                });
        }

        public static FutureScheduledCommand newFutureFollowWaypointsCommand(
                        SwerveDrive swerve,
                        List<Translation2d> waypoints,
                        Pose2d finalPose2d,
                        boolean reversed) {
                return new FutureScheduledCommand(() -> executor.submit(() -> newFollowWaypointsCommand(
                                swerve, swerve.getPose2d(), waypoints, finalPose2d, reversed)));
        }
}
