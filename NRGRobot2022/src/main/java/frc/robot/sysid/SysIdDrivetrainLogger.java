// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sysid;

/** Add your docs here. */
public class SysIdDrivetrainLogger extends SysIdLogger {

    double primaryMotorVoltage;
    double secondaryMotorVoltage;

    @Override
    protected boolean isWrongMechanism() {
        String mechanism = getMechanism();

        return !mechanism.equals("Drivetrain") && !mechanism.equals("Drivetrain (Angular)");
    }

    /**
     * Logs data for the SysID tool for a Swerve drivetrain.
     * 
     * @param measurePosition     The measured position of the robot in the
     *                            direction of movement.
     * @param measuredVelocity    The measured velocity of the robot.
     * @param measuredAngle       The measured orientation of the robot.
     * @param measuredAngularRate The measured angular rate of the change of the
     *                            robot.
     */
    public void log(double measurePosition, double measuredVelocity, double measuredAngle, double measuredAngularRate) {
        updateData();

        // The data format for a general drivetrain is described here in
        // https://github.com/wpilibsuite/sysid/blob/main/docs/data-collection.md#drivetrain
        //
        // SysID assumes a differential drivetrain, so left & right side data from the
        // Swerve drive is duplicated to match the expected data format.
        addData(getTimestamp(),
                primaryMotorVoltage,
                secondaryMotorVoltage,
                measurePosition,
                measurePosition,
                measuredVelocity,
                measuredVelocity,
                measuredAngle,
                measuredAngularRate);

        double motorVoltage = getMotorVoltage();

        primaryMotorVoltage = (isRotating() ? -1 : 1) * motorVoltage;
        secondaryMotorVoltage = motorVoltage;
    }

    @Override
    public void reset() {
        super.reset();
        primaryMotorVoltage = 0.0;
        secondaryMotorVoltage = 0.0;
    }

}
