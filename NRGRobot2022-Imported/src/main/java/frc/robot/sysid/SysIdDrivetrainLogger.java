// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sysid;

/** Add your docs here. */
public class SysIdDrivetrainLogger extends SysIdLogger {
    private final double wheelModuleRadius;

    private double primaryMotorVoltage;
    private double secondaryMotorVoltage;

    /**
     * Constructs an instance of a logger to gather data on a drivetrain.
     * 
     * @param wheelModuleRadius The distance in meters from the center of the robot
     *                          frame to the wheel module. This is used to calculate
     *                          the distance driven by each side of the robot to
     *                          simulate expected data for a differential drive.
     */
    public SysIdDrivetrainLogger(double wheelModuleRadius) {
        this.wheelModuleRadius = wheelModuleRadius;
    }

    @Override
    protected boolean isWrongMechanism() {
        String mechanism = getMechanism();

        return !mechanism.equals("Drivetrain") && !mechanism.equals("Drivetrain (Angular)");
    }

    /**
     * Logs data for the SysID tool for a Swerve drivetrain.
     * 
     * @param measuredPosition    The measured position of the robot in the
     *                            direction of movement.
     * @param measuredVelocity    The measured velocity of the robot, in meters per
     *                            second.
     * @param measuredAngle       The measured orientation of the robot, in radians.
     * @param measuredAngularRate The measured angular rate of the change of the
     *                            robot, in radians per second.
     */
    public void log(
            double measuredPosition,
            double measuredVelocity,
            double measuredAngle,
            double measuredAngularRate) {
        updateData();

        boolean rotating = isRotating();
        double leftPosition;
        double rightPosition;

        if (!rotating) {
            // When running the linear translation tests, report the same position for both
            // sides to simulate the data for a differntial drivetrain.
            leftPosition = measuredPosition;
            rightPosition = measuredPosition;
        } else {
            // When running the rotation tests, calculate the distance travelled by the left
            // and right sides of the robot using the wheel module distance.
            rightPosition = wheelModuleRadius * measuredAngle;
            leftPosition = -rightPosition;
        }

        // The data format for a general drivetrain is described here in
        // https://github.com/wpilibsuite/sysid/blob/main/docs/data-collection.md#drivetrain
        //
        // SysID assumes a differential drivetrain, so left & right side data from the
        // Swerve drive is duplicated to match the expected data format.
        addData(getTimestamp(),
                primaryMotorVoltage,
                secondaryMotorVoltage,
                leftPosition,
                rightPosition,
                (rotating ? -1 : 1) * measuredVelocity,
                measuredVelocity,
                measuredAngle,
                measuredAngularRate);

        double motorVoltage = getMotorVoltage();

        primaryMotorVoltage = (rotating ? -1 : 1) * motorVoltage;
        secondaryMotorVoltage = motorVoltage;
    }

    @Override
    public void reset() {
        super.reset();
        primaryMotorVoltage = 0.0;
        secondaryMotorVoltage = 0.0;
    }

}
