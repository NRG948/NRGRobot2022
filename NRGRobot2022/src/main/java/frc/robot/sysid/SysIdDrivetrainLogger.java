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

    public void log(double position, double velocity, double measuredAngle, double angularRate) {
        updateData();
        addData(getTimestamp());
        addData(primaryMotorVoltage);
        addData(secondaryMotorVoltage);
        addData(position);
        addData(position);
        addData(velocity);
        addData(velocity);
        addData(measuredAngle);
        addData(angularRate);
        double motorVoltage = getMotorVoltage();
        primaryMotorVoltage = (isRotating() ?-1:1) * motorVoltage;
        secondaryMotorVoltage = motorVoltage;
    }

    @Override
    public void reset() {
        super.reset();
        primaryMotorVoltage = 0.0;
        secondaryMotorVoltage = 0.0;
    }

}
