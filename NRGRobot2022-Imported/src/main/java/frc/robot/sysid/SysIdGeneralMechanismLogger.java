// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sysid;

/** Add your docs here. */
public class SysIdGeneralMechanismLogger extends SysIdLogger {

    private double motorVoltage = 0;

    @Override
    protected boolean isWrongMechanism() {
        String mechanism = getMechanism();

        return !mechanism.equals("Arm") && !mechanism.equals("Elevator") && !mechanism.equals("Simple");
    }

    /**
     * Logs data for the SysID tool for a simple mechanism with a motor and one
     * degree of freedom (e.g. Arm, Elevator, etc.)
     * 
     * @param measuredPosition The measured position of the mechanism.
     * @param measuredVelocity The measured velocity of the mechanism.
     */
    public void log(double measuredPosition, double measuredVelocity) {
        updateData();

        // The data format for a general mechanism is described here in
        // https://github.com/wpilibsuite/sysid/blob/main/docs/data-collection.md#non-drivetrain-mechanisms
        //
        // SysID assumes a differential drivetrain, so left & right side data from the
        // Swerve drive is duplicated to match the expected data format.
        addData(getTimestamp(), motorVoltage, measuredPosition, measuredVelocity);

        motorVoltage = getMotorVoltage();
    }

    @Override
    public void reset() {
        super.reset();
        motorVoltage = 0;
    }
}
