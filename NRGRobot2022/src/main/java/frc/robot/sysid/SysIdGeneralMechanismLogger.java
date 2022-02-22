// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sysid;

/** Add your docs here. */
public class SysIdGeneralMechanismLogger extends SysIdLogger{

    private double motorVoltage = 0;

    @Override
    protected boolean isWrongMechanism() {
        String mechanism = getMechanism();
        return !mechanism.equals("Arm") && !mechanism.equals("Elevator") && !mechanism.equals("Simple");
    }

    public void log(double measuredPosition, double measuredVelocity) {
        updateData();

        addData(getTimestamp(), motorVoltage, measuredPosition, measuredVelocity);

        motorVoltage = getMotorVoltage();
    }

    @Override
    public void reset() {
        super.reset();
        motorVoltage = 0;
    }    
}
