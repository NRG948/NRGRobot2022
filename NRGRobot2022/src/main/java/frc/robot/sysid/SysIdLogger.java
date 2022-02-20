// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sysid;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** An abstract base class enabling SysId integration. */
public abstract class SysIdLogger {
    private static final int DATA_ARRAY_CAPACITY = 36000;

    private double voltageCommand;
    private double motorVoltage;
    private double timestamp;
    private double startTime;
    boolean rotate;
    String testType;
    String mechanism;
    ArrayList<Double> data = new ArrayList<Double>();

    /** Construct an instance of this class. */
    public SysIdLogger() {
        SmartDashboard.putNumber("SysIdVoltageCommand", 0.0);
        SmartDashboard.putString("SysIdTestType", "");
        SmartDashboard.putString("SysIdTest", "");
        SmartDashboard.putBoolean("SysIdRotate", false);
        SmartDashboard.putBoolean("SysIdOverflow", false);
        SmartDashboard.putBoolean("SysIdWrongMech", false);
    }

    /** Initializes the logger. */
    public void init() {
        mechanism = SmartDashboard.getString("SysIdTest", "");

        SmartDashboard.putBoolean("SysIdIsWrongMech", isWrongMechanism());

        testType = SmartDashboard.getString("SysIdTestType", "");
        rotate = SmartDashboard.getBoolean("SysIdRotate", false);
        voltageCommand = SmartDashboard.getNumber("SysIdVoltageCommand", 0.0);
        startTime = Timer.getFPGATimestamp();
        data.ensureCapacity(DATA_ARRAY_CAPACITY);
        data.clear();
    }

    /** Returns the motor voltage */
    public double getMotorVoltage() {
        return motorVoltage;
    }

    /** Updates the logger data from the current SysId values. */
    public void updateData() {
        timestamp = Timer.getFPGATimestamp();

        if (isWrongMechanism()) {
            motorVoltage = 0.0;
        } else if (testType.equals("Quasistatic")) {
            motorVoltage = voltageCommand * (timestamp - startTime);
        } else if (testType.equals("Dynamic")) {
            motorVoltage = voltageCommand;
        } else {
            motorVoltage = 0.0;
        }
    }

    /** Sends the data to the SysId tool. */
    public void sendData() {
        System.out.println(String.format("Collected %d data points.", data.size()));

        SmartDashboard.putBoolean("SysIdOverflow", data.size() >= DATA_ARRAY_CAPACITY);

        StringBuilder b = new StringBuilder();
        for (int i = 0; i < data.size(); ++i) {
            if (i != 0)
                b.append(",");
            b.append(data.get(i));
        }

        SmartDashboard.putString("SysIdTelemetry", b.toString());

        reset();
    }

    /** Resets the logger. */
    public void reset() {
        motorVoltage = 0.0;
        timestamp = 0.0;
        startTime = 0.0;
        data.clear();
    }

    /** Returns true if the wrong mechanism is being profiled. */
    protected abstract boolean isWrongMechanism();
}
