// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.concurrent.Future;

/** Add your docs here. */
public interface FutureSupplier<Type> {
    Future<Type> getFuture();
    
}
