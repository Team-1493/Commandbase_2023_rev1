// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj2.command.InstantCommand;

  public class ResetGyro extends InstantCommand {
    public ResetGyro(SwerveDrive swervedrive) {
      super(swervedrive::resetGyro, swervedrive);
    }
  }


