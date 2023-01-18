// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SwerveDrive;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveStick extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveDrive m_SwerveDrive;
  private final Supplier<double[]> m_stickState;

  
  public DriveStick(SwerveDrive sd,Supplier<double[]> stickState) {
    m_SwerveDrive = sd;
    m_stickState=stickState;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_SwerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_SwerveDrive.setMotorsFromStick(m_stickState.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
