// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveStick;
import frc.robot.commands.ResetGyro;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Stick;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.SwerveDrive;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here..
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public final SwerveDrive m_swervedriveSystem = new SwerveDrive();
  public final Stick driverJoystick =new Stick();
  Supplier<double[]> stickState = () -> driverJoystick.readDriverStick();

  public final DriveStick driveCommand = new DriveStick(m_swervedriveSystem,stickState); 

  public JoystickButton btnResetGyro = driverJoystick.getButton(2);
  public JoystickButton btnUpdateConstants = driverJoystick.getButton(3);

  // Replace with CommandPS4Controller or CommandJoystick if needed

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_swervedriveSystem.setDefaultCommand(driveCommand);
    // Configure the trigger bindings
    configureBindings();
  }


  private void configureBindings() {

    new Trigger(btnResetGyro).onTrue(new ResetGyro(m_swervedriveSystem));
    new Trigger(btnUpdateConstants).onTrue(m_swervedriveSystem.UpdateConstantsCommand());    

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
