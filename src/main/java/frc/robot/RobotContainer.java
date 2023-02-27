// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.AutoGenerator;
import frc.robot.commands.DriveStick;
import frc.robot.commands.FollowLimelight;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.autobalancer;
import frc.robot.commands.ReflectiveTape;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.Stick;
import frc.robot.subsystems.Limelight;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
  public final Limelight m_Limelight = new Limelight();
  public final IntakeSystem m_IntakeSystem = new IntakeSystem();
  public final AutoGenerator autoGenerator = new AutoGenerator(m_swervedriveSystem);
  public final Stick driverJoystick =new Stick();
<<<<<<< HEAD
  public final ReflectiveTape reflectivetape = new ReflectiveTape(m_swervedriveSystem);
  public final autobalancer m_autobalancer = new autobalancer(m_swervedriveSystem);
=======
  public final ReflectiveTape reflectivetape = new ReflectiveTape(m_swervedriveSystem, m_Limelight);
>>>>>>> 78619456452df5316ec1ce8f8942597f83bfb142
  Supplier<double[]> stickState = () -> driverJoystick.readDriverStick();

  public final DriveStick driveCommand = new DriveStick(m_swervedriveSystem,stickState); 

  public JoystickButton btnAimAtTape = driverJoystick.getButton(1);
  public JoystickButton btnResetGyro = driverJoystick.getButton(2);
  public JoystickButton btnUpdateConstants = driverJoystick.getButton(3);
  public JoystickButton btnFollowLimelight = driverJoystick.getButton(4);
  public JoystickButton coneGrabberForward = driverJoystick.getButton(6); // R1
  public JoystickButton coneGrabberBackward = driverJoystick.getButton(5); //L1
<<<<<<< HEAD
  public JoystickButton autobalance = driverJoystick.getButton(7);

=======
  public JoystickButton btnIntakeCube = driverJoystick.getButton(7);
  public JoystickButton btnGrabCone = driverJoystick.getButton(8);
>>>>>>> 78619456452df5316ec1ce8f8942597f83bfb142

  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_swervedriveSystem.setDefaultCommand(driveCommand);
    // Configure the trigger bindings
    configureBindings();
  }


  private void configureBindings() {

    new Trigger(btnResetGyro).onTrue(new ResetGyro(m_swervedriveSystem));
    new Trigger(btnUpdateConstants).onTrue(m_swervedriveSystem.UpdateConstantsCommand());    
    new Trigger(btnUpdateConstants).onTrue(m_IntakeSystem.UpdateConstants());    
    new Trigger(btnFollowLimelight).whileTrue(new FollowLimelight(m_swervedriveSystem, m_Limelight));
    new Trigger(btnAimAtTape).whileTrue(reflectivetape);
<<<<<<< HEAD
    new Trigger(autobalance).whileTrue(new InstantCommand( () -> m_autobalancer.getandsetheading()));
=======
    new Trigger(btnIntakeCube).onTrue(m_IntakeSystem.IntakeCube());
    new Trigger(btnGrabCone).onTrue(m_IntakeSystem.GrabCone());
    new Trigger(btnIntakeCube).onFalse(m_IntakeSystem.StopMotors());
    new Trigger(btnGrabCone).onFalse(m_IntakeSystem.StopMotors());


>>>>>>> 78619456452df5316ec1ce8f8942597f83bfb142
    new Trigger(driverJoystick.pov0).onTrue(m_swervedriveSystem.rotateInPlace(0.));
    new Trigger(driverJoystick.pov0).onTrue(m_swervedriveSystem.rotateInPlace(0.));
    new Trigger(driverJoystick.pov90).onTrue(m_swervedriveSystem.rotateInPlace(90));
    new Trigger(driverJoystick.pov180).onTrue(m_swervedriveSystem.rotateInPlace(180));   
    new Trigger(driverJoystick.pov270).onTrue(m_swervedriveSystem.rotateInPlace(-90));
    
/*  Tayab - you need to rethink these
 
    new Trigger(moveMotorForward).onTrue(grabbingCone.rotateMotorFoward.until(coneLimitSwitch::get));
    new Trigger(moveMotorBackward).onTrue(grabbingCone.rotateMotorBackward.until(coneLimitSwitch::get));

    new Trigger(coneGrabberForwardButton).onTrue(grabbingCone.rotateMotorFoward);
    new Trigger(coneGrabberBackwardButton).onTrue(grabbingCone.rotateMotorBackward);
 */

  }


  public Command getAutonomousCommand1() {
    // An example command will be run in autonomou
    autoGenerator.updatePID();;
    return autoGenerator.autoCommand1();
  }

  public Command getAutonomousCommand2() {
    // An example command will be run in autonomous
    return autoGenerator.autoCommand2();
  }

// We have different PID constants for the drive wheels between teleop and auto
// Switch between slot 0 for teleop and slot 1 for auto 
  public void setPIDslot(int slot){
    m_swervedriveSystem.setPIDSlot(slot);
  }



}
