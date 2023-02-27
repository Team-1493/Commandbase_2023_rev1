// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSystem extends SubsystemBase {
    final TalonFX TopConveyor;
    final TalonFX BotConveyor;
    final TalonFX FrontIntakeBar;
    final TalonFX RearIntakeBars;
    private double topConveyorSpeed = 1;
    private double botConveyorSpeed = 1.5;
    private double frontIntakePower = 0.3;
    private double rearIntakePower = 0.3;
    private double conveyorKf = 1024;
    private double shootSpeedMultiplier = 1;
    public IntakeSystem() {
        SmartDashboard.putNumber("Top Conveyor Speed",topConveyorSpeed);
        SmartDashboard.putNumber("Bot Conveyor Speed",botConveyorSpeed);
        SmartDashboard.putNumber("Front Intake Power",frontIntakePower);
        SmartDashboard.putNumber("Rear Intake Power",rearIntakePower);
        SmartDashboard.putNumber("Shoot Speed Multiplier", shootSpeedMultiplier);

        TopConveyor = new TalonFX(12);
        BotConveyor = new TalonFX(13);
        FrontIntakeBar = new TalonFX(10);
        RearIntakeBars = new TalonFX(11);
        TopConveyor.config_kF(0,conveyorKf );
        BotConveyor.config_kF(0,conveyorKf );
        TopConveyor.setInverted(InvertType.InvertMotorOutput);
        FrontIntakeBar.setInverted(InvertType.InvertMotorOutput);
    }

  /**
   * Example command factory method.
   *
   * @return a command
   */

  public CommandBase UpdateConstants(){
    return runOnce(() ->{
      topConveyorSpeed = SmartDashboard.getNumber("Top Conveyor Speed",topConveyorSpeed);
      botConveyorSpeed = SmartDashboard.getNumber("Bot Conveyor Speed",botConveyorSpeed);
      frontIntakePower = SmartDashboard.getNumber("Front Intake Power",frontIntakePower);
      rearIntakePower = SmartDashboard.getNumber("Rear Intake Power",rearIntakePower);
      shootSpeedMultiplier = SmartDashboard.getNumber("Shoot Speed Multiplier", shootSpeedMultiplier);
    });
  }

  public CommandBase IntakeCube() {
    return runOnce(() -> {
      TopConveyor.set(ControlMode.Velocity, topConveyorSpeed);
      BotConveyor.set(ControlMode.Velocity, botConveyorSpeed);
      FrontIntakeBar.set(ControlMode.PercentOutput, frontIntakePower);
      RearIntakeBars.set(ControlMode.PercentOutput, rearIntakePower);
    });
  }

  public CommandBase GrabCone() {
    return runOnce(() -> {
      FrontIntakeBar.set(ControlMode.PercentOutput, frontIntakePower);
      RearIntakeBars.set(ControlMode.PercentOutput, -rearIntakePower);
    });
  }

  public CommandBase DropCone() {
    return runOnce(() -> {
      FrontIntakeBar.set(ControlMode.PercentOutput, -frontIntakePower);
      RearIntakeBars.set(ControlMode.PercentOutput, rearIntakePower);
    });
  }

  

  public CommandBase StopMotors(){
    return runOnce(() -> {TopConveyor.set(ControlMode.Velocity, 0);
    BotConveyor.set(ControlMode.Velocity, 0);
    FrontIntakeBar.set(ControlMode.PercentOutput, 0);
    RearIntakeBars.set(ControlMode.PercentOutput, 0);
  });
  }


  
  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
