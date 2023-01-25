// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;

import org.opencv.core.Mat.Tuple2;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public Limelight() {}
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");//X
  NetworkTableEntry ty = table.getEntry("ty");//Y
  NetworkTableEntry ta = table.getEntry("ta");//Area
  NetworkTableEntry tv = table.getEntry("tv");//bool target seen
  NetworkTableEntry ts = table.getEntry("ts");//rotation
 
  public double[] getVisionTarget() {
    double[] targetInfo = new double[4];
    targetInfo[0] = tx.getDouble(0.0);
    targetInfo[1] = ty.getDouble(0.0);
    targetInfo[2] = ta.getDouble(0.0);
    targetInfo[3] = ts.getDouble(0.0);
    return targetInfo;
  }

  public void switchVision(Boolean type){
    if(type){ // REFLECTIVE TAPE
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("<pipeline>").setNumber(0);
    }
    else{ // APRIL TAG
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("<pipeline>").setNumber(1);
    }
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
