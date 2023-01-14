package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

// This is more of a utility subsystem than an actual subsystem

public class Stick extends SubsystemBase {
  public Joystick joy0=new Joystick(0);
  
  public Stick() {

  }

  // read the joystick and return an array containing:
  // [0]  squared magnitude of axis 0, provides drive speed 
  // [1]  angle of axis zero,  provides drive direction
  // [2]  raw axis value axis 2, provides rotational speed
  public double[] readDriverStick(){
    double[] stickValues = new double[3];
    stickValues[0]=joy0.getMagnitude();
    stickValues[0] = stickValues[0]*stickValues[0];
    stickValues[1] = joy0.getDirectionDegrees();
    stickValues[2] = joy0.getRawAxis(2)*0.5;
    return stickValues;

  }

//   returns  a joystick button  (Not the value of the button!)
  public JoystickButton getButton(int buttonNumber){
    return new JoystickButton(joy0, buttonNumber);
  }

}
