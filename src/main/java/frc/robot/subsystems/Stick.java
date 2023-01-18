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
  // [0]  squared magnitude in for/back direction (for is pos)
  // [1]  squared magnitude in left/right direction (right is pos)
  // [2]  provides rotational speed
  // all values are -1 to 1
  public double[] readDriverStick(){
    double[] stickValues = new double[3];
    double direction = joy0.getDirectionDegrees();
    double mag=joy0.getMagnitude();
    stickValues[0]= -mag*mag*Math.cos(direction*Math.PI/180);
    stickValues[1]=mag*mag*Math.sin(direction*Math.PI/180);
    stickValues[3]=joy0.getRawAxis(2);
    return stickValues;
  }

//   returns  a joystick button  (Not the value of the button!)
  public JoystickButton getButton(int buttonNumber){
    return new JoystickButton(joy0, buttonNumber);
  }

}
