package frc.robot.commands;

import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveDrive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class ReflectiveTape extends CommandBase {
    PIDController TController = new PIDController(0.1, 0, 0);

    private SwerveDrive sds;
    private Limelight ll;
    int pipeline = 0;

    //NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    //NetworkTableEntry pipelineEntry = table.getEntry("pipeline");
    //NetworkTableEntry tx = table.getEntry("tx"); //yaw
    //NetworkTableEntry ty = table.getEntry("ty"); //pitch
    //NetworkTableEntry tv = table.getEntry("tv"); //0 or 1 value if has target

    double yaw;
    double pitch;
    boolean hasTarget;
    double omega;
    double vx; // forward and back
    double vy; // left and right
    double kpVy;
    boolean doneRotate = false;


    double[] target;

public ReflectiveTape(SwerveDrive m_sds, Limelight m_ll){
        sds=m_sds;
        ll = m_ll;
        addRequirements(sds, ll);
        SmartDashboard.putNumber("photon kP Yaw", 0.1);
        SmartDashboard.putNumber("photon kD Yaw", 0);
        SmartDashboard.putNumber("photon kP Vy", 2);
}

public void initialize() {
    doneRotate=false;
    TController.setSetpoint(0);
    double kpYaw=SmartDashboard.getNumber("photon kP Yaw", 0.05);
    TController.setP(kpYaw);
    double kdYaw=SmartDashboard.getNumber("photon kD Yaw", 0);
    kpVy=SmartDashboard.getNumber("photon kP Vy", 0);
    TController.setD(kdYaw);
    vx = 0;
    vy = 0;
}


    public void execute(){
      target = ll.getVisionTarget();
      SmartDashboard.putNumber("tx", target[1]);
      SmartDashboard.putNumber("Rbot theta", sds.heading);
      System.out.println("area = "+target[3]+"   ty="+target[2]);

      System.out.println("angle = "+sds.heading+"   tv="+target[0]+"   tx="+target[1]+"   kp="+TController.getP()+ "      kp vy=" + kpVy);

        //if it's in sight get the omega
        if (target[0] == 1){
          if (Math.abs(sds.heading) < 0.018 && Math.abs(target[1]) < 0.38) doneRotate=true;
          
          if (!doneRotate){
            omega = TController.calculate(target[1]);
            vy=sds.heading*kpVy;
            SmartDashboard.putNumber("Omega", omega);
            sds.setMotors(0, vy, omega);
          }

            else
             {
              sds.setMotors(0.6, 0, 0);
              }
            
            
        } 
/* 

            if (target[1] < 0.2 && target[1] > -0.2){
              if (sds.heading < -0.1){
                vy = -0.5;
                if (sds.heading < 0.1 && sds.heading > -0.1){
                  sds.setMotors(0, 0, 0);
                }

              }
              else if (sds.heading > 0){
                vy = 0.5;
                SmartDashboard.putNumber("Robot Theta", sds.heading);
                if (sds.heading < 0.1 && sds.heading > -0.1){
                  sds.setMotors(0, 0, 0);
                }
*/
              

            


        
        else  {sds.setMotors(0, 0, 0);
          SmartDashboard.putNumber("Photon has Target", 0);
        }
    }
    

    // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sds.allStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (target[3] > 0.55) {
      return true;
    }
    return false;
  }


}
