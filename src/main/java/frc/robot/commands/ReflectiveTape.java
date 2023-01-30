package frc.robot.commands;

import frc.robot.subsystems.SwerveDrive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class ReflectiveTape extends CommandBase {
    PhotonCamera camera = new PhotonCamera("IMX219");
    PIDController TController = new PIDController(0.01, 0, 0);

    private SwerveDrive sds;

    public PhotonPipelineResult result;
    public boolean hasTarget;

    public PhotonTrackedTarget target;

    private double yaw;
    private double pitch;
    private double skew;
    private double area;
    private double omega;

    private double range;

public ReflectiveTape(SwerveDrive m_sds){
        sds=m_sds;
        addRequirements(sds);
}

public void initialize() {
    
}

    public void execute(){
        //check if there is reflective tape in sight
        result = camera.getLatestResult();
        hasTarget = result.hasTargets();

        TController.setSetpoint(0);

        //if it's in sight get the omega
        if (hasTarget){
            target = result.getBestTarget();
            yaw = target.getYaw();
            omega = -TController.calculate(yaw);
            sds.setMotors(0, 0, omega);
        }
        else  sds.setMotors(0, 0, 0);
    }
    

    // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sds.setMotors(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }


}
