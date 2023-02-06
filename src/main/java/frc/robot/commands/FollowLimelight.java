package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;

public class FollowLimelight extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private SwerveDrive m_SwerveDrive;
  private Limelight m_limelight;
  private double kP_rotation = 0.04;
  private double kD_rotation = 0.002;
  private double kP_forwardDistance = -0.3;
  private double kD_forwardDistance = 0;
  private double kP_sideDistance = 1.25;
  private double kD_sideDistance = 0.1;

  private PIDController RotPIDController;
  private PIDController FDPIDController;
  private PIDController SidePIDController;


  private double rotation;
  private double sideDistance;
  private double forwardDistance;

  private double xVel = 0;
  private double yVel = 0;
  private double currentHeading;
  private double desiredHeading;

  private double rotationCutOff = 0.5;
  private double sideDistanceCutOff = 0.1;
  private double desiredForwardDistance = 3;
  private double forwardDistanceCutOff = 0.5;

  private double[] target;

  public FollowLimelight(SwerveDrive sd, Limelight limelight) {
    m_SwerveDrive = sd;
    m_limelight = limelight;
    SmartDashboard.putNumber("Limelight Rotation kP", kP_rotation);
    SmartDashboard.putNumber("Limelight Rotation kD", kD_rotation);
    SmartDashboard.putNumber("Limelight Rotation Cut Off", rotationCutOff);
    SmartDashboard.putNumber("Limelight Side Distance kP", kP_sideDistance);
    SmartDashboard.putNumber("Limelight Side Distance kD", kD_sideDistance);
    SmartDashboard.putNumber("Limelight Forward Distance kP", kP_forwardDistance);
    SmartDashboard.putNumber("Limelight Forward Distance kD", kD_forwardDistance);
    SmartDashboard.putNumber("Limelight Desired Forward Distance", desiredForwardDistance);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sd, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    kP_rotation = SmartDashboard.getNumber("Limelight Rotation kP", kP_rotation);
    kD_rotation = SmartDashboard.getNumber("Limelight Rotation kD", kD_rotation);
    rotationCutOff  = SmartDashboard.getNumber("Limelight Rotation Cut Off", rotationCutOff);
    kP_sideDistance= SmartDashboard.getNumber("Limelight Side Distance kP", kP_sideDistance);
    kD_sideDistance= SmartDashboard.getNumber("Limelight Side Distance kD", kD_sideDistance);
    kP_forwardDistance = SmartDashboard.getNumber("Limelight Forward Distance kP", kP_forwardDistance);
    kD_forwardDistance = SmartDashboard.getNumber("Limelight Forward Distance kD", kD_forwardDistance);
    desiredForwardDistance = SmartDashboard.getNumber("Limelight Desired Forward Distance", desiredForwardDistance);

    FDPIDController = new PIDController(kP_forwardDistance, 0, kD_forwardDistance);
    RotPIDController = new PIDController(kP_rotation, 0, kD_rotation);
    SidePIDController = new PIDController(kP_sideDistance, 0, kP_sideDistance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentHeading = m_SwerveDrive.heading;
    target = m_limelight.getVisionTarget();

    rotation = RotPIDController.calculate(target[1]);//target[1]*kP_rotation;

    forwardDistance = FDPIDController.calculate(desiredForwardDistance - target[3]);//(desiredForwardDistance - target[3])*kP_forwardDistance;
    
    
    if (Math.abs(currentHeading-0) > Math.abs(currentHeading-Math.PI)){
      desiredHeading = 0;
    }
    else{
      desiredHeading = Math.PI;
    }

    desiredHeading=0;
    sideDistance = (currentHeading-desiredHeading);
//    if (sideDistance>Math.PI){
//      sideDistance = sideDistance - 2*Math.PI;
//    }
    


    //System.out.println("Tx: " + target[1] + " R: " + rotation + " H: " + m_SwerveDrive.heading);
    // FIELD CENTRIC MOTION - MOVING FORWARD TOWARD APRILTAG

    xVel = Math.cos(m_SwerveDrive.heading) * forwardDistance;// + Math.sin(sideDistance)*kP_sideDistance;// FIELD FORWARD
    yVel = Math.sin(m_SwerveDrive.heading) * forwardDistance + SidePIDController.calculate(sideDistance);// + Math.cos(sideDistance)*kP_sideDistance;// FIELD LEFT & RIGHT

    if (target[0] == 1) {
      System.out.println("vx "+xVel+"  yvel "+yVel+"   rotation"+rotation);
      m_SwerveDrive.setMotors(0, yVel, rotation);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_SwerveDrive.allStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
//    if (Math.abs(target[1]) <= rotationCutOff){// && desiredForwardDistance - target[3] <= forwardDistanceCutOff) {
//      return true;
 //   }
    return false;
  }
}
