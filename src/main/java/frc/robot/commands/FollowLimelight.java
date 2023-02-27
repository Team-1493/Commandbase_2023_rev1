package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class FollowLimelight extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private SwerveDrive m_SwerveDrive;
  private Limelight m_limelight;
  private double kP_forward = -0.3;
  private double kD_forward = 0;
  private double kP_rotation = 0.05;
  private double kD_rotation = 0.0005;
  private double kP_side = -0.5;
  private double kD_side = 0.001;

  private PIDController RotPIDController;
  private PIDController FDPIDController;
  private ProfiledPIDController SidePIDController;


  private double rotation;
  private double sideDistance;
  private double forwardDistance;

  private double xVel = 0;
  private double yVel = 0;
  private double currentHeading;
  private double desiredHeading;

  private double rotationCutOff = 0.5;
  private double sideDistanceCutOff = 0.05;
  private double desiredForwardDistance = 2.5;
  private double forwardDistanceCutOff = 0.5;

  private double counter = 0;

  private double[] target;

  public FollowLimelight(SwerveDrive sd, Limelight limelight) {
    m_SwerveDrive = sd;
    m_limelight = limelight;
    SmartDashboard.putNumber("Limelight Rotation kP", kP_rotation);
    SmartDashboard.putNumber("Limelight Rotation kD", kD_rotation);
    SmartDashboard.putNumber("Limelight Rotation Cut Off", rotationCutOff);
    SmartDashboard.putNumber("Limelight Side kP", kP_side);
    SmartDashboard.putNumber("Limelight Side kD", kD_side);
    SmartDashboard.putNumber("Limelight Side Distance Cut Off", sideDistanceCutOff);
    SmartDashboard.putNumber("Limelight Forward kP", kP_forward);
    SmartDashboard.putNumber("Limelight Forward kD", kD_forward);
    SmartDashboard.putNumber("Limelight Desired Forward Distance", desiredForwardDistance);
    SmartDashboard.putNumber("Limelight Forward Distance Cut Off", forwardDistanceCutOff);


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sd, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    kP_rotation = SmartDashboard.getNumber("Limelight Rotation kP", kP_rotation);
    kD_rotation = SmartDashboard.getNumber("Limelight Rotation kD", kD_rotation);
    rotationCutOff  = SmartDashboard.getNumber("Limelight Rotation Cut Off", rotationCutOff);
    kP_side= SmartDashboard.getNumber("Limelight Side kP", kP_side);
    kD_side= SmartDashboard.getNumber("Limelight Side kD", kD_side);
    sideDistanceCutOff  = SmartDashboard.getNumber("Limelight Side Distance Cut Off", sideDistanceCutOff);
    kP_forward = SmartDashboard.getNumber("Limelight Forward kP", kP_forward);
    kD_forward = SmartDashboard.getNumber("Limelight Forward kD", kD_forward);
    desiredForwardDistance = SmartDashboard.getNumber("Limelight Desired Forward Distance", desiredForwardDistance);
    forwardDistanceCutOff  = SmartDashboard.getNumber("Limelight Forward Distance Cut Off", forwardDistanceCutOff);


    Constraints sideConstraint = new Constraints(2,1);
    FDPIDController = new PIDController(kP_forward, 0, kD_forward);
    RotPIDController = new PIDController(kP_rotation, 0, kD_rotation);
    SidePIDController = new ProfiledPIDController(kP_side, 0, kD_side,sideConstraint);
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

//CUTOFF 
    if (Math.abs(sideDistance) <= sideDistanceCutOff){
      sideDistance = 0;
    }
    if (Math.abs(forwardDistance) <= forwardDistanceCutOff){
      forwardDistance = 0;
    }
    


    //System.out.println("Tx: " + target[1] + " R: " + rotation + " H: " + m_SwerveDrive.heading);
    // FIELD CENTRIC MOTION - MOVING FORWARD TOWARD APRILTAG

    xVel = Math.cos(m_SwerveDrive.heading) * forwardDistance;// + Math.sin(sideDistance)*kP_sideDistance;// FIELD FORWARD
    yVel = Math.sin(m_SwerveDrive.heading) * forwardDistance + SidePIDController.calculate(sideDistance);// + Math.cos(sideDistance)*kP_sideDistance;// FIELD LEFT & RIGHT

    if (target[0] == 1) {
//      System.out.println("vx "+xVel+"  yvel "+yVel+"   rotation"+rotation);
      m_SwerveDrive.setMotors(xVel, yVel, rotation);
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
/*    if (Math.abs(target[1]) <= rotationCutOff && desiredForwardDistance - target[3] <= forwardDistanceCutOff) {
      counter += 1;
    }
    else{
      counter = 0;
    }*/
    return false;
  }
}
