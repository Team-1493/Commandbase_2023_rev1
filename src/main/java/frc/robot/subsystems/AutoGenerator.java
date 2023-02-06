package frc.robot.subsystems;
// comment added from computer 12 for github test
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveModule;



public class AutoGenerator extends SubsystemBase{
    //Loading all autonomous paths and defining them as variables
    public PathPlannerTrajectory testPath1 = PathPlanner.loadPath("testPath1", new PathConstraints(4, 3));
    public PathPlannerTrajectory testPath2 = PathPlanner.loadPath("testPath2", new PathConstraints(4, 3));
    
    

    //Defining a HashMap called eventMap, which will store all events that can run during auto
    private HashMap<String, Command> eventMap = new HashMap<>();
    
    //Defining the SwerveDrive used during autonomous
    private SwerveDrive sds;
    
    PIDController thetaController = new PIDController(.01, 0, 0);
    PIDController positionController = new PIDController(0.01, 0, 0);


    //This method will be called once during the beginning of autonomous
    public AutoGenerator(SwerveDrive m_sds) {
        //defining the SwerveDrive used during autonomous as the instance given by the method
        sds = m_sds;
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        
        //Putting Position PID values in the SmartDashboard
        SmartDashboard.putNumber("Position_kP", positionController.getP());
        SmartDashboard.putNumber("Position_kI", positionController.getI());
        SmartDashboard.putNumber("Position_kD", positionController.getD());
        
        //Putting Rotation PID values in the SmartDashboard
        SmartDashboard.putNumber("Rotation_kP", thetaController.getP());
        SmartDashboard.putNumber("Rotation_kI", thetaController.getI());
        SmartDashboard.putNumber("Rotation_kD", thetaController.getD());



        //Putting all default Smartdashboard values relating to auto paths
        SmartDashboard.putBoolean("Intake_Is_On", false);
        SmartDashboard.putNumber("Path_position", 0.00);

        //Putting all possible events in the global eventMap
        //(PrintCommand = placeholder)
        //Example of events that could be used for multiple paths:
        eventMap.put("intake_off", new PrintCommand("Intake Off"));

        eventMap.put("intake_on", new InstantCommand(() -> SmartDashboard.putBoolean("Intake_Is_On", true)));
        
        eventMap.put("place_cube_2", new PrintCommand("Cube has been placed!"));
        eventMap.put("balance_robot", new PrintCommand("Balanced Robot"));
        eventMap.put("path_started", new PrintCommand("Path has started"));
        eventMap.put("path_ended", new PrintCommand("Path has ended"));

        eventMap.put("marker_1", new InstantCommand(() -> SmartDashboard.putNumber("Path_position", 0.0)));
        eventMap.put("marker_2", new InstantCommand(() -> SmartDashboard.putNumber("Path_position", 0.5)));
        eventMap.put("marker_3", new InstantCommand(() -> SmartDashboard.putNumber("Path_position", 1.0)));
        
        
        //double timeEnd = testPath1.getEndState().timeSeconds;
        //double t = 0;
        //while  (t<timeEnd){
        //    State state = testPath1.sample(t);
        //    System.out.println(state.poseMeters.getX()+",  "+state.poseMeters.getY()+", "+state.poseMeters.getRotation().getDegrees()+", "+", "+state.velocityMetersPerSecond);
        //    t=t+0.1;
        //};
 //       System.out.println(testPath1.getMarkers().get(1));
 //       System.out.println(testPath1.getMarkers().get(2));
//       System.out.println(testPath1.getMarkers().get(3));
    }
    
    //Builds and returns a PPSwerveControllerCommand for the given path
    public PPSwerveControllerCommand buildSwerveControlCommand(PathPlannerTrajectory retrievedPath) {
        return new PPSwerveControllerCommand(
            retrievedPath, 
            sds::getPose,
            SwerveDrive.m_kinematics, 
            positionController, //x
            positionController, //y
            thetaController, //rotation
            sds::setModuleStates, 
            true,
            sds
        );
    }

    //Builds a FollowPathWithEvents using a given PathPlannerTrajectory
    public FollowPathWithEvents followEventBuilder(PathPlannerTrajectory retrievedPath) {
        return new FollowPathWithEvents(
            buildSwerveControlCommand(retrievedPath),
            retrievedPath.getMarkers(),
            eventMap
        );
    } 

    public void updatePID(){
        positionController.setPID(SmartDashboard.getNumber("Position_kP", positionController.getP()),SmartDashboard.getNumber("Position_kI", positionController.getI()),SmartDashboard.getNumber("Position_kD", positionController.getD()));
        thetaController.setPID(SmartDashboard.getNumber("Rotation_kP", positionController.getP()),SmartDashboard.getNumber("Rotation_kI", positionController.getI()),SmartDashboard.getNumber("Rotation_kD", positionController.getD()));
    }

    public SequentialCommandGroup autoCommand1() {
        return new SequentialCommandGroup(
            new InstantCommand( () -> sds.resetOdometry(testPath1.getInitialHolonomicPose())),
            followEventBuilder(testPath1),
            new InstantCommand( () -> sds.allStop())
        );
    }

    public SequentialCommandGroup autoCommand2(){
        return new SequentialCommandGroup(
            new InstantCommand( () -> sds.resetOdometry(testPath2.getInitialHolonomicPose())),
            followEventBuilder(testPath2),
            new InstantCommand( () -> sds.allStop())
        );
    }

}

