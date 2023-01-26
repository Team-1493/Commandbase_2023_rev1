package frc.robot.subsystems;
// comment added from computer 12 for github test
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
    public PathPlannerTrajectory testPath1 = PathPlanner.loadPath("testPath1", new PathConstraints(4, 3));
    public PathPlannerTrajectory testPath2 = PathPlanner.loadPath("testPath2", new PathConstraints(4, 3));
    private HashMap<String, Command> eventMap = new HashMap<>();
    private SwerveDrive sds;

    //This method will be called once during the beginning of autonomous
    public AutoGenerator(SwerveDrive m_sds) {
        //defining the SwerveDrive used during autonomous as the instance given by the method
        sds = m_sds;

        //Putting all possible events in the global eventMap
        //(These print statements are placeholders for the actual commands that will be run)
        //Example of events that could be used for multiple paths:
        eventMap.put("intake_on", new PrintCommand("Intake On"));
        eventMap.put("intake_off", new PrintCommand("Intake Off"));
        eventMap.put("place_cube_2", new PrintCommand("Cube has been placed!"));
        eventMap.put("balance_robot", new PrintCommand("Balanced Robot"));
        eventMap.put("path_started", new PrintCommand("Path has started!"));
        eventMap.put("path_complete", new PrintCommand("Path is complete!"));

        //Examples of events that could be used for only one path (testPath):
        eventMap.put("testPath_marker1", new PrintCommand("Passed marker 1"));
        eventMap.put("testPath_marker1", new PrintCommand("Passed marker 2"));

    }
    
    //Builds and returns a PPSwerveControllerCommand for the given path
    public PPSwerveControllerCommand buildSwerveControlCommand(PathPlannerTrajectory retrievedPath) {
        return new PPSwerveControllerCommand(
            retrievedPath, 
            sds::getPose,
            SwerveDrive.m_kinematics, 
            new PIDController(1, 0, 0), 
            new PIDController(1, 0, 0), 
            new PIDController(1, 0, 0),
            sds::setModuleStates, 
            true,
            sds
        );
    }

    //Builds a FollowPathWithEvents using a given PathPlannerTrajectory
    public FollowPathWithEvents followEventBuilder(PathPlannerTrajectory retrievedPath) {
        // HashMap<String, Command> eventMap = new HashMap<>();
        // eventMap.put("Waypoint 1", new PrintCommand("nice"));

        return new FollowPathWithEvents(
            buildSwerveControlCommand(retrievedPath),
            retrievedPath.getMarkers(),
            eventMap
        );
    } 

    public SequentialCommandGroup autoCommand1() {
        return new SequentialCommandGroup(
            new InstantCommand( () -> sds.resetOdometry(testPath1.getInitialHolonomicPose())),
            buildSwerveControlCommand(testPath1),
            followEventBuilder(testPath1),
            new InstantCommand( () -> sds.allStop())
        );
    }

    public SequentialCommandGroup autoCommand2(){
        return new SequentialCommandGroup(
            new InstantCommand( () -> sds.resetOdometry(testPath1.getInitialHolonomicPose())),
            buildSwerveControlCommand(testPath2),
            followEventBuilder(testPath2),
            new InstantCommand( () -> sds.allStop())
        );
    }

}

