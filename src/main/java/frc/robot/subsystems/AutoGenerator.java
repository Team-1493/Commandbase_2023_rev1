package frc.robot.subsystems;

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
    public PathPlannerTrajectory testPath = PathPlanner.loadPath("test path", new PathConstraints(4, 3));
    private HashMap<String, Command> eventMap = new HashMap<>();
    private SwerveDrive sds;

    public AutoGenerator(SwerveDrive m_sds) {
        sds = m_sds;
    }

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

    public FollowPathWithEvents followEventsBuilder(PathPlannerTrajectory retrievedPath, HashMap<String, Command> hashEventMap) {
        // HashMap<String, Command> eventMap = new HashMap<>();
        // eventMap.put("Waypoint 1", new PrintCommand("nice"));

        return new FollowPathWithEvents(
            buildSwerveControlCommand(retrievedPath),
            retrievedPath.getMarkers(),
            hashEventMap
        );
    } 

    public SequentialCommandGroup autoCommands() {
        return new SequentialCommandGroup(
            new InstantCommand( () -> sds.resetOdometry(testPath.getInitialHolonomicPose())),
            buildSwerveControlCommand(testPath),
            new InstantCommand( () -> followEventsBuilder(testPath, eventMap)),
            new InstantCommand( () -> sds.allStop())
        );
        
    }
}

