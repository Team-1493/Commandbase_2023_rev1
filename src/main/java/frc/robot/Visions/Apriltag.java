package frc.robot.Visions;
// Author - Richard
// import the network modules neccessary for the april tag
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;

import frc.robot.subsystems.SwerveDrive;;

/*Still work in progress for april tags
 *
 * Plan
 * 
 * Take in network table values (done)
 * Use the pose estimations to move robot
 * 
 * 
 */


public class Apriltag {
    public double FPS;
    public double Tag_id;

    public double tx;
    public double ty;
    public double tz;

    public double rx;
    public double ry;
    public double rz;

    private NetworkTableEntry fps;
    private NetworkTableEntry tag_id;

    private NetworkTableEntry translation_x;
    private NetworkTableEntry translation_y;
    private NetworkTableEntry translation_z;

    private NetworkTableEntry rotation_x;
    private NetworkTableEntry rotation_y;
    private NetworkTableEntry rotation_z;
    

    Apriltag(){
        // instance for setting up Network entries
        NetworkTableInstance ntinst = NetworkTableInstance.getDefault();
        NetworkTable vision_nt = ntinst.getTable("Vision");

        // get fps, tag id from vision_nt
        fps = vision_nt.getEntry("fps");
        tag_id = vision_nt.getEntry("tag_id");

        /*Get pose estimations from vision_nt */
        // get translation estimations
        translation_x = vision_nt.getEntry("translation_x");
        translation_y = vision_nt.getEntry("translation_y");
        translation_z = vision_nt.getEntry("translation_z");
        
        // get rotation estimations
        rotation_x = vision_nt.getEntry("rotation_x");
        rotation_y = vision_nt.getEntry("rotation_y");
        rotation_z = vision_nt.getEntry("rotation_z");



    }

    public void moveToAprilTag () {
        
        FPS = fps.getDouble((0));
        Tag_id = tag_id.getDouble((0));
        
        tx = translation_x.getDouble((0)); // horizontal movement of the apriltag 1/10
        ty = translation_y.getDouble((0)); // vertical movement of the apriltag 1/10
        tz = translation_z.getDouble((0)); // the distance betweeen the april tag and the camera, (if the april tag is directly near the camera then, 0)

        rx = rotation_x.getDouble((0));  // the vertically flipping of the apriltag
        ry = rotation_y.getDouble((0)); // the horizontal flipping of the apriltag
        rz = rotation_z.getDouble((0)); // the rotation of the apriltag

        SwerveDrive drivemotors = new SwerveDrive();
        
        double velx = 0.0;
        double vely = 0.0;
        double angle = Math.atan((tx * 10)/(tz * 10));

        drivemotors.setMotors(0, 0, angle);

    }


}
