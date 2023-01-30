package frc.robot.commands;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.Sensors.Pigeon;
import java.lang.Math;

import edu.wpi.first.math.geometry.Rotation2d;

public class autobalancer { 
    public SwerveDrive sds;
    public Pigeon getheadingDegreesPigeon;
    public autobalancer(SwerveDrive m_sds) {
        sds = m_sds;
    }

    public void properheading() {
        double heading = sds.heading * (180/Math.PI);
        double desiredHeading=0; 
        desiredHeading = Math.round(heading/(Math.PI/2));

        sds.setMotors(0, 0, new Rotation2d(desiredHeading));
        if(desiredHeading == 0 || desiredHeading == 180){
        } 
        if(desiredHeading == 90 || desiredHeading == 270){
            //use pitch
        }
    }
}