package frc.robot.commands;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.Sensors.Pigeon;
import java.lang.Math;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class autobalancer { 
    public SwerveDrive sds;
    public Pigeon getheadingDegreesPigeon;
    public double MaxRollValue = 3;
    public double MaxPitchValue = 3;


    public autobalancer(SwerveDrive m_sds) {
        sds = m_sds;
    }

    public void getandsetheading() {
        double desiredHeading=0;
        desiredHeading = Math.round(sds.heading/(Math.PI/2));
        double heading = desiredHeading * (180/Math.PI);
        
        sds.setMotors(0, 0, new Rotation2d(heading));
        heading=0;
        if(heading == 0){
            //use pitch
            Double pitch = Pigeon.Instance.getPitch();
            System.out.println("pitch = "+pitch);
            SmartDashboard.putNumber("using pitch", pitch);
            if (pitch > MaxPitchValue){
                Double xpos = sds.getPose().getX();
                Double Desiredxpos = xpos;
                while (xpos<Desiredxpos+1){
                 sds.setMotors(-0.5, 0, 0);
                 Desiredxpos = sds.getPose().getX();
                 SmartDashboard.putNumber("Auto-Desiredypos_0_pos_pitch", Desiredxpos);
                }
            }
            if (pitch < -MaxPitchValue){
                Double xpos = sds.xposMeters; 
                Double Desiredxpos = xpos;
                System.out.println("Init xpos="+xpos);
                while (pitch < -MaxPitchValue){
                    pitch = Pigeon.Instance.getPitch();
                    System.out.println("pitch"+pitch);
                    xpos = sds.xposMeters; 
                    System.out.println("xpos="+xpos);
                 sds.setMotors(0.5, 0, 0);
                 Desiredxpos = sds.xposMeters;
                }
            }
        } 
        if(heading == 180){
            //use pitch
            Double pitch = Pigeon.Instance.getPitch();
            SmartDashboard.putNumber("using pitch", pitch);
            if (pitch > MaxPitchValue){
                Double xpos = sds.getPose().getX();
                Double Desiredxpos = xpos;
                while (xpos<Desiredxpos-1){
                    sds.setMotors(-0.5, 0, 0);
                    Desiredxpos = sds.getPose().getX();
                    SmartDashboard.putNumber("Auto-Desiredypos_180_pos_pitch", Desiredxpos);

                }
            }
                if (pitch < -MaxPitchValue){
                    Double xpos = sds.getPose().getX();
                    Double Desiredxpos = xpos;
                    while (xpos>Desiredxpos+1){
                     sds.setMotors(0.5, 0, 0);
                     Desiredxpos = sds.getPose().getX();
                     SmartDashboard.putNumber("Auto-Desiredypos_180_neg_pitch", Desiredxpos);
                    }
                }
        }
        if(heading == 90 ){
            //use roll
            Double Roll = Pigeon.Instance.getRoll();
            SmartDashboard.putNumber("using pitch", Roll);

            if (Roll > MaxRollValue){
                Double ypos = sds.getPose().getY();
                Double Desiredypos = ypos;
                while (ypos<Desiredypos-1){
                    sds.setMotors(0, -0.5, 0);
                    Desiredypos = sds.getPose().getY();
                    SmartDashboard.putNumber("Auto-Desiredypos_90_pos_roll", Desiredypos);
                }
            }
            if (Roll < -MaxRollValue){
                Double ypos = sds.getPose().getY();
                Double Desiredypos = ypos;
                while (ypos>Desiredypos+1){
                    sds.setMotors(0, 0.5, 0);
                    Desiredypos = sds.getPose().getY();
                    SmartDashboard.putNumber("Auto-Desiredypos_90_neg_roll", Desiredypos);
                }
            }
        }
        
    
        
        if(heading == 270){
            //use roll
            Double Roll = Pigeon.Instance.getRoll();
            SmartDashboard.putNumber("using pitch", Roll);
            if (Roll > MaxRollValue){
                Double ypos = sds.getPose().getY();
                Double Desiredypos = ypos;
                while (ypos<Desiredypos-1){
                    sds.setMotors(0, -0.5, 0);
                    Desiredypos = sds.getPose().getY();
                    SmartDashboard.putNumber("Auto-Desiredypos_270_pos_roll", Desiredypos);
                }
                if (Roll < -MaxRollValue){
                    Desiredypos = sds.getPose().getY();
                    while (ypos>Desiredypos+1){
                        sds.setMotors(0, 0.5, 0);
                        Desiredypos = sds.getPose().getY();
                        SmartDashboard.putNumber("Auto-Desiredypos_270_neg_roll", Desiredypos);
                    }
                }
            }
        }
    }
}