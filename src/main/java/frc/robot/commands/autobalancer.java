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
        if(heading<10 && heading>350){
            //use pitch
            Double pitch = Pigeon.Instance.getPitch();
            System.out.println("pitch = "+pitch);
            SmartDashboard.putNumber("using pitch", pitch);
            if (pitch > MaxPitchValue){
                Double xpos = sds.getPose().getX();
                Double Desiredxpos = xpos;
                while (pitch > MaxPitchValue){
                    pitch = Pigeon.Instance.getPitch();
                    System.out.println("pitch"+pitch);
                    sds.setMotors(-0.5, 0, 0);
                    xpos = sds.xposMeters;  

                }
            }
            if (pitch < -MaxPitchValue){
                Double xpos = sds.xposMeters; 
                System.out.println("Init xpos="+xpos);
                while (pitch < -MaxPitchValue){
                    pitch = Pigeon.Instance.getPitch();
                    System.out.println("pitch"+pitch);
                    xpos = sds.xposMeters;  
                    System.out.println("xpos="+xpos);
                    sds.setMotors(0.5, 0, 0);
                    Double Desiredxpos = sds.xposMeters;
                    System.out.println("desired xpos = " +Desiredxpos);
                
                }
            }
        } 
        if(heading>170 && heading<190){
            //use pitch
            Double pitch = Pigeon.Instance.getPitch();
            SmartDashboard.putNumber("using pitch", pitch);
            if (pitch > MaxPitchValue){
                Double xpos = sds.xposMeters; 
                while (pitch > MaxPitchValue){
                    pitch = Pigeon.Instance.getPitch();
                    System.out.println("pitch"+pitch);
                    sds.setMotors(-0.5, 0, 0);
                    xpos = sds.xposMeters; 
                    System.out.println("xpos = "+xpos);

                }
            }
                if (pitch < -MaxPitchValue){
                    Double xpos = sds.getPose().getX();
                    while (pitch < -MaxPitchValue){
                        pitch = Pigeon.Instance.getPitch();
                        sds.setMotors(0.5, 0, 0);
                        xpos = sds.xposMeters; 
                        System.out.println("xpos = "+xpos);
                    }
                }
        }
        if(heading>80 && heading<100){
            //use roll
            Double Roll = Pigeon.Instance.getRoll();
            SmartDashboard.putNumber("using pitch", Roll);

            if (Roll > MaxRollValue){
                Double ypos = sds.yposMeters; 
                while (Roll > MaxRollValue){
                    Roll = Pigeon.Instance.getRoll();
                    System.out.println("Roll"+Roll);
                    sds.setMotors(0, -0.5, 0);
                    ypos = sds.yposMeters; 
                    System.out.println("ypos = "+ypos);
                }
            }
            if (Roll < -MaxRollValue){
                Double ypos = sds.yposMeters; 
                while (Roll < -MaxRollValue){
                    Roll = Pigeon.Instance.getRoll();
                    sds.setMotors(0, 0.5, 0);
                    ypos = sds.yposMeters; 
                    System.out.println("ypos = "+ypos);
                }
            }
        }
        
    
        //not updated yet/
        if(heading>260 && heading<280){
            //use roll
            Double Roll = Pigeon.Instance.getRoll();
            SmartDashboard.putNumber("using pitch", Roll);
            if (Roll > MaxRollValue){
                Double ypos = sds.yposMeters; 
                Double Desiredypos = ypos;
                while (ypos<Desiredypos-1){
                    sds.setMotors(0, -0.5, 0);
                    Desiredypos = sds.yposMeters;
                    SmartDashboard.putNumber("Auto-Desiredypos_270_pos_roll", Desiredypos);
                }
                if (Roll < -MaxRollValue){
                    Desiredypos = sds.yposMeters; 
                    while (Roll < -MaxRollValue){
                        sds.setMotors(0, 0.5, 0);
                        Desiredypos = sds.yposMeters;
                        SmartDashboard.putNumber("Auto-Desiredypos_270_neg_roll", Desiredypos);
                    }
                }
            }
        }
    }
}