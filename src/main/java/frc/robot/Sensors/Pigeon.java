package frc.robot.Sensors;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;

import edu.wpi.first.math.geometry.Rotation2d;

public class Pigeon {
     Pigeon2 pigeon;

    public Pigeon(int ID){
        pigeon = new Pigeon2(20);
        pigeon.configFactoryDefault();      
        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_1_General, 251);
        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.RawStatus_4_Mag, 251);
        resetAngle();

    }

// returns a heading angle in the rangle of -180 to +180
// where 0 is facing downfield (or starting orientation)
// and angle is measured CCW  (negative angle CW)    
    public double getHeadingDegrees(){
        double[] ypr_deg = new double[3];
        pigeon.getYawPitchRoll(ypr_deg);
        double angle=ypr_deg[0] % 360;
        if (angle>180) angle=angle-360;
        else if (angle <-180) angle = angle + 360;
        return angle;
    } 

    public double getHeadingRadians(){
        return getHeadingDegrees()*Math.PI/180.;
    }

    // returns a Rotation2d object, angle in radians
    public Rotation2d getRotation2d(){
        return new Rotation2d(Math.PI*getHeadingDegrees()/180.);
    }



    public void resetAngle(){
        pigeon.setYaw(0);
    }

    public void setAngle(double heading){
        pigeon.setYaw(heading);
    }

    public double getTemperature(){
        return  pigeon.getTemp();
    }

    public void calibrate(){
//        pigeon.enterCalibrationMode(CalibrationMode.Temperature);
    }



}
