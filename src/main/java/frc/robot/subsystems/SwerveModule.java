// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;




public class SwerveModule{
    public final TalonFX m_drive;
    private final TalonFX m_turn;
    private final CANCoder e_turn;
    private final double piOver180 = Math.PI/180.0;
    SimpleMotorFeedforward feedforward_drive_T;
    SimpleMotorFeedforward feedforward_drive_A;
    SimpleMotorFeedforward feedforward_drive;
    double voltageComp=11.5;
    double speedPrev=0;


    // Robot Dimensions for MK4 Swerve
    private  double  wheelDiamInches = 4.084;
    private double wheelCircumferenceMaters=wheelDiamInches*Math.PI*0.0254; // 0.32588 
    private double gearRatioDrive=8.1428; 
    // 1499  (3mps = 4695 rpm)
    private double MPSToRPM = 60.0*gearRatioDrive/wheelCircumferenceMaters;  
    // convert velocity in meters/sec to Talon speed unit (encoder counts/100ms)
    private double MPSToNativeSpeed = MPSToRPM*2048.0/600.0;  
    // convert radians to encoder counts - 4096 counts per rotation
    private double RadiansToNativePos=4096.0/(2*Math.PI);


    // Drive Motor Constants
    private double kP_drive=0.0;  //1.19 from characterization;
    private double kF_drive=0.0;   // 1023/20660
    private double kD_drive=0.0;   // 1023/20660
    private double kS_drive= 0.0399; 
    private double kV_drive= 0.260;  // 
    private double kA_drive= 0.0;  //
   

   // Drive Motor Constants for auto
   private double kP_driveAuto=0.025;  // 0.0514 from SYSID
   private double kF_driveAuto=0.0;   // 1023/20660
   private double kD_driveAuto=0.0;   // 1023/20660
   private double kS_driveAuto= 0.0399;  //   0.6058/12 = 0.0504 from SYSID
   private double kV_driveAuto= 0.240;  //  3.0562/12 = 0.254 from SYSID 
   private double kA_driveAuto= 0.033;  // 0.23728 /12 = 0.024 from SYSID
  
//  Turn (Swerve) Motor Constants
    private double kP_turn=0.5; //0.5,   0.421 from characterization 
    private double kD_turn=0;  // 0 from characterization    
    private double kF_turn=0.0;   //  max turn RPM on ground = 485, 3310 units/100ms


    private double kMaxOutputDrive=1;
    private double kMaxOutputTurn=0.8;

    String m_name;    

public SwerveModule(String name, int driveID, int turnID, int cancoderID, double zeropos){     
// set up the drive motor        
    m_name = name;                         
    m_drive=new TalonFX(driveID);
    m_drive.configFactoryDefault();
    m_drive.configVoltageCompSaturation(voltageComp);
    m_drive.enableVoltageCompensation(true);
    m_drive.setNeutralMode(NeutralMode.Brake);
    m_drive.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0, 25);
    m_drive.configVelocityMeasurementWindow(8, 10);

    m_drive.setStatusFramePeriod(21, 20);
    m_drive.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat,251);
    m_drive.setStatusFramePeriod(8,249);
    m_drive.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic,239);
    m_drive.setStatusFramePeriod(StatusFrame.Status_12_Feedback1,233);
    m_drive.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1,229);
    m_drive.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus,227);

  


    m_drive.config_kP(0, kP_drive);
    m_drive.config_kF(0, kF_drive);
    m_drive.config_kD(0, kD_drive);
    m_drive.config_kP(1, kP_driveAuto);
    m_drive.config_kF(1, kF_driveAuto);
    m_drive.config_kD(1, kD_driveAuto);


    m_drive.configClosedLoopPeakOutput(0,kMaxOutputDrive);
                    


// set up the turn encoder
    e_turn=new CANCoder(cancoderID);
    e_turn.configSensorDirection(false);                    
    e_turn.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    e_turn.setPosition(e_turn.getAbsolutePosition()-zeropos);

    e_turn.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 231);  

    // set up the turn motor    
    m_turn=new TalonFX(turnID);
    
    //  reduce the rate of status update to reduce CAN trffic
    m_turn.setStatusFramePeriod(21, 20);
    m_turn.setStatusFramePeriod(8,249);
    m_turn.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat,251);
    m_turn.setStatusFramePeriod(StatusFrame.Status_12_Feedback1,251);
    m_turn.setStatusFramePeriod(StatusFrame.Status_7_CommStatus,251);
    m_turn.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 249);
    m_turn.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 249);
    m_turn.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 249);
    m_turn.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic,239);
    m_turn.setStatusFramePeriod(StatusFrame.Status_12_Feedback1,233);
    m_turn.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1,229);
    m_turn.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus,227);
    m_turn.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 255);
    m_turn.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 249);
    

    m_turn.configFactoryDefault();
    m_turn.setNeutralMode(NeutralMode.Brake);

    m_turn.configRemoteFeedbackFilter(e_turn, 0, 25);
    m_turn.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0,0,25);       
//    m_turn.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);                
    m_turn.configForwardSoftLimitEnable(false);
    m_turn.configReverseSoftLimitEnable(false);
    m_turn.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Signed_PlusMinus180);
    m_turn.configPeakOutputForward(1);
    m_turn.configPeakOutputReverse(-1);
    m_turn.configClosedLoopPeakOutput(0, 0.8);
    m_turn.configSelectedFeedbackCoefficient(1);
    m_turn.config_kP(0,kP_turn);
    m_turn.config_kD(0,kD_turn);
    m_turn.config_kF(0,kF_turn);
    

    feedforward_drive_T=new SimpleMotorFeedforward(kS_drive, kV_drive, kA_drive);
    feedforward_drive_A=new SimpleMotorFeedforward(kS_driveAuto, kV_driveAuto, kA_driveAuto);
    feedforward_drive=feedforward_drive_T;               

    SmartDashboard.putNumber("kD_Drive",kD_drive);
    SmartDashboard.putNumber("kP_Drive",kP_drive);
    SmartDashboard.putNumber("kF_Drive",kF_drive);
    SmartDashboard.putNumber("kS_Drive",kS_drive);
    SmartDashboard.putNumber("kV_Drive",kV_drive);
    SmartDashboard.putNumber("kA_Drive",kA_drive);


    SmartDashboard.putNumber("kD_DriveAuto",kD_driveAuto);
    SmartDashboard.putNumber("kP_DriveAuto",kP_driveAuto);
    SmartDashboard.putNumber("kF_DriveAuto",kF_driveAuto);
    SmartDashboard.putNumber("kS_DriveAuto",kS_driveAuto);
    SmartDashboard.putNumber("kV_DriveAuto",kV_driveAuto);
    SmartDashboard.putNumber("kA_DriveAuto",kA_driveAuto);

    

    SmartDashboard.putNumber("kP_Turn",kP_turn);
    SmartDashboard.putNumber("kD_Turn",kD_turn);
    SmartDashboard.putNumber("kF_Turn",kF_turn);
    SmartDashboard.putNumber("MaxOutput Turn",kMaxOutputTurn);

}


// get the module state for odometry calculations: velocity in mps and module angle in radians
// *** need to clean up the conversion - go directly from native units to mps 
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            velNativeToRPM_talon(m_drive.getSelectedSensorVelocity())/MPSToRPM, 
            new Rotation2d(getTurnPosition_Rad()));
    }



// Set the drive motor, takes speed in MPS, converts to native units - encoder counts per 100ms
// Set the turn motor, takes angle in radian, converts to native unit - encoder counts 
    public void setMotors(double speed,double turnAngle) {
        double acc = (speed-speedPrev)/0.020;
        speedPrev=speed;

        m_drive.set(ControlMode.Velocity,  speed*MPSToNativeSpeed,
            DemandType.ArbitraryFeedForward, feedforward_drive.calculate(speed,acc));
    m_turn.set(ControlMode.Position,turnAngle*RadiansToNativePos);
    }


// set all motors to zero 
public void setMotorsAllStop() {
    m_drive.set(ControlMode.PercentOutput,0);
    m_turn.set(ControlMode.PercentOutput,0);
}


  public void resetEncoders() {
      m_drive.setSelectedSensorPosition(0);
    }

// get the turn encoder position, measured in rotations
    public double getTurnPosition_Rot() {
        return ( e_turn.getPosition()/360);
    }

// get the turn encoder position, measured in rotations
    public double getTurnPosition_Deg() {
        return (e_turn.getPosition());
   }       

   // get the turn encoder position, measured in radians
    public double getTurnPosition_Rad() { 
        return (e_turn.getPosition()*piOver180 );
   }       
   
       // get the turn encoder absolute position, measured in degrees
     public double getTurnAbsPosition() {
        return e_turn.getAbsolutePosition();
       }

// get the turn velocity velocity, measured in rpm
    public double getTurnVelocity() {
        return (m_turn.getSelectedSensorVelocity()*600/4096);
   }       

// get the turn output, measured in rotations
    public double getTurnMotorCLT() {
        return m_turn.getClosedLoopTarget();
   }       

   // get the turn output, measured in rotations
    public double getTurnMotorCLE() {
        return m_turn.getClosedLoopError();
   }       


// get the drive encoder position, measured in rotations
     public double getDrivePosition() {
        return (m_drive.getSelectedSensorPosition()/2048.);
       }

// get the drive encoder velocity, measured in rpm
     public double getDriveVelocity() {
        return  velNativeToRPM_talon(m_drive.getSelectedSensorVelocity());
       }

// get voltage
    public double getVoltage() {
        return  m_drive.getMotorOutputVoltage();
    }
    
// get the drive encoder velocity, measured in rpm
    public double getDriveErrorRPM() {
        return  velNativeToRPM_talon(m_drive.getClosedLoopError());
   }

  


    private double velNativeToRPM_talon(double vel_NativeUnits){
        return  vel_NativeUnits*600/2048;
    }

    public double MPStoRPM(double mps){
        return mps*MPSToRPM;
    }

    public void updateConstants(){
        kP_drive= SmartDashboard.getNumber("kP_Drive",kP_drive);
        kF_drive= SmartDashboard.getNumber("kF_Drive",kF_drive);
        kD_drive= SmartDashboard.getNumber("kD_Drive",kD_drive);
        kS_drive= SmartDashboard.getNumber("kS_Drive",kS_drive);
        kV_drive= SmartDashboard.getNumber("kV_Drive",kV_drive);
        kA_drive= SmartDashboard.getNumber("kA_Drive",kA_drive);

        kP_driveAuto= SmartDashboard.getNumber("kP_DriveAuto",kP_driveAuto);
        kF_driveAuto= SmartDashboard.getNumber("kF_DriveAuto",kF_driveAuto);
        kD_driveAuto= SmartDashboard.getNumber("kD_DriveAuto",kD_driveAuto);
        kS_driveAuto= SmartDashboard.getNumber("kS_DriveAuto",kS_driveAuto);
        kV_driveAuto= SmartDashboard.getNumber("kV_DriveAuto",kV_driveAuto);
        kA_driveAuto= SmartDashboard.getNumber("kA_DriveAuto",kA_driveAuto);

        kP_turn= SmartDashboard.getNumber("kP_Turn",kP_turn);
        kD_turn= SmartDashboard.getNumber("kD_Turn",kD_turn);
        kMaxOutputTurn=SmartDashboard.getNumber("MaxOutput Turn",kMaxOutputTurn);

        m_drive.config_kF(0,kF_drive);
        m_drive.config_kP(0,kP_drive);
        m_drive.config_kD(0,kD_drive);
        m_drive.config_kF(1,kF_driveAuto);
        m_drive.config_kP(1,kP_driveAuto);
        m_drive.config_kD(1,kD_driveAuto);

        m_turn.config_kP(0,kP_turn);
        m_turn.config_kD(0,kD_turn);

        feedforward_drive_T=new SimpleMotorFeedforward(kS_drive, kV_drive, kA_drive);
        feedforward_drive_A=new SimpleMotorFeedforward(kS_driveAuto, kV_driveAuto, kA_driveAuto);
        feedforward_drive=feedforward_drive_T;                
    }

    public void setPIDslot(int slot){
        if(slot==0) {
            feedforward_drive=feedforward_drive_T;             
            m_drive.selectProfileSlot(slot, 0);
        }
        else {
            feedforward_drive=feedforward_drive_A;               
            m_drive.selectProfileSlot(slot, 1); 
        }
    }
}