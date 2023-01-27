package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class ConeGrabber extends SubsystemBase{
    private DigitalInput limit = new DigitalInput(1);
    private TalonFX coneMotor;
    private  Solenoid pistonForward;
    private Solenoid pistonBackward;

    public ConeGrabber(TalonFX motor, Solenoid solenoid1, Solenoid solenoid2) {
        coneMotor = motor;
        pistonForward = solenoid1;
        pistonBackward = solenoid2;

        motor.configFactoryDefault();
        motor.setInverted(false);
        motor.setNeutralMode(NeutralMode.Brake);

        pistonForward.set(true);
        pistonBackward.set(false);

    }


    public Command rotateMotorFoward() {
        return new InstantCommand( () -> coneMotor.set(ControlMode.Velocity, 1));
    }

    public Command rotateMotorBackward() {
        return new InstantCommand(() -> coneMotor.set(ControlMode.Velocity, -1));
    }

  /*   Tayab - a squential command group must be a list of commands, what you listed are not commands.

    public Command grabCone() {
        return new SequentialCommandGroup(
            pistonForward.set(pistonBackward.get()),
            pistonBackward.set(!pistonForward.get())
        );
*/
    }



