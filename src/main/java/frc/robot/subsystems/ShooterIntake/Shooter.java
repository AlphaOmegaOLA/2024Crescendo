package frc.robot.subsystems.ShooterIntake;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.ShooterIntakeConstants;

import javax.sound.midi.Sequence;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;

public class Shooter extends SubsystemBase 
{
    private CANSparkMax leftShooterMotor;
    private CANSparkMax rightShooterMotor;

    public Shooter()
    {
        leftShooterMotor = new CANSparkMax(ShooterIntakeConstants.Shooter.LEFT_SHOOTER_MOTOR_ID, MotorType.kBrushless);
        leftShooterMotor.setOpenLoopRampRate(.5);
        leftShooterMotor.setInverted(true);
        rightShooterMotor = new CANSparkMax(ShooterIntakeConstants.Shooter.RIGHT_SHOOTER_MOTOR_ID, MotorType.kBrushless);
        rightShooterMotor.setOpenLoopRampRate(.5);
    }

<<<<<<< HEAD
    private void setMotors(double speed)
=======
    public void shootSlow()
>>>>>>> f8f0039ed2cb983b944429ee146bdb0298fd665d
    {
        leftShooterMotor.set(speed);
        rightShooterMotor.set(speed);
    }

<<<<<<< HEAD
    public Command slow()
=======
    public void shootFast()
>>>>>>> f8f0039ed2cb983b944429ee146bdb0298fd665d
    {
        return this.startEnd(() -> this.setMotors(-ShooterIntakeConstants.Shooter.HALF_SPEED),
            () -> this.setMotors(0));
    }

    public Command fast()
    {
        return this.startEnd(() -> this.setMotors(-ShooterIntakeConstants.Shooter.FULL_SPEED),
            () -> this.setMotors(0));
    }

    public void manual(double speed)
    {
        this.setMotors(speed);       
    }

    public void shooterStop()
    {
        this.setMotors(0);
    }
}
