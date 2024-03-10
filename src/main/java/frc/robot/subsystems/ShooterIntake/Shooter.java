package frc.robot.subsystems.ShooterIntake;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.ShooterIntakeConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Shooter extends SubsystemBase 
{
    private CANSparkMax leftShooterMotor;
    private CANSparkMax rightShooterMotor;

    public Shooter()
    {
        leftShooterMotor = new CANSparkMax(ShooterIntakeConstants.Shooter.LEFT_SHOOTER_MOTOR_ID, MotorType.kBrushless);
        rightShooterMotor = new CANSparkMax(ShooterIntakeConstants.Shooter.RIGHT_SHOOTER_MOTOR_ID, MotorType.kBrushless);
        leftShooterMotor.setInverted(true);
    }

    public void shootAmp()
    {
        leftShooterMotor.set(-ShooterIntakeConstants.Shooter.HALF_SPEED);
        rightShooterMotor.set(-ShooterIntakeConstants.Shooter.HALF_SPEED);
    }

    public void shootSpeaker()
    {
        leftShooterMotor.set(-ShooterIntakeConstants.Shooter.FULL_SPEED);
        rightShooterMotor.set(-ShooterIntakeConstants.Shooter.FULL_SPEED);
    }

    public void shootManual(double speed)
    {
        leftShooterMotor.set(speed);
        rightShooterMotor.set(speed);        
    }

    public void stop()
    {
        leftShooterMotor.set(0);
        rightShooterMotor.set(0);
    }
}
