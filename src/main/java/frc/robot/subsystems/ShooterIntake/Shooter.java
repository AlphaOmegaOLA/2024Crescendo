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
        leftShooterMotor = new CANSparkMax(ShooterIntakeConstants.Shooter.leftShooterMotorID, MotorType.kBrushless);
        rightShooterMotor = new CANSparkMax(ShooterIntakeConstants.Shooter.rightShooterMotorID, MotorType.kBrushless);
    }

    public void shootAmp()
    {
        leftShooterMotor.set(ShooterIntakeConstants.Shooter.halfSpeed);
        rightShooterMotor.set(ShooterIntakeConstants.Shooter.halfSpeed);
    }

    public void shootSpeaker()
    {
        leftShooterMotor.set(ShooterIntakeConstants.Shooter.fullSpeed);
        rightShooterMotor.set(ShooterIntakeConstants.Shooter.fullSpeed);
    }
}
