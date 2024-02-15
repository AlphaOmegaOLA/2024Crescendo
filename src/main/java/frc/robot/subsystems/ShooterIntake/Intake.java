package frc.robot.subsystems.ShooterIntake;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.ShooterIntakeConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Intake extends SubsystemBase
{
    private CANSparkMax intakeMotor;

    public Intake()
    {
        intakeMotor = new CANSparkMax(ShooterIntakeConstants.Intake.intakeMotorID, MotorType.kBrushless);
    }

    public void intakeFloor()
    {
        intakeMotor.set(ShooterIntakeConstants.Intake.fullSpeed);
    }

    public void intakeSourceZone()
    {
        intakeMotor.set(ShooterIntakeConstants.Intake.fullSpeed);
    }
}
