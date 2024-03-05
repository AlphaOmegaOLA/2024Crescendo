package frc.robot.subsystems.ShooterIntake;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterIntakeConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake extends SubsystemBase
{
    private CANSparkMax intakeMotor;
    private DigitalInput photoEye;

    public Intake()
    {
        intakeMotor = new CANSparkMax(ShooterIntakeConstants.Intake.INTAKE_MOTOR_ID, MotorType.kBrushless);
        photoEye = new DigitalInput(ShooterIntakeConstants.Intake.PHOTOEYE_DIO_ID);
    }

    public void intake()
    {
        if (!hasNote())
        {
            intakeMotor.set(ShooterIntakeConstants.Intake.FULL_SPEED);
        }
        else
        {
            stop();
        }
    }

    public void stop()
    {
        intakeMotor.set(0);
    }

    public boolean hasNote()
    {
        // Can invert this with ! if wiring is backwards
        return photoEye.get();
    }

    public void periodic()
    {
        SmartDashboard.putBoolean("HAS NOTE", hasNote());
    }
}
