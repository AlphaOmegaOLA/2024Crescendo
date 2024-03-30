package frc.robot.subsystems.ShooterIntake;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterIntakeConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/* The Note intake rollers */
public class Intake extends SubsystemBase
{
    private Spark intakeMotor;
    //private DigitalInput photoEye;
    private AnalogInput photoEye;

    public Intake()
    {
        intakeMotor = new Spark(ShooterIntakeConstants.Intake.INTAKE_MOTOR_ID);
        //photoEye = new DigitalInput(ShooterIntakeConstants.Intake.PHOTOEYE_DIO_ID);
        photoEye = new AnalogInput(ShooterIntakeConstants.Intake.PHOTOEYE_DIO_ID);
    }

    // Take a Note in
    public void intake(double speed)
    {
        intakeMotor.set(speed * .45);
    }

    // Run intake at reduced speed
    public Command slow()
    {
        return this.startEnd(() -> this.intakeMotor.set(ShooterIntakeConstants.Intake.HALF_SPEED), 
            () -> this.intakeMotor.set(0));
    }

    // Run intake at full speed
    public Command fast()
    {
        return this.startEnd(() -> this.intakeMotor.set(ShooterIntakeConstants.Intake.FULL_SPEED), 
            () -> this.intakeMotor.set(0));
    }

    public void intakeStop()
    {
        intakeMotor.set(0);
    } 

    // Use the photoeye to see if a note is detected
    public boolean hasNote()
    {
        // Can invert this with ! if wiring is backwards
        //return !photoEye.get();
        if (photoEye.getValue() > 206)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    // Periodically check the status of the intake to see
    // if a note is detected and print the status to the dashboard.
    public void periodic()
    {
        SmartDashboard.putBoolean("HAS NOTE", hasNote());
        //System.out.println(hasNote());
        //System.out.println(hasNote());
        SmartDashboard.putNumber("PhotoEye Value", photoEye.getValue());
        SmartDashboard.putNumber("PhotoEye Voltage", photoEye.getVoltage());
    }
}
