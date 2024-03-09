package frc.robot.subsystems.ShooterIntake;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterIntakeConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/* The Note intake rollers */
public class Intake extends SubsystemBase
{
    private Spark intakeMotor;
    private DigitalInput photoEye;

    public Intake()
    {
        intakeMotor = new Spark(ShooterIntakeConstants.Intake.INTAKE_MOTOR_ID);
        //photoEye = new DigitalInput(ShooterIntakeConstants.Intake.PHOTOEYE_DIO_ID);
    }

    // Take a Note in
    public void intake(double speed)
    {
        intakeMotor.set(speed *.45);

    }

    // Stop the rollers when called
    public void stop()
    {
        intakeMotor.set(0);
    }

    // Use the photoeye to see if a note is detected
    public boolean hasNote()
    {
        // Can invert this with ! if wiring is backwards
        //return photoEye.get();
        return false;
    }

    // Periodically check the status of the intake to see
    // if a note is detected and print the status to the dashboard.
    public void periodic()
    {
        //SmartDashboard.putBoolean("HAS NOTE", hasNote());
    }
}
