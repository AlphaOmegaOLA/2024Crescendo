package frc.robot.subsystems.ShooterIntake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.ShooterIntakeConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Arm extends SubsystemBase 
{
    private CANSparkMax leftArmMotor;
    private CANSparkMax rightArmMotor;
    // encoders here

    public Arm()
    {
        leftArmMotor = new CANSparkMax(ShooterIntakeConstants.Arm.leftArmMotorID, MotorType.kBrushless);
        rightArmMotor = new CANSparkMax(ShooterIntakeConstants.Arm.rightArmMotorID, MotorType.kBrushless);
        // encoders here
    }

    public void armFloor()
    {
        //encoders
        //PID
        //motors
    }
}
