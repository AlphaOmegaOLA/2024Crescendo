package frc.robot.subsystems.ShooterIntake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.ShooterIntakeConstants;
import frc.robot.subsystems.ShooterIntake.RevThroughBoreEncoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;


public class Arm extends SubsystemBase 
{
    private CANSparkMax leftArmMotor;
    private CANSparkMax rightArmMotor;
    private RevThroughBoreEncoder armEncoder;

    public Arm()
    {
        leftArmMotor = new CANSparkMax(ShooterIntakeConstants.Arm.LEFT_ARM_MOTOR_ID, MotorType.kBrushless);
        leftArmMotor.setIdleMode(IdleMode.kBrake);
        //leftArmMotor.setInverted(true);
        rightArmMotor = new CANSparkMax(ShooterIntakeConstants.Arm.RIGHT_ARM_MOTOR_ID, MotorType.kBrushless);
        rightArmMotor.setIdleMode(IdleMode.kBrake);
        //rightArmMotor.setInverted(true);
        armEncoder = new RevThroughBoreEncoder(ShooterIntakeConstants.Arm.ARM_ENCODER_ID);
        armEncoder.setOffset(ShooterIntakeConstants.Arm.ARM_ENCODER_OFFSET);

        //armEncoder.setInverted(true);
    }

    public void intakeFloor()
    {
        //encoders
        //PID
        //motors
        


    }
}

