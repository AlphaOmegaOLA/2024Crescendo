package frc.robot.subsystems.ShooterIntake;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.ShooterIntakeConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm extends SubsystemBase 
{
    private DigitalInput photoEye = new DigitalInput(ShooterIntakeConstants.Arm.PHOTOEYE_DIO_ID);
    private CANSparkMax leftArmMotor;
    private CANSparkMax rightArmMotor;
    private RevThroughBoreEncoder armEncoder;
    private static double updateOutput;
    private static double maxVelocity;
    private static double maxAcceleration;
    private static double proportional;
    private static double integral;
    private static double differential;
    private static double staticFriction;
    private static double gravityGain;
    private static double velocityGain;
    private final TrapezoidProfile.Constraints constraints;
    private final ProfiledPIDController controller;
    private final ElevatorFeedforward feedForward;

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
        updateOutput = ShooterIntakeConstants.Arm.ARM_UPDATE_OUTPUT;
        maxVelocity = ShooterIntakeConstants.Arm.ARM_MAX_VELOCITY;
        maxAcceleration = ShooterIntakeConstants.Arm.ARM_MAX_ACCELERATION;
        proportional = ShooterIntakeConstants.Arm.ARM_P;
        integral = ShooterIntakeConstants.Arm.ARM_I;
        differential = ShooterIntakeConstants.Arm.ARM_D;
        staticFriction = ShooterIntakeConstants.Arm.ARM_S;
        gravityGain = ShooterIntakeConstants.Arm.ARM_G;
        velocityGain = ShooterIntakeConstants.Arm.ARM_V;
        constraints =  new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);
        controller = new ProfiledPIDController(proportional, integral, differential, constraints, updateOutput);
        feedForward = new ElevatorFeedforward(staticFriction, gravityGain, velocityGain);
    }

    private void setAngle(double angle)
    {
        controller.setGoal(angle); // double Degrees

        if (!tooFar())
        {
            leftArmMotor.setVoltage(
                controller.calculate(armEncoder.getAngle().getDegrees())
                    + feedForward.calculate(controller.getSetpoint().velocity));

            rightArmMotor.setVoltage(
                controller.calculate(armEncoder.getAngle().getDegrees())
                    + feedForward.calculate(controller.getSetpoint().velocity));
        }
        else
        {
            leftArmMotor.setVoltage(0);
            rightArmMotor.setVoltage(0);
        }
    }

    public void intakeFloor()
    {
        setAngle(ShooterIntakeConstants.Arm.ARM_FLOOR_ANGLE);
    }

    public void intakeSource()
    {
        setAngle(ShooterIntakeConstants.Arm.ARM_SOURCE_ANGLE);
    }

    public void shootAmp()
    {
        setAngle(ShooterIntakeConstants.Arm.ARM_AMP_ANGLE);
    }

    public void shootSpeaker()
    {
        setAngle(ShooterIntakeConstants.Arm.ARM_SPEAKER_ANGLE);
    }

    public void climb()
    {
        setAngle(ShooterIntakeConstants.Arm.ARM_CLIMB_ANGLE);
    }

    public boolean tooFar()
    {
        // Can invert this with ! if wiring is backwards
        return photoEye.get();
    }

    public void periodic()
    {
        SmartDashboard.putNumber("ARM ANGLE", armEncoder.getAngle().getDegrees());
        SmartDashboard.putNumber("ARM OFFSET", armEncoder.getOffset());
    }
}

