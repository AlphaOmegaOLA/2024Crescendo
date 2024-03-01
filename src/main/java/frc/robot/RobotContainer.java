package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.*;
import frc.robot.subsystems.Articulation.PoseEstimator;
import frc.robot.subsystems.ShooterIntake.Arm;
import frc.robot.subsystems.ShooterIntake.Intake;
import frc.robot.subsystems.ShooterIntake.Shooter;
import frc.robot.subsystems.swerve.rev.RevSwerve;
import frc.robot.constants.ControllerMap;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer 
{
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = 1;
    private final int strafeAxis = 0;
    private final int rotationAxis = 4;
    //private final int speedDial = 5;

    
    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, ControllerMap.LOGO_RIGHT);
    private final JoystickButton dampen = new JoystickButton(driver, ControllerMap.RB);
    private final POVButton up = new POVButton(driver, 90);
    private final POVButton down = new POVButton(driver, 270);
    private final POVButton right = new POVButton(driver, 180);
    private final POVButton left = new POVButton(driver, 0);

    /* Operator Buttons */
    private final JoystickButton arm_source = new JoystickButton(operator, ControllerMap.B);
    private final JoystickButton arm_floor = new JoystickButton(operator, ControllerMap.X);
    private final JoystickButton arm_speaker = new JoystickButton(operator, ControllerMap.Y);
    private final JoystickButton arm_amp = new JoystickButton(operator, ControllerMap.A);
    private final JoystickButton arm_climb = new JoystickButton(operator, ControllerMap.RB);
    private final JoystickButton intake = new JoystickButton(operator, ControllerMap.LEFT_TRIGGER);
    private final JoystickButton shootSpeaker = new JoystickButton(operator, ControllerMap.RIGHT_TRIGGER);
    private final JoystickButton shootAmp = new JoystickButton(operator, ControllerMap.LB);

    /* Subsystems */
    private final RevSwerve s_Swerve = new RevSwerve();
    private final Arm s_Arm = new Arm();
    private final Intake s_Intake = new Intake();
    private final Shooter s_Shooter = new Shooter();
    private final PoseEstimator s_PoseEstimator = new PoseEstimator();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() 
    {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> false,
                () -> dampen.getAsBoolean(),
                () -> 1 //speed multiplier 
            )
        );

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() 
    {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

        //heading lock bindings
        up.onTrue(
            new InstantCommand(() -> States.driveState = States.DriveStates.d90)).onFalse(
            new InstantCommand(() -> States.driveState = States.DriveStates.standard)
            );
        left.onTrue(
            new InstantCommand(() -> States.driveState = States.DriveStates.d180)).onFalse(
            new InstantCommand(() -> States.driveState = States.DriveStates.standard)
            );
        right.onTrue(
            new InstantCommand(() -> States.driveState = States.DriveStates.d0)).onFalse(
            new InstantCommand(() -> States.driveState = States.DriveStates.standard)
            );
        down.onTrue(
            new InstantCommand(() -> States.driveState = States.DriveStates.d270)).onFalse(
            new InstantCommand(() -> States.driveState = States.DriveStates.standard)
            );

        arm_source.onTrue(new InstantCommand(() -> s_Arm.intakeSource(), s_Arm));
        arm_floor.onTrue(new InstantCommand(() -> s_Arm.intakeFloor(), s_Arm));
        arm_speaker.onTrue(new InstantCommand(() -> s_Arm.shootSpeaker(), s_Arm));
        arm_amp.onTrue(new InstantCommand(() -> s_Arm.shootAmp(), s_Arm));
        arm_climb.onTrue(new InstantCommand(() -> s_Arm.climb(), s_Arm));

        intake.onTrue(
            new InstantCommand(() -> s_Intake.intake(), s_Intake)).onFalse(
            new InstantCommand(() -> s_Intake.stop(), s_Intake)
            );

        shootSpeaker.onTrue(
                new SequentialCommandGroup(
                    new InstantCommand(() -> s_Shooter.shootSpeaker(), s_Shooter),
                    new WaitCommand(1),
                    new InstantCommand(() -> s_Intake.intake(), s_Intake)
                )).onFalse(
                    new SequentialCommandGroup(
                        new InstantCommand(() -> s_Shooter.stop(), s_Shooter),
                        new InstantCommand(() -> s_Intake.stop(), s_Intake)
                    )
                );

        shootAmp.onTrue(
                new SequentialCommandGroup(
                    new InstantCommand(() -> s_Shooter.shootAmp(), s_Shooter),
                    new WaitCommand(1),
                    new InstantCommand(() -> s_Intake.intake(), s_Intake)
                )).onFalse(
                    new SequentialCommandGroup(
                        new InstantCommand(() -> s_Shooter.stop(), s_Shooter),
                        new InstantCommand(() -> s_Intake.stop(), s_Intake)
                    )
                );
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() 
    {
        return null;
    }
}
