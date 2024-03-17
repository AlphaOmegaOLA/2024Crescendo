package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import frc.robot.subsystems.ShooterIntake.Intake;
import frc.robot.subsystems.ShooterIntake.PIDArm;
import frc.robot.subsystems.ShooterIntake.Shooter;
import frc.robot.subsystems.swerve.rev.RevSwerve;
import frc.robot.constants.ControllerMap;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.commands.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer 
{
    /* Autonomous menu */
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    /* Controllers */
    private final XboxController driver = new XboxController(0);
    private final XboxController operator = new XboxController(1);

    /* Drive Controls */
    private final int translationAxis = 1;
    private final int strafeAxis = 0;
    private final int rotationAxis = 4;
    
    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, ControllerMap.LOGO_RIGHT);
    private final JoystickButton dampen = new JoystickButton(driver, ControllerMap.RB);
    private final POVButton up = new POVButton(driver, 90);
    private final POVButton down = new POVButton(driver, 270);
    private final POVButton right = new POVButton(driver, 180);
    private final POVButton left = new POVButton(driver, 0);

    /* Operator Buttons */
    private final JoystickButton arm_source = new JoystickButton(operator, XboxController.Button.kB.value);
    private final JoystickButton arm_floor = new JoystickButton(operator, XboxController.Button.kX.value);
    private final JoystickButton arm_speaker = new JoystickButton(operator, XboxController.Button.kY.value);
    private final JoystickButton arm_amp = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton shootAmp = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    private final JoystickButton shootSpeaker = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
  
    /* Subsystems */
    private final RevSwerve s_Swerve = new RevSwerve();
    //private final Arm s_Arm = new Arm();
    private final PIDArm s_Arm = new PIDArm();
    private final Intake s_Intake = new Intake();
    private final Shooter s_Shooter = new Shooter();

    private final UsbCamera usbcamera;



    /* Autonomous Commands */

    // Distance in meters: Feet to meters is feet * 0.3048;
    // Duration in seconds
    // speed is meters per second for the drive command: duration / distance

    private final SequentialCommandGroup leftSide = 
        new SequentialCommandGroup(
                   new InstantCommand(() -> States.armState = States.ArmStates.Speaker),
                    new WaitCommand(2),
                    new InstantCommand(() -> s_Shooter.shootSpeaker(), s_Shooter),
                    new WaitCommand(1),
                    new InstantCommand(() -> s_Intake.shootSpeaker(), s_Intake),
                    new WaitCommand(1),
                    new InstantCommand(() -> s_Shooter.stop()),
                    new InstantCommand(() -> s_Intake.stop()),
                   new InstantCommand(() -> States.armState = States.ArmStates.Source),
                    new WaitCommand(1),                   
                    new InstantCommand(() -> s_Swerve.drive(new Translation2d(-2.3, 0), 0, true, false), s_Swerve),
                    new WaitCommand(8)
                    );

    private final SequentialCommandGroup straightBack = 
        new SequentialCommandGroup(
                   new InstantCommand(() -> States.armState = States.ArmStates.Speaker),
                    new WaitCommand(2),
                    new InstantCommand(() -> s_Shooter.shootSpeaker(), s_Shooter),
                    new WaitCommand(1),
                    new InstantCommand(() -> s_Intake.shootSpeaker(), s_Intake),
                    new WaitCommand(1),
                    new InstantCommand(() -> s_Shooter.stop()),
                    new InstantCommand(() -> s_Intake.stop()),
                    new InstantCommand(() -> States.armState = States.ArmStates.Source),
                    new WaitCommand(1),                   
                    new InstantCommand(() -> s_Swerve.drive(new Translation2d(-2.0, 0), 0, true, false), s_Swerve),
                    new WaitCommand(4)
                    );

    public final SequentialCommandGroup rightSide = 
        new SequentialCommandGroup(
                     new InstantCommand(() -> States.armState = States.ArmStates.Speaker),
                    new WaitCommand(2),
                    new InstantCommand(() -> s_Shooter.shootSpeaker(), s_Shooter),
                    new WaitCommand(1),
                    new InstantCommand(() -> s_Intake.shootSpeaker(), s_Intake),
                    new WaitCommand(1),
                    new InstantCommand(() -> s_Shooter.stop()),
                    new InstantCommand(() -> s_Intake.stop()),
                    new InstantCommand(() -> States.armState = States.ArmStates.Source),
                    new WaitCommand(1),
                    new InstantCommand(() -> s_Swerve.drive(new Translation2d(-2.0, 0), 0, true, false), s_Swerve),
                    new WaitCommand(2),
                    new InstantCommand(() -> s_Swerve.drive(new Translation2d(1, 1), s_Swerve.getPose().getRotation().getDegrees() * -1, true, false), s_Swerve),
                    new WaitCommand(3),
                    new InstantCommand(() -> s_Swerve.drive(new Translation2d(-2.0, 0), 0, true, false), s_Swerve),
                    new WaitCommand(5)
                    );

    public final SequentialCommandGroup shootStop = 
        new SequentialCommandGroup(
                     new InstantCommand(() -> States.armState = States.ArmStates.Speaker),
                    new WaitCommand(2),
                    new InstantCommand(() -> s_Shooter.shootSpeaker(), s_Shooter),
                    new WaitCommand(1),
                    new InstantCommand(() -> s_Intake.shootSpeaker(), s_Intake),
                    new WaitCommand(1),
                    new InstantCommand(() -> s_Shooter.stop()),
                    new InstantCommand(() -> s_Intake.stop()),
                    new InstantCommand(() -> States.armState = States.ArmStates.Source)
                    );

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() 
    {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> driver.getRawAxis(translationAxis), 
                () -> driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> false,
                () -> dampen.getAsBoolean(),
                () -> 1 //speed multiplier 
            )
        );

        s_Intake.setDefaultCommand(
        Commands.run(
            () ->
                s_Intake.intake(operator.getLeftY()), s_Intake
            )
        );
        
        s_Shooter.setDefaultCommand(
        Commands.run(
            () ->
                s_Shooter.shootManual(operator.getRightY()), s_Shooter
            )
        );

        s_Arm.setDefaultCommand(
            new PIDArmCommand(
                s_Arm        
            )
        );

        // Camera
        usbcamera = CameraServer.startAutomaticCapture();

        // Configure the button bindings
        configureButtonBindings();

        autoChooser.setDefaultOption("Swervy D on the LEFT", leftSide);
        autoChooser.addOption("Swervy D in the CENTER", straightBack);
        autoChooser.addOption("Swervy D on the RIGHT", rightSide);
        autoChooser.addOption("Swervy D SHOOTS ONLY", shootStop);
        SmartDashboard.putData("Auto Mode", autoChooser);
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

        arm_source.onTrue(new InstantCommand(() -> States.armState = States.ArmStates.Source));
        arm_floor.onTrue(new InstantCommand(() -> States.armState = States.ArmStates.Floor));
        arm_speaker.onTrue(new InstantCommand(() -> States.armState = States.ArmStates.Speaker));
        arm_amp.onTrue(new InstantCommand(() -> States.armState = States.ArmStates.Amp));

        shootSpeaker.onTrue(
                new SequentialCommandGroup(
                    new InstantCommand(() -> s_Shooter.shootSpeaker(), s_Shooter),
                    new WaitCommand(1),
                    new InstantCommand(() -> s_Intake.shootSpeaker(), s_Intake),
                    new WaitCommand(3)
                )).onFalse(
                    new SequentialCommandGroup(
                        new InstantCommand(() -> s_Shooter.stop(), s_Shooter),
                        new InstantCommand(() -> s_Intake.stop(), s_Intake)
                    )
                );

        shootAmp.whileTrue(
                new SequentialCommandGroup(
                    new InstantCommand(() -> s_Shooter.shootAmp(), s_Shooter),
                    new InstantCommand(() -> s_Intake.shootAmp(), s_Intake),
                    new WaitCommand(3)
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

        //return rightSide;
        return autoChooser.getSelected();
        
    }
}
