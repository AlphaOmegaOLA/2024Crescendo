package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
import frc.robot.commands.*;

/* PathPlanner */
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer 
{
    /* Autonomous menu */
    //private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    private final SendableChooser<Command> autoChooser;

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

    // B = Source Angle
    private final JoystickButton arm_source = new JoystickButton(operator, XboxController.Button.kB.value);
    // X = Floor Angle
    private final JoystickButton arm_floor = new JoystickButton(operator, XboxController.Button.kX.value);
    // Y = Speaker Angle
    private final JoystickButton arm_speaker = new JoystickButton(operator, XboxController.Button.kY.value);
    // A = Amp Angle
    private final JoystickButton arm_amp = new JoystickButton(operator, XboxController.Button.kA.value);
    // Right Bumper = Shoot Amp (Shooter and Intake simultaneously at half speed)
    private final JoystickButton shootAmp = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    // Left Bumper = Shoot Speaker (Shooter starts then Intake feeds 1 second later both at full speed)
    private final JoystickButton shootSpeaker = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
  
    /* Subsystems */
    private final RevSwerve s_Swerve = new RevSwerve();
    private final PIDArm s_Arm = new PIDArm();
    private final Intake s_Intake = new Intake();
    private final Shooter s_Shooter = new Shooter();

    // Microsoft Life cam on arm
    private final UsbCamera usbcamera;

    /* Commands */

    // Arm states
    private final InstantCommand c_floorAngle = new InstantCommand(() -> States.armState = States.ArmStates.Floor);
    private final InstantCommand c_speakerAngle = new InstantCommand(() -> States.armState = States.ArmStates.Speaker);
    private final InstantCommand c_sourceAngle = new InstantCommand(() -> States.armState = States.ArmStates.Source);
    private final InstantCommand c_ampAngle = new InstantCommand(() -> States.armState = States.ArmStates.Amp);

    // Shooter-Intake commands

    // Shooter starts fast and then the amp does 1-second later both at full speed while the Left Bumper is pressed
    private final SequentialCommandGroup c_shootFast = s_Shooter.fast().alongWith(new WaitCommand(1)).andThen(s_Intake.fast());

    // Same as c_shootFast above but it stops the motors after 3 seconds
    private final SequentialCommandGroup c_shootFastAuto = s_Shooter.fast().withTimeout(3).alongWith(
        new WaitCommand(1)).andThen(
            s_Intake.fast().withTimeout(1));

    // Shooter and intake start simultaneoulsy and run half speed while the right button is pressed
    private final ParallelCommandGroup c_shootSlow = s_Shooter.slow().alongWith(s_Intake.slow());  

    // Intake at full speed while the joystick is pressed
    private final Command c_intakeFast = s_Intake.fast();

    // Intake at full speed with a 2-second timeout
    private final Command c_intakeFastAuto = s_Intake.fast().withTimeout(2);
    
    // Move arm to speaker angle, 
    private final SequentialCommandGroup c_shootSpeakerOnlyAuto = c_speakerAngle.andThen(
        c_shootFastAuto).andThen(c_sourceAngle);
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() 
    {
        // PathPlanner registered commands
        NamedCommands.registerCommand("Arm Amp Angle", c_ampAngle);
        NamedCommands.registerCommand("Arm Floor Angle", c_floorAngle);
        NamedCommands.registerCommand("Arm Source Angle", c_sourceAngle);
        NamedCommands.registerCommand("Arm Speaker Angle", c_speakerAngle);
        NamedCommands.registerCommand("Intake Fast", c_intakeFastAuto);
        NamedCommands.registerCommand("Shoot Slow", c_shootFastAuto);
        
        // Pathplanner auto builder from commands in the deploy/pathplanner
        // Default auto will be `Commands.none()
        autoChooser = AutoBuilder.buildAutoChooser(); 
        SmartDashboard.putData("Auto Mode", autoChooser);
        
        // Sets up Swerve with a dampener tied to the right bumper button
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

        // Manual intake with left joystick
        s_Intake.setDefaultCommand(
        Commands.run(() -> s_Intake.intake(operator.getLeftY()), s_Intake)
        );
        
        // Manual intake with right joystick. We only have one on our
        // new operator console but there's a switch to change which
        // joystick it is and we shouldn't need this anymore.
        s_Shooter.setDefaultCommand(
        Commands.run(() -> s_Shooter.manual(operator.getRightY()), s_Shooter)
        );

        // The defaults arm state is whatever angle it's at
        s_Arm.setDefaultCommand(new PIDArmCommand(s_Arm));

        // Camera
        usbcamera = CameraServer.startAutomaticCapture();

        // Configure the button bindings
        configureButtonBindings();

        // Put the autonomous chooser menu on the dashboard
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

        // Zero out the heading when the match starts. Can we automate this?
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

        // Heading Lock Bindings
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

        arm_source.onTrue(c_sourceAngle);
        arm_floor.onTrue(c_floorAngle);
        arm_speaker.onTrue(c_speakerAngle);
        arm_amp.onTrue(c_ampAngle);

        shootSpeaker.whileTrue(c_shootFast);
        shootAmp.whileTrue(c_shootSlow);
    }       
        
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() 
    {
        return autoChooser.getSelected(); 
    }
}
