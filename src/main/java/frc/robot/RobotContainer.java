package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    // private boolean robotCentric = false;

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private limeLight LimeLight = new limeLight();


    public boolean trackingObject = false;

    private void toggleTracking() {
        trackingObject = !trackingObject;
    };
    
    private void hastarget() {
        if (LimeLight.hasTarget()) {
            s_Swerve.drive(new Translation2d(5, 0), s_Swerve.calcultaeTrackingVelocity(3), false, true, true);
        } else {
            s_Swerve.drive(new Translation2d(0,0), 1.5, true, true, false);
        }
    }


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean(),
                this
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
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

        new JoystickButton(driver, Button.kA.value).onTrue(
            new InstantCommand(
                () -> toggleTracking()
            )
        );
        new JoystickButton(driver, Button.kB.value).onTrue(
            new InstantCommand(
                () -> hastarget()
            )
        );
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     */
    public void getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        
    }

    public Swerve getSwerve() {
      return s_Swerve;
    }
}