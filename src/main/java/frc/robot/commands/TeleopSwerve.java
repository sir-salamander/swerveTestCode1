package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {    
    private Swerve s_Swerve;    
    private double translationSup;
    private double strafeSup;
    private double rotationSup;
    private BooleanSupplier robotCentricSup;
    public TeleopSwerve(Swerve s_Swerve, XboxController controller, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        controller = new XboxController(0);
        this.translationSup = controller.getLeftY();
        this.strafeSup = controller.getLeftX();
        this.rotationSup = controller.getRightX();
        this.robotCentricSup = robotCentricSup;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup, Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup, Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup, Constants.stickDeadband);

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
}