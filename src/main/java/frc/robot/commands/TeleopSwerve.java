package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {    
    private Swerve s_Swerve;    
    private int translationSup;
    private int strafeSup;
    private int rotationSup;
    private BooleanSupplier robotCentricSup;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.s_Swerve = RobotContainer.s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = RobotContainer.translationAxis;
        this.strafeSup = RobotContainer.strafeAxis;
        this.rotationSup = RobotContainer.rotationAxis;
        this.robotCentricSup = RobotContainer.robotCentric;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup, Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup, Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup, Constants.stickDeadband);

        /* Drive */
        s_Swerve.drive(
            new Translation2d(RobotContainer.translationAxis, RobotContainer.strafeAxis).times(Constants.Swerve.maxSpeed), 
            RobotContainer.rotationAxis * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
}