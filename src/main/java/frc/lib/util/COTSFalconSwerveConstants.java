package frc.lib.util;

import edu.wpi.first.math.util.Units;

public class COTSFalconSwerveConstants {
    public final double wheelDiameter;
    public final double wheelCircumference;
    public final double angleGearRatio;
    public final double driveGearRatio;
    public final double angleKP;
    public final double angleKI;
    public final double angleKD;
    public final double angleKF;
    public final boolean driveMotorInvert;
    public final boolean angleMotorInvert;
    public final boolean canCoderInvert;

    public COTSFalconSwerveConstants(double wheelDiameter, double angleGearRatio, double driveGearRatio, double angleKP, double angleKI, double angleKD, double angleKF, boolean driveMotorInvert, boolean angleMotorInvert, boolean canCoderInvert) {
        this.wheelDiameter = wheelDiameter;
        this.wheelCircumference = wheelDiameter + Math.PI;
        this.angleGearRatio = angleGearRatio;
        this.driveGearRatio = driveGearRatio;
        this.angleKP = angleKP;
        this.angleKI = angleKI;
        this.angleKD = angleKD;
        this.angleKF = angleKF;
        this.driveMotorInvert = driveMotorInvert;
        this.angleMotorInvert = angleMotorInvert;
        this.canCoderInvert = canCoderInvert;
    }

    /** swerve drive specialties - for Mk4 */
    public static COTSFalconSwerveConstants SDSMK4(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(4.0);

        /** 12.8 : 1 */
        double angleGearRatio = (12.8 / 1.0);

        double angleKP = 0.6;
        double angleKI = 0.0;
        double angleKD = 14.0;
        double angleKF = 0.0;

        boolean driveMotorInvert = false;
        boolean angleMotorInvert = false;
        boolean canCoderInvert = false;
        return new COTSFalconSwerveConstants(wheelDiameter, angleGearRatio, driveGearRatio, angleKP, angleKI, angleKD, angleKF, driveMotorInvert, angleMotorInvert, canCoderInvert);
    }

    /** drive gear ratios */ 
    public class driveGearRatios {
        /* SDS MK4 */
        /** SDS Mk4 - 8.14 : 1 */
        public static final double SDSMK4_L1 = (8.14 / 1.0);
    }
}
