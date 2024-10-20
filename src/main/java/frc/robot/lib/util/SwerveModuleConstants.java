package frc.robot.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
    public final int driveMotorID;
    public final int angleMotorID;
    public final int cancoderID;
    public final Rotation2d angleOffset;
    public boolean invertDrive;
    public boolean invertEncoder;
    public boolean invertSteer;

    /**
     * Swerve Module Constants to be used when creating swerve modules.
     * @param driveMotorID
     * @param angleMotorID
     * @param canCoderID
     * @param angleOffset
     * @param invertDrive
     * @param invertEncoder
     * @param invertSteer
     */
    public SwerveModuleConstants(int driveMotorID, int angleMotorID, int canCoderID, Rotation2d angleOffset, boolean invertEncoder, boolean invertDrive, boolean invertSteer) {
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.cancoderID = canCoderID;
        this.angleOffset = angleOffset;
        this.invertDrive = invertDrive;
        this.invertEncoder = invertEncoder;
        this.invertSteer = invertSteer;
    }
}
