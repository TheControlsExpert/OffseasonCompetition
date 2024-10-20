// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }


  public static class ShooterConstants {

    public static final int T_ID = 0;
    public static final int B_ID = 0;
    public static final double kP = 0;
    public static final double kD = 0;
    public static final boolean T_Inverted = false;
    public static final boolean B_Inverted = false;
    public static final double IDLE_POSITION = 0;
    public static final double AMP_POSITION = 0;
    public static final double PODIUM_POSITION = 0;
    public static final double SUBWOOFER_POSITION = 0;
    public static final double AMP_POSITIONL = 0;
    public static final double AMP_POSITIONB = 0;
    public static final double AMP_POSITIONT = 0;
    public static final double PASSING_POSITION = 0;
    public static int kS;
    public static double kV;
    public static double kA;

  }


  public static class SwerveConstants {

    public static final double DriveMotorSupplyCurrent = 0;
    public static final double DriveMotorStatorCurrent = 0;
    public static final double SteerMotorSupplyCurrent = 0;
    public static final double SteerMotorStatorCurrent = 0;
    public static final double WheelDiameterMeters = 0;
    public static final double Ratio_MotortoWheel = 0;
    private static final double wheelBase = 0;
    private static final double trackWidth = 0;
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
             new Translation2d(wheelBase / 2.0, trackWidth / 2.0), 
             new Translation2d(wheelBase / 2.0, -trackWidth / 2.0), 
             new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0),
             new Translation2d(-wheelBase / 2.0, trackWidth / 2.0));
    public static final Measure<Velocity<Distance>> MaxSpeedMetersPerSecond_Modules = null;
    
  }

  public static class VisionConstants {

    public static final double Height_LL3 = 0;
    public static final double Angle_LL3 = 0;
    public static final int MAX_ANGULAR_VEL = 0;

  }
    
    
    
    public static class IntakeConstants {

    public static final int LEFTINTAKE_ID = 0;
    public static final int RIGHTINTAKE_ID = 0;
    public static final int HANDOFF_ID = 0;
    public static final boolean LEFT_INVERSION = false;
    public static final boolean RIGHT_INVERSION = false;
    public static final boolean HANDOFF_INVERSION = false;
    public static final int frontIR_ID = 0;
    public static final int firstIR_ID = 0;
    public static final int middleIR_ID = 0;
    public static final int lastIR_ID = 0;
    public static final double IDLE_POSITION = 0;
    public static final double AMP_POSITION = 0;
    public static final double PODIUM_POSITION = 0;
    public static final double SUBWOOFER_POSITION = 0;
    public static final double PASSING_POSITION = 0;

  }


  public static class PivotConstants {

    public static final int PivotID = 0;
    public static final double kP = 0;
    public static final double kD = 0;
    public static final boolean PivotInversion = false;
    public static final double AMP_ANGLE_ROTATIONS = 0;
    public static final double SUBWOOFER_ANGLE_ROTATIONS = 0;
    public static final double AMP = 0;
    public static final double HOME_POSITION = 8;
    public static final double AMP_POSITION = 0;
    public static final double PODIUM_POSITION = 0;
    public static final double SUBWOOFER_POSITION = 0;
    public static final double TRAVELLING_POSITION = 12;
    public static final int MAX_EXTENSION = 0;
    public static final double MIN_EXTENSION = 0;
    public static final double kTolerance = 0;
    public static final double PASSING_POSITION = 0;
    
  }


  public static class LocalizationConstants {

    public static final double InitialPoseCovariancex = 0;
    public static final double InitialPoseCovariancey = 0;
    public static final double BufferLifespan = 0;
    public static final double DistanceMultiplierOdometry = 0;
    public static final double VisionMultiplier = 0;
    public static final double VisionMultiplierX = 0;
    public static final double VisionMultiplierY = 0;
    
  }


public static final int ResidualMultiplierOdometry = 0;

}