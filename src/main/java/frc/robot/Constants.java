// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import frc.robot.lib.util.COTSTalonFXSwerveConstants;
import frc.robot.lib.util.SwerveModuleConstants;

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


    
    public static final boolean T_Inverted = false;
    public static final boolean B_Inverted = false;
    public static final double IDLE_POSITION = 0;
    
    public static final double PODIUM_POSITION = 0;
    public static final double SUBWOOFER_POSITION = 4000;
    public static final double AMP_POSITIONL = 0;
    public static final double AMP_POSITIONB = 0;
    public static final double AMP_POSITIONT = 0;
    public static final double PASSING_POSITION = 0;


  }


  public static class SwerveConstants {

    public static final double DriveMotorSupplyCurrent = 0;
    public static final double DriveMotorStatorCurrent = 0;
    public static final double SteerMotorSupplyCurrent = 0;
    public static final double SteerMotorStatorCurrent = 0;
    public static final double WheelDiameterMeters = 0;
    public static final double Ratio_MotortoWheel = 0;
    public static final double trackWidth = 0.63125; 
    public static final double wheelBase = 0.58125; 
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
             new Translation2d(wheelBase / 2.0, trackWidth / 2.0), 
             new Translation2d(wheelBase / 2.0, -trackWidth / 2.0), 
             new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0),
             new Translation2d(-wheelBase / 2.0, trackWidth / 2.0));
    public static final double MaxSpeedMetersPerSecond_Modules = 5;


    public static final SwerveModuleConfig SwerveConfig0 = new SwerveModuleConfig(
      2,
      1,
      3,
     Rotation2d.fromDegrees(180-139.482),
      false,
      true,
      0,
      5,
      0,
      0,
      0,
      0,
      0
    );

    public static final SwerveModuleConfig SwerveConfig1 = new SwerveModuleConfig(
      5,
      4,
      6,
     Rotation2d.fromDegrees(116.279),
      false,
      false,
      0,
      5,
      0,
      0,
      0,
      0,
      0
    );

    public static final SwerveModuleConfig SwerveConfig2 = new SwerveModuleConfig(
      8,
      7,
      9,
     Rotation2d.fromDegrees(-131.2207+180),
      true,
      true,
      0,
      5,
      0,
      0,
      0,
      0,
      0
    );


    public static final SwerveModuleConfig SwerveConfig3 = new SwerveModuleConfig(
      8,
      7,
      9,
     Rotation2d.fromDegrees(-111.796),
      false,
      true,
      0,
      5,
      0,
      0,
      0,
      0,
      0
    );



    
    public static record SwerveModuleConfig(
       int driveID,
       int turnID,
      int absoluteEncoderID,
      Rotation2d absoluteEncoderOffsetRotations,
      boolean driveMotorInverted,
      boolean steerMotorInverted, 
      double ffkS,
      double ffkV,
      double ffkA,
      double drivekP,
      double drivekD,
      double turnkP,
      double turnkD
      ) {}

    
  }



  public static final class Swerve {
        public static final COTSTalonFXSwerveConstants chosenModule = 
        COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L1);

        /* Drivetrain Constants */
        public static final double trackWidth = 0.63125; 
        public static final double wheelBase = 0.58125; 
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
             new Translation2d(wheelBase / 2.0, trackWidth / 2.0), 
             new Translation2d(wheelBase / 2.0, -trackWidth / 2.0), 
             new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0),
             new Translation2d(-wheelBase / 2.0, trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.10; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.667/12; //TODO: This must be tuned to specific robot
        public static final double driveKV = 2.44/12;
        public static final double driveKA = 0.27/12;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 3;
            public static final boolean invertDrive = false;
            public static final boolean invertSteer = true;
            public static final boolean invertEncoder = false;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(180-139.482);
            //36.123046875 + 2.28515625)
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, invertEncoder, invertDrive, invertSteer);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 6;
            public static final boolean invertDrive = false;
            public static final boolean invertSteer = false;
            public static final boolean invertEncoder = false;

            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(116.279);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, invertEncoder, invertDrive, invertSteer);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 8;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 9;
            public static final boolean invertDrive = true;
            public static final boolean invertSteer = true;
            public static final boolean invertEncoder = false;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-131.2207+180);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, invertEncoder, invertDrive, invertSteer);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 11;
            public static final int angleMotorID = 10;
            public static final int canCoderID = 12;
            public static final boolean invertDrive = false;
            public static final boolean invertSteer = true;
            public static final boolean invertEncoder = false;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-111.796);
            //-120.937
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, invertEncoder, invertDrive, invertSteer);
        }
    }

  public static class VisionConstants {

    public static final double Height_LL3 = 0.2;
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
    public static final double kP = 0.15;
    public static final double kD = 0;
    public static final boolean PivotInversion = false;
    public static final double AMP_ANGLE_ROTATIONS = 0;
    public static final double SUBWOOFER_ANGLE_ROTATIONS = 0;
    public static final double AMP = 0;
    public static final double HOME_POSITION = 8;
    public static final double AMP_POSITION = 0;
    public static final double PODIUM_POSITION = 0;
    public static final double SUBWOOFER_POSITION = 5.5;
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