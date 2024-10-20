package frc.robot.subsystems.SwerveActual;

import frc.robot.subsystems.SwerveActual.SwerveModule;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.proto.Kinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Velocity;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;

import static edu.wpi.first.units.MutableMeasure.mutable;


import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.VoltsPerMeterPerSecond;

import java.sql.Driver;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;


public class Swerve extends SubsystemBase {
    public SwerveDrivePoseEstimator estimator;
    //public SwerveDriveOdometry odometry;
    public SwerveModule[] mSwerveMods;
    public AHRS gyro;
        
    
    
    
    SendableChooser<String> AutoChooser = new SendableChooser<String>();
    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
    private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

    private SysIdRoutine swervRoutine = new SysIdRoutine(
        new Config(),
        new SysIdRoutine.Mechanism( 
            (Measure<Voltage> volts) -> {setVoltageSwerve(volts);},
            log -> {
                // Record a frame for the left motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("mod0")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            mSwerveMods[0].getOutput() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(mSwerveMods[0].getDistance(), Meters))
                    .linearVelocity(m_velocity.mut_replace(mSwerveMods[0].getEncoderV(), MetersPerSecond));
                log.motor("mod1")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            mSwerveMods[1].getOutput() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(mSwerveMods[1].getDistance(), Meters))
                    .linearVelocity(m_velocity.mut_replace(mSwerveMods[1].getEncoderV(), MetersPerSecond));
                log.motor("mod2")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            mSwerveMods[2].getOutput() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(mSwerveMods[2].getDistance(), Meters))
                    .linearVelocity(m_velocity.mut_replace(mSwerveMods[2].getEncoderV(), MetersPerSecond));
                log.motor("mod3")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            mSwerveMods[3].getOutput() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(mSwerveMods[3].getDistance(), Meters))
                    .linearVelocity(m_velocity.mut_replace(mSwerveMods[3].getEncoderV(), MetersPerSecond));
                // Record a frame for the right motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
            },
                
            this)
          
    );
    private Pose2d oldPose;

    public Swerve() {
        gyro = new AHRS(Port.kUSB);
        resetGyro();
        

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

         estimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions(), new Pose2d(1.33, 5.52, Rotation2d.fromDegrees(0)));
        
        //odometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());
        
        //        
// AutoBuilder.configureHolonomic(
//             this::getPose, // Robot pose supplier
//             this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
//             this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
//             this::driveRobotOriented, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
//             new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
//                     new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
//                     new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
//                     4.5, // Max module speed, in m/s
//                     0.42905, // Drive base radius in meters. Distance from robot center to furthest module.
//                     new ReplanningConfig() // Default path replanning config. See the API for the options here
//             ),
//             () -> {
//               // Boolean supplier that controls when the path will be mirrored for the red alliance
//               // This will flip the path being followed to the red side of the field.
//               // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

//               var alliance = DriverStation.getAlliance();
//               if (alliance.isPresent()) {
//                 return alliance.get() == DriverStation.Alliance.Red;
//               }
//               return true;
//             },
//             this // Reference to this subsystem to set requirements
            
        
//     );
//       PathPlannerLogging.setLogActivePathCallback((poses) -> field2.getObject("path").setPoses(poses));
//     SmartDashboard.putData("Field", field2);
  }




  public ChassisSpeeds getRobotRelativeSpeeds() {
    return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    
  }

  public ChassisSpeeds getFieldRelativeSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotRelativeSpeeds(), getHeading());
    
  }




  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return swervRoutine.quasistatic(direction);
  }

  /**
   * Returns a command that will execute a dynamic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return swervRoutine.dynamic(direction);
  }

  public void setVoltageSwerve(Measure<Voltage> volts) {
    mSwerveMods[0].setVoltage(-volts.in(Volts));
    mSwerveMods[1].setVoltage(-volts.in(Volts));
    mSwerveMods[2].setVoltage(-volts.in(Volts));
    mSwerveMods[3].setVoltage(-volts.in(Volts));
  }

  public void driveRobotOriented(ChassisSpeeds speeds) {
     ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(speeds, 0.02);

    setModuleStates(Constants.Swerve.swerveKinematics.toSwerveModuleStates(targetSpeeds));

    }


    public Pose2d getPose() {
        return estimator.getEstimatedPosition();
    }

        
    

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getGyroYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    
   
    public void setPose(Pose2d pose) {
        SmartDashboard.putBoolean("pose is set", true);
        estimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
       // odometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading){
       estimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
       //odometry.resetPosition(heading, getModulePositions(), getPose());
    }

    public void zeroHeading(){
      estimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
      //odometry.resetPosition(getGyroYaw(), getModulePositions(), getPose());
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(360  - gyro.getYaw());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public void resetGyro(){
        gyro.reset();
    }
    @Override
    public void periodic(){
      SmartDashboard.putNumber("gyro from pose", getPose().getRotation().getDegrees());
       // oldPose = estimator.getEstimatedPosition();
        //odometry.update(getGyroYaw(), getModulePositions());
        //estimator.updateWithTime(Timer.getFPGATimestamp(), getGyroYaw(), getModulePositions());
       // updateVision2();
      //  field2.setRobotPose(estimator.getEstimatedPosition());
        
        

        

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
        SmartDashboard.putNumber("GYRO READING", getGyroYaw().getDegrees());
        
    }

    // public Command getSelectedAuto() {

        

    //     // SmartDashboard.putBoolean("Auto SELECTED", true);
    //     String auto = AutoChooser.getSelected();
    //     setPose(PathPlannerAuto.getStaringPoseFromAutoFile(auto));


        
    //     // Create a path following command using AutoBuilder. This will also trigger event markers.
    //     return new PathPlannerAuto(auto);

        
    // }

    // public boolean isHalfway() {
    //     if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
    //        return getPose().getX() < 8;

    //     }

    //     return getPose().getX() > 8;
         
    // }

    

//     public void updateVision() {
//         //Limelight3
        
//         Pose2d ll3pose = ll3.getVision().getPose();
//        // field.setRobotPose(ll3pose);
//         if (ll3.mode == Limelight3.MODES.APRILTAGS) {
//             if (!ll3.getVision().getisNew() | ll3.getVision().getPose().getX() != 0 | ll3.isNew == false ) {
//                 return;
//             }

//             else {
//                 double xySTD;
//                 double rotationSTD;
//                 if (ll3.getVisionStatistics()[2] >= 2) {
//                     xySTD = 0.5;
//                     rotationSTD = 6;
//                   }
//                   // 1 target with large area and close to estimated pose
//                   else if (ll3.getVisionStatistics()[3] > 0.1 && ll3pose.getTranslation().getDistance(getPose().getTranslation()) < 0.5) {
//                     xySTD = 1.0;
//                     rotationSTD = 12;
//                   }
//                   // 1 target farther away and estimated pose is close
//                   else if (ll3.getVisionStatistics()[3] > 0.01 && ll3pose.getTranslation().getDistance(getPose().getTranslation()) < 0.3) {
//                     xySTD = 2.0;
//                     rotationSTD = 30;
//                   }
//                   // conditions don't match to add a vision measurement
//                   else {
//                     return;
//                   }


//                    estimator.setVisionMeasurementStdDevs(
//           VecBuilder.fill(xySTD, xySTD, Units.degreesToRadians(rotationSTD)));
//       estimator.addVisionMeasurement(ll3pose, Timer.getFPGATimestamp());
//     }
// }

// //Limelight 2
// Pose2d ll2pose = ll2.getVision().getPose();
// SmartDashboard.putNumber("x val", ll3pose.getX());
// SmartDashboard.putNumber("y val", ll3pose.getY());
// field.setRobotPose(ll2pose);
       
//             if (!ll2.getVision().getisNew() | ll2.getVision().getPose().getX() != 0 | ll2.isNew == false ) {
//                 return;
//             }

//             else {
//                 double xySTD;
//                 double rotationSTD;
               
//                   // 1 target with large area and close to estimated pose
//                    if (ll2.getVisionStatistics()[0] > 0.05 && ll2pose.getTranslation().getDistance(getPose().getTranslation()) < 0.5) {
//                     xySTD = 1.1;
//                     rotationSTD = 9999999;
//                   }
//                   // 1 target farther away and estimated pose is close
//                   else if (ll2.getVisionStatistics()[] > 0.035 && ll2pose.getTranslation().getDistance(getPose().getTranslation()) < 0.3) {
//                     xySTD = 2.2;
//                     rotationSTD = 9999999;
//                   }
//                   // conditions don't match to add a vision measurement
//                   else {
//                     return;
//                   }


//                    estimator.setVisionMeasurementStdDevs(
//           VecBuilder.fill(xySTD, xySTD, rotationSTD));
//       estimator.addVisionMeasurement(ll2pose, Timer.getFPGATimestamp());
//     }
// }


public void updateVision2() {
//         //Limelight3
        
//         Pose2d ll3pose = ll3.getVision3().getPose();
//         SmartDashboard.putNumber("x val", ll3pose.getX());
// SmartDashboard.putNumber("y val", ll3pose.getY());
//        field.setRobotPose(ll3pose);
//         if (ll3.mode == Limelight3.MODES.APRILTAGS) {
//             if (!ll3.getVision3().getisNew() | ll3.getVision3().getPose().getX() != 0 | ll3.isNew == false ) {
//                 return;
//             }

//             else {
//                 double xySTD;
//                 double rotationSTD;
//                 if (ll3.getVisionStatistics()[2] > 1) {
//                     xySTD = 0.25;
//                     rotationSTD = 999999;
//                      }
//                   // 1 target with large area and close to estimated pose
//                   else if (ll3.getVisionStatistics()[0] > 0.1 && ll3pose.getTranslation().getDistance(getPose().getTranslation()) < 0.5) {
//                     xySTD = 1.0;
//                     rotationSTD = 12;
//                   }
//                   // 1 target farther away and estimated pose is close
//                   else if (ll3.getVisionStatistics()[0] > 0.03 && ll3pose.getTranslation().getDistance(getPose().getTranslation()) < 0.3) {
//                     xySTD = 2.0;
//                     rotationSTD = 30;
//                   }
//                   // conditions don't match to add a vision measurement
//                   else {
//                     return;
//                   }


//                    estimator.setVisionMeasurementStdDevs(
//           VecBuilder.fill(xySTD, xySTD, Units.degreesToRadians(rotationSTD)));
//       estimator.addVisionMeasurement(ll3pose, Timer.getFPGATimestamp());
//     }
// }

//Limelight 2
// Pose2d ll2pose;
// if (ll2.getNumUsableTags() < 2) {
//  ll2pose = ll2.getVision3().getPose();
// }
// else {
//     ll2pose = ll2.getVision().getPose();

// }
// //field2.setRobotPose(ll2pose);
       

// //         field.setRobotPose(ll2pose);
        
//             if (!ll2.getVision3().getisNew() | ll2.getVision3().getPose().getX() == 0) {

//                 SmartDashboard.putBoolean("is giving pose", false);
//                 SmartDashboard.putBoolean("ll is new", ll2.getVision3().getisNew());
//                 SmartDashboard.putNumber("ll x", ll2.getVision3().getPose().getX());
//                 return;
//             }

//             else {
//                 double xySTD;
//                 double rotationSTD;
//                 SmartDashboard.putNumber("best area", ll2.getVisionStatistics()[0]);
//                 SmartDashboard.putNumber("best distance", ll2.getVisionStatistics()[1]);


//                 if (ll2.getVisionStatistics()[2] > 1 && ll2.getVisionStatistics()[0] > 0.004) {
//                     xySTD = 5;
//                     rotationSTD = 999999;
//                      }
//                   // 1 target with large area and close to estimated pose
//                   else if (ll2.getVisionStatistics()[0] > 0.004 && ll2pose.getTranslation().getDistance(getPose().getTranslation()) < 0.15) {
//                     xySTD = 8;
//                     rotationSTD = 999999999;
//                   }
//                   // 1 target farther away and estimated pose is close
//                   else if (ll2.getVisionStatistics()[0] > 0.0005 && ll2pose.getTranslation().getDistance(getPose().getTranslation()) < 0.3) {
//                     xySTD = 10;
//                     rotationSTD = 999999999;
//                   }
//                   // conditions don't match to add a vision measurement
//                   else {
//                      SmartDashboard.putBoolean("is giving pose", false);
//                     return;
                    
//                   }


//                    estimator.setVisionMeasurementStdDevs(
//           VecBuilder.fill(xySTD, xySTD, Units.degreesToRadians(rotationSTD)));
//       estimator.addVisionMeasurement(ll2pose, Timer.getFPGATimestamp());
//       SmartDashboard.putBoolean("is giving pose", true);

//     }
}
}

        
    
