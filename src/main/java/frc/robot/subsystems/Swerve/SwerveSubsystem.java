package frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.Constants.SwerveConstants;

public class SwerveSubsystem extends SubsystemBase {
    SwerveModule[] modules = new SwerveModule[4];
    GyroIO_Navx gyroIO = new GyroIO_Navx();
    GyroIOInputsAutoLogged inputsgyro = new GyroIOInputsAutoLogged();
    
    

    public SwerveSubsystem() {
         modules[0] = new SwerveModule(new SwerveModuleIOFalcon(Constants.SwerveConstants.SwerveConfig0), 0);
         modules[1] = new SwerveModule(new SwerveModuleIOFalcon(Constants.SwerveConstants.SwerveConfig1), 1);
         modules[2] = new SwerveModule(new SwerveModuleIOFalcon(Constants.SwerveConstants.SwerveConfig2), 2);
         modules[3] = new SwerveModule(new SwerveModuleIOFalcon(Constants.SwerveConstants.SwerveConfig3), 3);

        gyroIO.resetGyro();
        
        
    }

    @Override
    public void periodic() {
        gyroIO.updateInputs(inputsgyro);
        
        for (SwerveModule module : modules) {
            module.updateInputs();
        }

        

        SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
        double avgLatency = 0;
        double avgTimestamp = 0;
        for (int i = 0; i<4; i++) {
            moduleDeltas[i] = modules[i].getWheelPosition();
            avgLatency += modules[i].getModInputs().Drivelatency + modules[i].getModInputs().Steerlatency;
            avgTimestamp += modules[i].getModInputs().currentTimestamp;

        }

        avgLatency = avgLatency / 8;
        avgTimestamp = avgTimestamp/4;

        for (int i = 0; i<4; i++) {
            SmartDashboard.putNumber("setpoint drive" + i, modules[i].previousState.speedMetersPerSecond);
        }

        RobotState.publishOdometry(moduleDeltas, inputsgyro.yawPositionRads, avgTimestamp - avgLatency, inputsgyro.yawvelrads);
    }  
    
    
    public void setChassisSpeeds(ChassisSpeeds speedsfieldrelative) {
        SwerveModuleState[] swerveModuleStates =
            Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
            speedsfieldrelative.vxMetersPerSecond, 
            speedsfieldrelative.vyMetersPerSecond, 
            speedsfieldrelative.omegaRadiansPerSecond,
            Rotation2d.fromRadians(inputsgyro.yawPositionRads)));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,  Constants.SwerveConstants.MaxSpeedMetersPerSecond_Modules);

        for (int i = 0; i<4; i++) {
            modules[i].setSetpoint(swerveModuleStates[i]);
        }
        
        
    
 }
}
