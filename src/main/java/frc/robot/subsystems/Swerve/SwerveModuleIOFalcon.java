package frc.robot.subsystems.Swerve;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants.SwerveModuleConfig;

public class SwerveModuleIOFalcon implements SwerveModuleIO {
    TalonFX driveMotor;
    TalonFX steerMotor;
    CANcoder steerEncoder;
    VelocityVoltage velocityVoltage = new VelocityVoltage(0.0);
    PositionVoltage positionVoltage = new PositionVoltage(0.0);
    



    public SwerveModuleIOFalcon(SwerveModuleConfig config) {

        //drive motor
        TalonFXConfiguration for_drive = new TalonFXConfiguration();
        driveMotor = new TalonFX(config.driveID());
      

      for_drive.Slot0.kA = config.ffkA();
      for_drive.Slot0.kS = config.ffkS();
      for_drive.Slot0.kV = config.ffkV();
      for_drive.Slot0.kP = config.drivekP();
      for_drive.Slot0.kD = config.drivekD();

      for_drive.CurrentLimits.SupplyCurrentLimit = Constants.SwerveConstants.DriveMotorSupplyCurrent;
      for_drive.CurrentLimits.SupplyCurrentLimitEnable = true;
      for_drive.CurrentLimits.StatorCurrentLimit = Constants.SwerveConstants.DriveMotorStatorCurrent;
      for_drive.CurrentLimits.StatorCurrentLimitEnable = true;


      driveMotor.setInverted(config.driveMotorInverted());
      for_drive.MotorOutput.NeutralMode = NeutralModeValue.Brake;

      //cancoder


      steerEncoder = new CANcoder(config.absoluteEncoderID());
      double absolutePosition = steerEncoder.getAbsolutePosition().getValueAsDouble() - config.absoluteEncoderOffsetRotations().getRotations();
    //   steerMotor.setPosition(absolutePosition);


    //steer motor

      TalonFXConfiguration for_steer = new TalonFXConfiguration();
      steerMotor = new TalonFX(config.turnID());
  


      for_steer.Slot0.kP = config.turnkP();
      for_steer.Slot0.kD = config.turnkD();

      for_steer.CurrentLimits.SupplyCurrentLimit = Constants.SwerveConstants.SteerMotorSupplyCurrent;
      for_steer.CurrentLimits.SupplyCurrentLimitEnable = true;
      for_steer.CurrentLimits.StatorCurrentLimit = Constants.SwerveConstants.SteerMotorStatorCurrent;
      for_steer.CurrentLimits.StatorCurrentLimitEnable = true;

      

      //apply config

      for (int i = 0; i < 4; i++) {
      boolean error = driveMotor.getConfigurator().apply(for_drive, 0.1) == StatusCode.OK;
      error = error && (steerMotor.getConfigurator().apply(for_steer, 0.1) == StatusCode.OK);
      if (!error) break;
      }

      //extra steer motor config

      steerMotor.setInverted(config.steerMotorInverted());
      steerMotor.setPosition(absolutePosition);



      //optimization
      // driveMotor.optimizeBusUtilization();
      // steerMotor.optimizeBusUtilization();

      BaseStatusSignal.setUpdateFrequencyForAll(50, driveMotor.getPosition(), driveMotor.getVelocity(), steerMotor.getPosition());
    }

  public void updateInputs(ModuleIOInputs inputs) {

    inputs.driveMotorConnected =
        BaseStatusSignal.refreshAll(
                driveMotor.getPosition(),
                driveMotor.getVelocity(),
                driveMotor.getSupplyVoltage(),
                driveMotor.getSupplyCurrent()
                )
            .isOK();
    inputs.turnMotorConnected =
        BaseStatusSignal.refreshAll(
                steerMotor.getPosition(), 
                steerMotor.getVelocity(), 
                steerMotor.getSupplyVoltage(), 
                steerMotor.getSupplyCurrent())
            .isOK();


    inputs.drivePositionRotations = driveMotor.getPosition().getValueAsDouble();
    inputs.driveVelocityRotationsPerSec = driveMotor.getVelocity().getValueAsDouble();
    inputs.driveAppliedVolts = driveMotor.getSupplyVoltage().getValueAsDouble();
    inputs.Drivelatency = driveMotor.getPosition().getTimestamp().getLatency();
    inputs.driveSupplyCurrentAmps = driveMotor.getSupplyCurrent().getValueAsDouble();
    

    
    inputs.turnPosition = Rotation2d.fromRotations(steerMotor.getPosition().getValueAsDouble()).getRadians();
    inputs.turnVelocityRotationsPerSec = steerMotor.getVelocity().getValueAsDouble();
    inputs.turnAppliedVolts = steerMotor.getSupplyVoltage().getValueAsDouble();
    inputs.turnSupplyCurrentAmps = steerMotor.getSupplyCurrent().getValueAsDouble();
    inputs.Steerlatency = steerMotor.getPosition().getTimestamp().getLatency();


    inputs.currentTimestamp = Timer.getFPGATimestamp();
  }


  @Override
  public void runDriveVelocitySetpoint(double velocityRotationsPerSec) {
        driveMotor.setControl(velocityVoltage.withVelocity(velocityRotationsPerSec));
  }

  @Override
  public void runTurnPositionSetpoint(double angleRotations) {
    steerMotor.setControl(positionVoltage.withPosition(angleRotations));
  }

}
