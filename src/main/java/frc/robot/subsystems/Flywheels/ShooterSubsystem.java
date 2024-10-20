package frc.robot.subsystems.Flywheels;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PivotConstants;

public class ShooterSubsystem extends SubsystemBase {
    public enum ShooterDesiredState {
        ALIGNING,
        AMP,
        PODIUM,
        SUBWOOFER,
        PASSING,
        IDLE
    }
    
    private ShooterDesiredState ShooterdesiredState = ShooterDesiredState.IDLE;
    private ShooterDesiredState previousdesiredState = ShooterDesiredState.IDLE;
    @AutoLogOutput
    private double currentSetpointB = 0;
    @AutoLogOutput
    private double currentSetpointT = 0;
    @AutoLogOutput
    private boolean atPosition = true;

    

    private FlywheelIONEO shooterIO;
    private FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();


    public ShooterSubsystem(FlywheelIONEO shooterIO) {
        this.shooterIO = shooterIO;
    }

    @Override
    public void periodic() {
        shooterIO.updateInputs(inputs);

         if (previousdesiredState != ShooterdesiredState) {
             Logger.recordOutput("ShooterDesiredState", ShooterdesiredState.toString());
              previousdesiredState = ShooterdesiredState;
         }

         double newsetpointB = 0;
         double newsetpointT = 0;

            if (ShooterdesiredState.equals(ShooterDesiredState.IDLE)) {
                newsetpointB = Constants.ShooterConstants.IDLE_POSITION;
                newsetpointT = Constants.ShooterConstants.IDLE_POSITION;

            }
    
            else if (ShooterdesiredState.equals(ShooterDesiredState.AMP)) {
                newsetpointB = Constants.ShooterConstants.AMP_POSITIONB;
                newsetpointT = Constants.ShooterConstants.AMP_POSITIONT;

            }
    
            else if (ShooterdesiredState.equals(ShooterDesiredState.PODIUM)) {
                newsetpointT = Constants.ShooterConstants.PODIUM_POSITION;
                newsetpointB = Constants.ShooterConstants.PODIUM_POSITION;
            }
    
            else if (ShooterdesiredState.equals(ShooterDesiredState.SUBWOOFER)) {
                newsetpointB = Constants.ShooterConstants.SUBWOOFER_POSITION;
                newsetpointT = Constants.ShooterConstants.SUBWOOFER_POSITION;
            }
    
            else if (ShooterdesiredState.equals(ShooterDesiredState.PASSING)) {
                newsetpointB = Constants.ShooterConstants.PASSING_POSITION;
                newsetpointT = Constants.ShooterConstants.PASSING_POSITION;
            }
    
            // else if (ShooterdesiredState.equals(ShooterDesiredState.ALIGNING)) {
            //     newsetpointT = RobotState.getAligningShooterVelocities();
            //     newsetpointB = RobotState.getAligningShooterVelocities();

            // }

            if (newsetpointB != currentSetpointB | newsetpointT != currentSetpointT) {
                shooterIO.set_B_vel(newsetpointB);
                shooterIO.set_T_vel(newsetpointT);
                
                currentSetpointB = newsetpointB;
                currentSetpointT = newsetpointT;
        
            }


              if (Math.abs(currentSetpointT - inputs.Tvelocity) < PivotConstants.kTolerance && 
                  Math.abs(currentSetpointB - inputs.Bvelocity) < PivotConstants.kTolerance) {
                  atPosition = true;
              }
    
              else {
                  atPosition = false;
              }
             
    }

    public void setDesiredState(ShooterDesiredState state) {
        ShooterdesiredState = state;
    }


    
    
}
