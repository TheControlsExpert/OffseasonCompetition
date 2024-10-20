package frc.robot.subsystems.Pivot;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Pivot.PivotIO.PivotIOInputs;
import frc.robot.subsystems.Rollers.IntakeSubsystem;
import frc.robot.subsystems.Rollers.IntakeSubsystem.Checkpoint;

public class PivotSubsystem extends SubsystemBase {


    public static enum DesiredStates {
        AMP,
        SUBWOOFER,
        PODIUM,
        PASSING,
        ALIGNING,
        TRAVELLING,
        HOME
    }

   
    private DesiredStates PivotDesiredState = DesiredStates.TRAVELLING;

    @AutoLogOutput
    private double currentSetpoint = 0.0;


    @AutoLogOutput
    private boolean atPosition = true;
    private PivotIONEO pivotIO;
    private PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
   // private PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
   private IntakeSubsystem intake;

   public PivotSubsystem(PivotIONEO pivotIO, IntakeSubsystem intake) {
       this.pivotIO = pivotIO;
       this.intake = intake;
   }


   @Override
   public void periodic() {
    SmartDashboard.putString("Pivot state", PivotDesiredState.toString());
         pivotIO.updateInputs(inputs);         

         if (PivotDesiredState.equals(DesiredStates.HOME)) {
            if (intake.CompletedCheckpoint.equals(Checkpoint.IDLE)) {
                currentSetpoint = Constants.PivotConstants.TRAVELLING_POSITION;
            }

            else {
            currentSetpoint = Constants.PivotConstants.HOME_POSITION;
            }
         }

         else if (PivotDesiredState.equals(DesiredStates.AMP)) {
            currentSetpoint = 6.35;
         }

         else if (PivotDesiredState.equals(DesiredStates.PODIUM)) {
            currentSetpoint = Constants.PivotConstants.PODIUM_POSITION;
         }

         else if (PivotDesiredState.equals(DesiredStates.SUBWOOFER)) {
            currentSetpoint = Constants.PivotConstants.SUBWOOFER_POSITION;
         }

         else if (PivotDesiredState.equals(DesiredStates.TRAVELLING)) {
            currentSetpoint = Constants.PivotConstants.TRAVELLING_POSITION;
         }





        //  else if (PivotDesiredState.equals(DesiredStates.ALIGNING)) {
        //     newsetpoint = RobotState.getAligningPivotPosition();
        //     if (newsetpoint > Constants.PivotConstants.MAX_EXTENSION) {
        //         newsetpoint = Constants.PivotConstants.MAX_EXTENSION - 1;
        //     }

        //     else if (newsetpoint < Constants.PivotConstants.MIN_EXTENSION) {
        //         newsetpoint = Constants.PivotConstants.MIN_EXTENSION + 1;
        //     }
        //   }




          else if (PivotDesiredState.equals(DesiredStates.PASSING)) {
            currentSetpoint = Constants.PivotConstants.PASSING_POSITION;
            if (currentSetpoint > Constants.PivotConstants.MAX_EXTENSION) {
                currentSetpoint = Constants.PivotConstants.MAX_EXTENSION - 1;
            }

            else if (currentSetpoint < Constants.PivotConstants.MIN_EXTENSION) {
                currentSetpoint = Constants.PivotConstants.MIN_EXTENSION + 1;
            }
          }

          


         
             pivotIO.setSetpointInRotations(currentSetpoint);
         


         if (Math.abs(currentSetpoint - inputs.encoderpos) < PivotConstants.kTolerance) {
             atPosition = true;
         }

         else {
             atPosition = false;
         }
   }



public void setDesiredState(DesiredStates state) {
       PivotDesiredState = state;
   }
    
}
