package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot.PivotSubsystem;
import frc.robot.subsystems.Pivot.PivotSubsystem.DesiredStates;
import frc.robot.subsystems.Rollers.IntakeSubsystem;
import frc.robot.subsystems.Rollers.IntakeSubsystem.Checkpoint;

public class IntakeManualCommand extends Command{ 

    private IntakeSubsystem intake;
    private PivotSubsystem pivot;


    public IntakeManualCommand(IntakeSubsystem intake, PivotSubsystem pivot) {
        this.intake = intake;
        this.pivot = pivot;
        addRequirements(intake, pivot);
        
    }

    @Override
    public void initialize() {
        intake.CompletedCheckpoint = Checkpoint.INITIATED;
        
        pivot.setDesiredState(DesiredStates.HOME);
    }




    

    


    
}
