package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot.PivotSubsystem;
import frc.robot.subsystems.Pivot.PivotSubsystem.DesiredStates;
import frc.robot.subsystems.Rollers.IntakeSubsystem;
import frc.robot.subsystems.Rollers.IntakeSubsystem.Checkpoint;

public class OuttakeCommand extends Command{ 

    private IntakeSubsystem intake;
    private PivotSubsystem pivot;


    public OuttakeCommand(IntakeSubsystem intake, PivotSubsystem pivot) {
        this.intake = intake;
        this.pivot = pivot;

        addRequirements(intake, pivot);
        

        
    }

    @Override
    public void initialize() {
        intake.CompletedCheckpoint = Checkpoint.OUTTAKING;
        pivot.setDesiredState(DesiredStates.HOME); 
        
    }


    @Override
    public void end(boolean interrupted) {
        
        intake.CompletedCheckpoint = Checkpoint.IDLE;
        pivot.setDesiredState(DesiredStates.TRAVELLING);

        
    }

    


    
}
