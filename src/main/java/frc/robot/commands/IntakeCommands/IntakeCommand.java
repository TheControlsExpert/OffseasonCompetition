package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Pivot.PivotSubsystem;
import frc.robot.subsystems.Pivot.PivotSubsystem.DesiredStates;
import frc.robot.subsystems.Rollers.IntakeSubsystem;
import frc.robot.subsystems.Rollers.IntakeSubsystem.Checkpoint;


public class IntakeCommand extends Command{ 

    private IntakeSubsystem intake;
    private PivotSubsystem pivot;


    public IntakeCommand(IntakeSubsystem intake, PivotSubsystem pivot) {
        this.intake = intake;
        this.pivot = pivot;
        addRequirements(intake, pivot);

       

    }

    @Override
    public void initialize() {
        intake.CompletedCheckpoint = Checkpoint.INITIATED;
        pivot.setDesiredState(DesiredStates.HOME);
    }

    @Override
    public void end(boolean interrupted) {
        if (intake.CompletedCheckpoint.equals(Checkpoint.INITIATED)) {
            intake.CompletedCheckpoint = Checkpoint.IDLE;
            pivot.setDesiredState(DesiredStates.TRAVELLING);

        }

        
    }

    


    
}
