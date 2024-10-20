package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Flywheels.ShooterSubsystem;
import frc.robot.subsystems.Pivot.PivotSubsystem;
import frc.robot.subsystems.Pivot.PivotSubsystem.DesiredStates;
import frc.robot.subsystems.Rollers.IntakeSubsystem;
import frc.robot.subsystems.Rollers.IntakeSubsystem.Checkpoint;

public class Shoot extends Command {

    private PivotSubsystem pivot;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;

    public Shoot(IntakeSubsystem intake, PivotSubsystem pivot, ShooterSubsystem shooter) {
        this.pivot = pivot;
        this.intake = intake;
        this.shooter = shooter;

        addRequirements(intake, pivot, shooter);

        
    }

    @Override
    public void initialize() {
        intake.CompletedCheckpoint = Checkpoint.EJECTED;

    }


    @Override
    public void end(boolean interrupted) {
        //make sure not to be a monkey
        if (intake.CompletedCheckpoint.equals(Checkpoint.IDLE)) {
            pivot.setDesiredState(DesiredStates.TRAVELLING);
            
        }
    }

   
    
}
