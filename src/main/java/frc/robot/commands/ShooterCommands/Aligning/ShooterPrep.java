package frc.robot.commands.ShooterCommands.Aligning;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Flywheels.ShooterSubsystem;
import frc.robot.subsystems.Flywheels.ShooterSubsystem.ShooterDesiredState;
import frc.robot.subsystems.Pivot.PivotSubsystem;
import frc.robot.subsystems.Pivot.PivotSubsystem.DesiredStates;

public class ShooterPrep extends Command {

    private PivotSubsystem pivot;
    private ShooterSubsystem shooter;




    public ShooterPrep(ShooterSubsystem shooter, PivotSubsystem pivot) {
        this.pivot = pivot;
        this.shooter = shooter;
        addRequirements(pivot, shooter);
      
    }

    @Override
    public void initialize() {
        pivot.setDesiredState(DesiredStates.ALIGNING);
        shooter.setDesiredState(ShooterDesiredState.ALIGNING);
    }



    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            pivot.setDesiredState(DesiredStates.TRAVELLING);
            shooter.setDesiredState(ShooterDesiredState.IDLE);
        }
    }

    
}
