package frc.robot.commands.ShooterCommands.Aligning;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Flywheels.ShooterSubsystem;
import frc.robot.subsystems.Flywheels.ShooterSubsystem.ShooterDesiredState;
import frc.robot.subsystems.Pivot.PivotSubsystem;
import frc.robot.subsystems.Pivot.PivotSubsystem.DesiredStates;
import frc.robot.subsystems.Rollers.IntakeSubsystem;
import frc.robot.subsystems.Rollers.IntakeSubsystem.Checkpoint;

public class Subwoofer extends Command {

    private ShooterSubsystem shooter;
    private PivotSubsystem pivot;
    private CommandXboxController controller;
    private IntakeSubsystem intake;

    public Subwoofer(PivotSubsystem pivot, ShooterSubsystem shooter, CommandXboxController controller, IntakeSubsystem intake) {
        this.shooter = shooter;
        this.pivot = pivot;
        this.controller = controller;
        this.intake = intake;
        addRequirements(pivot, shooter, intake);
    }

    @Override
    public void initialize() {
        shooter.setDesiredState(ShooterDesiredState.SUBWOOFER);
        pivot.setDesiredState(DesiredStates.SUBWOOFER);
    }

    @Override
    public void execute() {
        if (controller.rightTrigger().getAsBoolean()) {
            intake.CompletedCheckpoint = Checkpoint.EJECTED;        
        }
    }

    @Override
    public boolean isFinished() {
        return !intake.isTouchingLastSensor();
    }


    @Override
    public void end(boolean interrupted) {
     
        shooter.setDesiredState(ShooterDesiredState.IDLE);
        pivot.setDesiredState(DesiredStates.HOME);
        intake.CompletedCheckpoint = Checkpoint.IDLE;
    }
    
}

