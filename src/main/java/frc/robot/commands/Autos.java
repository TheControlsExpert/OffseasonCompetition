// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Flywheels.ShooterSubsystem;
import frc.robot.subsystems.Flywheels.ShooterSubsystem.ShooterDesiredState;
import frc.robot.subsystems.Pivot.PivotSubsystem;
import frc.robot.subsystems.Pivot.PivotSubsystem.DesiredStates;
import frc.robot.subsystems.Rollers.IntakeSubsystem;
import frc.robot.subsystems.SwerveActual.Swerve;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Autos extends Command {

  private Swerve swervo;
  private PivotSubsystem pivot;
  private ShooterSubsystem shooter;
  double initTime;
  private IntakeSubsystem intake;

  public Autos(Swerve swervo, PivotSubsystem pivot, ShooterSubsystem shooter, IntakeSubsystem intake) {

    this.swervo = swervo;
    this.pivot = pivot;
    this.shooter = shooter;
    this.intake = intake;
}


  @Override
  public void initialize() {
    pivot.setDesiredState(DesiredStates.SUBWOOFER);
    shooter.setDesiredState(ShooterDesiredState.SUBWOOFER);
     initTime = Timer.getFPGATimestamp();
    
  }

  @Override
  public void execute() {
    if (Timer.getFPGATimestamp() - initTime > 3) {
      intake.CompletedCheckpoint = IntakeSubsystem.Checkpoint.EJECTED;
    }

    
  }

  @Override
  public void end(boolean interrupted) {
    pivot.setDesiredState(DesiredStates.TRAVELLING);
    shooter.setDesiredState(ShooterDesiredState.IDLE);


    
    
  }

  @Override
  public boolean isFinished() {
    return !intake.isTouchingLastSensor();
  }

}
  
