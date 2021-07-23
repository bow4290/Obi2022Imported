/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterStatus;

public class ToggleShooterSolenoidCommand extends CommandBase {
  private final ShooterSubsystem shooterSubsystem;

  public ToggleShooterSolenoidCommand(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    if (ShooterSubsystem.getShooterPosition() == ShooterStatus.DOWN) {
      shooterSubsystem.shooterUp();
    } else{
      shooterSubsystem.shooterDown();
    }
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
