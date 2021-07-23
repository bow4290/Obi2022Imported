/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimbCommand extends CommandBase {
  private final ClimberSubsystem climberSubsystem;

  public ClimbCommand(ClimberSubsystem climberSubsystem) {
    this.climberSubsystem = climberSubsystem;
    addRequirements(climberSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    climberSubsystem.climb(ClimberConstants.climberSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    climberSubsystem.climberStop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
