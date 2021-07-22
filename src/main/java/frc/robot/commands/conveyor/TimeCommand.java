/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.conveyor;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TimeCommand extends CommandBase {
  private double commandStartTimestamp = 0;
  private double elapsedRunTime = 0;

  public TimeCommand() {
  }

  @Override
  public void initialize() {
    commandStartTimestamp = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {
    elapsedRunTime = Timer.getFPGATimestamp() - commandStartTimestamp;
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return(elapsedRunTime >= 5);
  }
}
