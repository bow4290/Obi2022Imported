/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class StopDrivingCommand extends CommandBase {
  private final DrivetrainSubsystem drivetrainSubsystem;

  public StopDrivingCommand(DrivetrainSubsystem drivetrainSubsystem) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    drivetrainSubsystem.stopDrive();
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
