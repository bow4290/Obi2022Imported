/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends CommandBase {
  private final ShooterSubsystem shooterSubsystem;

  public ShootCommand(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    switch(RobotContainer.LimelightShootingPosition){
      case 0:
        shooterSubsystem.shooterShoot(ShooterConstants.shootSpeedPosition0);
        break;
      case 1:
        shooterSubsystem.shooterShoot(ShooterConstants.shootSpeedPosition1);
        break;
      case 2:
        shooterSubsystem.shooterShoot(ShooterConstants.shootSpeedPosition2);
        break;
      case 3:
        shooterSubsystem.shooterShoot(ShooterConstants.shootSpeedPosition3);
        break;
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.shooterStop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
