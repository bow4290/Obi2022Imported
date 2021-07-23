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
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends CommandBase {
  private final ShooterSubsystem shooterSubsystem;
  private final ConveyorSubsystem conveyorSubsystem;
  private static double shooterMotorSpeed = 0;
  private static double shooterRateSpeed = 0;
  private static double shooterRateError = 0;
  private static double shooterRatekP = 0.03;
  private static double shooterRateCorrection = 0;
  private static double shooterRateSetPoint = 15000;
  private static double shooterMotorSpeedCorrection = 0;

  public ShootCommand(ShooterSubsystem shooterSubsystem, ConveyorSubsystem conveyorSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.conveyorSubsystem = conveyorSubsystem;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    switch(RobotContainer.LimelightShootingPosition){
      case 0:     // Trench Front
        shooterMotorSpeed = ShooterConstants.shootSpeedPosition0;
        break;
      case 1:     // Trench Back
        shooterMotorSpeed = ShooterConstants.shootSpeedPosition1;
        break;
      case 2:
        shooterMotorSpeed = ShooterConstants.shootSpeedPosition2;
        break;
      case 3:     // Auto start line
        shooterMotorSpeed = ShooterConstants.shootSpeedPosition3;
        break;
    }

    shooterRateSpeed = (ShooterConstants.shooterMotorToRateSlope * shooterMotorSpeed) - ShooterConstants.shooterMotorToRateIntercept;                   // Roughly converts shooter motor speed (0 to 1) into encoder rate (0 to 215,000)
    shooterRateError = shooterRateSpeed - conveyorSubsystem.getEncoderRate();                                                                           // Shooter rate error is the target rate minus the encoder rate
    shooterRateCorrection = shooterRatekP * (shooterRateError + shooterRateSetPoint);                                                                   // Obtain corrected rate but add an offset (setpoint)
    shooterMotorSpeedCorrection = (shooterRateCorrection + ShooterConstants.shooterMotorToRateIntercept) / ShooterConstants.shooterMotorToRateSlope;    // Convert rate speed back to motor speed value (0 to 1)
    shooterSubsystem.shooterShoot(shooterMotorSpeed + shooterMotorSpeedCorrection);                                                                     // Shoot add target speed plus corrected speed

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
