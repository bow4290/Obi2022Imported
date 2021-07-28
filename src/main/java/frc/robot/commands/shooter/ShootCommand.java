/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.sensors.Limelight;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends CommandBase {
  private final ShooterSubsystem shooterSubsystem;
  private final ConveyorSubsystem conveyorSubsystem;
  private Limelight limelight;
  private static double shooterMotorSpeed = 0;
  private static double shooterRateSpeed = 0;
  private static double shooterRateError = 0;
  private static double shooterRatekP = 0.3;
  private static double shooterRateCorrection = 0;
  private static double shooterRateSetPoint = 15000;
  private static double shooterMotorSpeedCorrection = 0;

  public ShootCommand(ShooterSubsystem shooterSubsystem, ConveyorSubsystem conveyorSubsystem, Limelight limelight) {
    this.shooterSubsystem = shooterSubsystem;
    this.conveyorSubsystem = conveyorSubsystem;
    this.limelight = limelight;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double limelightShootingPosition = limelight.getPipeline();
    if (limelightShootingPosition == 0) { // Trench Front
      shooterMotorSpeed = ShooterConstants.shootSpeedPosition0;
    } else if (limelightShootingPosition == 1) { // Trench Back
      shooterMotorSpeed = ShooterConstants.shootSpeedPosition1;
    } else if (limelightShootingPosition == 2) {
      shooterMotorSpeed = ShooterConstants.shootSpeedPosition2;
    } else if (limelightShootingPosition == 3) { // Auto Line
      shooterMotorSpeed = ShooterConstants.shootSpeedPosition3;
    }

    shooterRateSpeed = (ShooterConstants.shooterMotorToRateSlope * shooterMotorSpeed) - ShooterConstants.shooterMotorToRateIntercept;                   // Roughly converts shooter motor speed (0 to 1) into encoder rate (0 to 215,000)
    shooterRateError = shooterRateSpeed - conveyorSubsystem.getEncoderRate() + shooterRateSetPoint;                                                     // Shooter rate error is the target rate minus the encoder rate plus an offset
    shooterRateCorrection = shooterRatekP * shooterRateError;                                                                                           // Obtain corrected rate
    shooterMotorSpeedCorrection = (Math.abs(shooterRateCorrection) + ShooterConstants.shooterMotorToRateIntercept) / ShooterConstants.shooterMotorToRateSlope;    // Convert rate speed back to motor speed value (0 to 1)

    if(shooterRateCorrection >= 0){
      shooterSubsystem.shooterShoot(shooterMotorSpeed + shooterMotorSpeedCorrection);
    } else{
      shooterSubsystem.shooterShoot(shooterMotorSpeed - shooterMotorSpeedCorrection);
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
