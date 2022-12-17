/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends CommandBase {
  private final ShooterSubsystem shooterSubsystem;
  private final ConveyorSubsystem conveyorSubsystem;
  private static double shooterRatekP = 0.3;
  private static double shooterRateSetPoint = 20000;

  public ShootCommand(ShooterSubsystem shooterSubsystem, ConveyorSubsystem conveyorSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.conveyorSubsystem = conveyorSubsystem;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    System.out.println("Shooting Command Initialized");
  }

  @Override
  public void execute() {
    double targetShooterSpeed = ShooterConstants.shooterSpeedDefault;
    SmartDashboard.putNumber("Target Shooter Speed: ", targetShooterSpeed);
    double targetShooterRate = (ShooterConstants.shooterMotorToRateSlope * targetShooterSpeed) - ShooterConstants.shooterMotorToRateIntercept;                   // Roughly converts shooter motor speed (0 to 1) into encoder rate (0 to 215,000)
    shooterSubsystem.setTargetShooterRate(targetShooterRate);
    
    // Proportional Control Compensation
    double shooterRateError = targetShooterRate - conveyorSubsystem.getEncoderRate() + shooterRateSetPoint;                                                     // Shooter rate error is the target rate minus the encoder rate plus an offset
    double shooterRateCorrection = shooterRatekP * shooterRateError;                                                                                           // Obtain corrected rate
    double shooterMotorSpeedCorrection = (Math.abs(shooterRateCorrection) + ShooterConstants.shooterMotorToRateIntercept) / ShooterConstants.shooterMotorToRateSlope;    // Convert rate speed back to motor speed value (0 to 1)

    if (shooterRateCorrection >= 0.0) {
      shooterSubsystem.shooterShoot(targetShooterSpeed + shooterMotorSpeedCorrection);
    } else{
      shooterSubsystem.shooterShoot(targetShooterSpeed - shooterMotorSpeedCorrection);
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.shooterStop();
    System.out.println("Shooting Command Ended");
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
