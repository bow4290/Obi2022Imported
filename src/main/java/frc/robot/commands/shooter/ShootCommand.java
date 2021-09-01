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
  private static double shooterRatekP = 0.3;
  private static double shooterRateSetPoint = 20000;

  private static enum ShootingZone {
    invalid, close, far
  }

  public ShootCommand(ShooterSubsystem shooterSubsystem, ConveyorSubsystem conveyorSubsystem, Limelight limelight) {
    this.shooterSubsystem = shooterSubsystem;
    this.conveyorSubsystem = conveyorSubsystem;
    this.limelight = limelight;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    System.out.println("Shooting Command Initialized");
  }

  @Override
  public void execute() {
    double targetShooterSpeed = calculateTargetShooterSpeed();
    shooterSubsystem.setTargetShooterSpeed(targetShooterSpeed);

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
    return !isShooterEnabled();
  }

  private double calculateTargetShooterSpeed(){
    if (getShootingZone() == ShootingZone.close) {
      return ShooterConstants.shooterSpeedClose;
    } else if (getShootingZone() == ShootingZone.far) {
      return ShooterConstants.shooterSpeedFar;
    }
    return ShooterConstants.shooterSpeedInvalid;
  }

  private ShootingZone getShootingZone() {
    ShootingZone shootingZone = ShootingZone.invalid;

    if(limelight.getBumperDistance() >= ShooterConstants.minimumShooterDistance &&
       limelight.getBumperDistance() <= ShooterConstants.thresholdShooterDistance){
       shootingZone = ShootingZone.close;
    }else
    if (limelight.getBumperDistance() > ShooterConstants.thresholdShooterDistance &&
        limelight.getBumperDistance() <= ShooterConstants.maximumShooterDistance){
        shootingZone = ShootingZone.far;
    } else {
        shootingZone = ShootingZone.invalid;
    }
    return shootingZone;
  }

  private boolean isShooterEnabled() {
    return getShootingZone() != ShootingZone.invalid;
  }

}
