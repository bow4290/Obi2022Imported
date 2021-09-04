/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.sensors.Limelight.*;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants.ShooterConstants;
import frc.robot.sensors.Limelight;

public class LimelightInitCommand extends CommandBase {
  private final ShooterSubsystem shooterSubsystem;
  private final Limelight limelight;
  boolean initFinished = false;

  private static enum ShootingZone {
    Default, close, far
  }
  
  public LimelightInitCommand(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    limelight = new Limelight();
  }

  @Override
  public void initialize() {
    System.out.println("Limelight Init Command Initialized");
    limelight.setLedMode(LedMode.ledOn);
    limelight.setCamMode(CamMode.vision);
    initFinished = false;
  }
  
  @Override
  public void execute(){
    shooterSubsystem.setTargetShooterSpeed(calculateTargetShooterSpeed());
    initFinished = true;
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Limelight Init Command Finished"); 
  }

  @Override
  public boolean isFinished() { 
    return initFinished;
  }

  private ShootingZone getShootingZone() {
    ShootingZone shootingZone = ShootingZone.Default;

    if(limelight.getBumperDistance() >= ShooterConstants.minimumShooterDistance &&
       limelight.getBumperDistance() <= ShooterConstants.thresholdShooterDistance){
       shootingZone = ShootingZone.close;
    }else
    if (limelight.getBumperDistance() > ShooterConstants.thresholdShooterDistance &&
        limelight.getBumperDistance() <= ShooterConstants.maximumShooterDistance){
        shootingZone = ShootingZone.far;
    } else {
        shootingZone = ShootingZone.Default;
    }
    SmartDashboard.putNumber("Bumper-Tower Distance: ", limelight.getBumperDistance());
    return shootingZone;
  }

  private double calculateTargetShooterSpeed(){
    if (getShootingZone() == ShootingZone.close) {
      return ShooterConstants.shooterSpeedClose;
    } else if (getShootingZone() == ShootingZone.far) {
      return ShooterConstants.shooterSpeedFar;
    } else {
    return ShooterConstants.shooterSpeedDefault;
    }
  }
  
}
