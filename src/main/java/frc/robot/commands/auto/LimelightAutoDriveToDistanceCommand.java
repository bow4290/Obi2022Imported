/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.sensors.Limelight;
import frc.robot.subsystems.DrivetrainSubsystem;

public class LimelightAutoDriveToDistanceCommand extends CommandBase {
  private final DrivetrainSubsystem drivetrainSubsystem;
  private final Limelight limelight;
  private double error = 0;
  private double dt = 0;
  private double sumError = 0;
  private double lastTimestamp = 0;
  private double lastError = 0;
  private double errorRate = 0;
  private double correctedLeftMotorSpeed = 0;
  private double correctedRightMotorSpeed = 0;
  private double kpAdjustment = 0;
  private double kiAdjustment = 0;
  private double kdAdjustment = 0;
  private int counter = 0;
  
  public LimelightAutoDriveToDistanceCommand(DrivetrainSubsystem drivetrainSubsystem, Limelight limelight) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.limelight = limelight;
    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    System.out.println("Running Limelight Distance Command");
    lastTimestamp = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {
    double yError = limelight.getYError();
    SmartDashboard.putNumber("Limelight Y Error: ", yError);
    dt = Timer.getFPGATimestamp() - lastTimestamp;
    error = yError; // Distance is directly proportional to the Y (vertical) error
    
    // Distance Integral Gain Calculation
    if (Math.abs(error) < LimelightConstants.distanceIntegralWindow) {
      sumError += error * dt;
    } else {
      sumError = 0;
    }

    // Distance Derivative Gain Calculation
    errorRate = (error - lastError) / dt;

    // Distance PID Compensation
    kpAdjustment = LimelightConstants.kpDistance3 * error;
    kiAdjustment = LimelightConstants.kiDistance3 * sumError;
    kdAdjustment = LimelightConstants.kdDistance3 * errorRate;

    correctedLeftMotorSpeed = kpAdjustment + kiAdjustment + kdAdjustment;
    correctedRightMotorSpeed = kpAdjustment + kiAdjustment + kdAdjustment;
    
    // Set maximum drive speed (forward and reverse)
    if (correctedLeftMotorSpeed > LimelightConstants.maxLimelightDriveSpeed){
      correctedLeftMotorSpeed = LimelightConstants.maxLimelightDriveSpeed;
    } else if (correctedLeftMotorSpeed < -LimelightConstants.maxLimelightDriveSpeed){
      correctedLeftMotorSpeed = -LimelightConstants.maxLimelightDriveSpeed;
    }
    
    if (correctedRightMotorSpeed > LimelightConstants.maxLimelightDriveSpeed){
      correctedRightMotorSpeed = LimelightConstants.maxLimelightDriveSpeed;
    } else if (correctedRightMotorSpeed < -LimelightConstants.maxLimelightDriveSpeed){
      correctedRightMotorSpeed = -LimelightConstants.maxLimelightDriveSpeed;
    }

    drivetrainSubsystem.drive(correctedLeftMotorSpeed, correctedRightMotorSpeed);

    lastTimestamp = Timer.getFPGATimestamp();
    lastError = error;
    SmartDashboard.putNumber("Distance Error Rate: ", errorRate);
    SmartDashboard.putNumber("Distance Sum Error: ", sumError);

    if (Math.abs(error) < 0.5){
      counter++;
    } else{
      counter = 0;
    }
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stopDrive();
    counter = 0;
    System.out.println("Limelight Distance Command Interrupted");
  }

  @Override
  public boolean isFinished() {
    return (counter > 50);
  }
}
