/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.sensors.Limelight;
import frc.robot.subsystems.DrivetrainSubsystem;

public class LimelightDriveToHeadingCommand extends CommandBase {
  private final DrivetrainSubsystem drivetrainSubsystem;
  private final Limelight limelight;
  private double dt = 0;
  private double lastTimestamp = 0;
  private double lastError = 0;
  private double errorRate = 0;
  private double error = 0;
  private double sumError = 0;
  private double correctedLeftMotorSpeed = 0;
  private double correctedRightMotorSpeed = 0;
  private boolean motorsMeetThresholdSpeed = false;
  private double motorTurnSpeedRatio = 0;
  private double kpAdjustment = 0;
  private double kiAdjustment = 0;
  private double kdAdjustment = 0;
  private int counter = 0;
  
  public LimelightDriveToHeadingCommand(DrivetrainSubsystem drivetrainSubsystem, Limelight limelight) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.limelight = limelight;
    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    System.out.println("Running Limelight Heading Command");
    lastTimestamp = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {
    double xError = limelight.getXError();
    SmartDashboard.putNumber("Limelight X Error: ", xError);
    dt = Timer.getFPGATimestamp() - lastTimestamp;
    error = xError;

    // Turn Integral Gain Calculation
    if (Math.abs(error) < LimelightConstants.turnIntegralWindow) {
      sumError += error * dt;
    } else {
      sumError = 0;
    }

    // Turn Derivative Gain Calculation
    errorRate = (error - lastError) / dt;

    // Turn PID Compensation
    kpAdjustment = LimelightConstants.kpAim0 * error;
    kiAdjustment = LimelightConstants.kiAim0 * sumError;
    kdAdjustment = LimelightConstants.kdAim0 * errorRate;

    correctedLeftMotorSpeed = -kpAdjustment - kiAdjustment - kdAdjustment;
    correctedRightMotorSpeed = kpAdjustment + kiAdjustment + kdAdjustment;
    
    motorTurnSpeedRatio = Math.abs(correctedLeftMotorSpeed / correctedRightMotorSpeed);

    // Set maximum drive speed whilst maintaining turn ratio
    if (correctedLeftMotorSpeed > LimelightConstants.maxLimelightTurnSpeed){
      correctedLeftMotorSpeed = LimelightConstants.maxLimelightTurnSpeed;
      motorsMeetThresholdSpeed = true;
    } else if (correctedLeftMotorSpeed < -LimelightConstants.maxLimelightTurnSpeed){
      correctedLeftMotorSpeed = -LimelightConstants.maxLimelightTurnSpeed;
      motorsMeetThresholdSpeed = true;
    } else{
      motorsMeetThresholdSpeed = false;
    }
    
    if (correctedRightMotorSpeed > LimelightConstants.maxLimelightTurnSpeed){
      correctedRightMotorSpeed = LimelightConstants.maxLimelightTurnSpeed;
      motorsMeetThresholdSpeed = true;
    } else if (correctedRightMotorSpeed < -LimelightConstants.maxLimelightTurnSpeed){
      correctedRightMotorSpeed = -LimelightConstants.maxLimelightTurnSpeed;
      motorsMeetThresholdSpeed = true;
    } else{
      motorsMeetThresholdSpeed = false;
    }

    if (motorsMeetThresholdSpeed == true){
      correctedRightMotorSpeed = correctedRightMotorSpeed / motorTurnSpeedRatio;
    }
    
    drivetrainSubsystem.drive(correctedLeftMotorSpeed, correctedRightMotorSpeed);

    lastTimestamp = Timer.getFPGATimestamp();
    lastError = error;

    if (Math.abs(error) < 0.25){
      counter++;
    } else{
      counter = 0;
    }
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stopDrive();
    System.out.println("Limelight Heading Command Interrupted");
  }

  @Override
  public boolean isFinished() {
    return (counter > 50);
  }
}
