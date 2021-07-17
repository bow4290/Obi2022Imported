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

public class LimelightDriveToDistanceCommand extends CommandBase {
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
  
  public LimelightDriveToDistanceCommand(DrivetrainSubsystem drivetrainSubsystem, Limelight limelight) {
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
    double kpAdjustment = LimelightConstants.kpDistance * error;
    double kiAdjustment = LimelightConstants.kiDistance * sumError;
    double kdAdjustment = LimelightConstants.kdDistance * errorRate;

    correctedLeftMotorSpeed = LimelightConstants.limelightDriveSpeed + kpAdjustment + kiAdjustment + kdAdjustment;
    correctedRightMotorSpeed = LimelightConstants.limelightDriveSpeed + kpAdjustment + kiAdjustment + kdAdjustment;

    drivetrainSubsystem.drive(correctedLeftMotorSpeed, correctedRightMotorSpeed);

    lastTimestamp = Timer.getFPGATimestamp();
    lastError = error;
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stopDrive();
    System.out.println("Limelight Distance Command Interrupted");
  }

  @Override
  public boolean isFinished() {
    return ((Math.abs(limelight.getYError()) < 0.2) && (Math.abs(errorRate) < 0.5));
  }
}
