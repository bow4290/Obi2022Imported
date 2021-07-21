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
  private double error = 0;
  private double sumError = 0;
  private double correctedLeftMotorSpeed = 0;
  private double correctedRightMotorSpeed = 0;
  
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

    // Turn PID Compensation
    double kpAdjustment = LimelightConstants.kpAim * error;
    double kiAdjustment = LimelightConstants.kiAim * sumError;

    correctedLeftMotorSpeed = LimelightConstants.limelightDriveSpeed - kpAdjustment - kiAdjustment;
    correctedRightMotorSpeed = LimelightConstants.limelightDriveSpeed + kpAdjustment + kiAdjustment;

    drivetrainSubsystem.drive(correctedLeftMotorSpeed, correctedRightMotorSpeed);

    lastTimestamp = Timer.getFPGATimestamp();
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stopDrive();
    System.out.println("Limelight Heading Command Interrupted");
  }

  @Override
  public boolean isFinished() {
    return ((Math.abs(limelight.getXError()) < 0.2) && (Math.abs(sumError) < 0.5));
  }
}
