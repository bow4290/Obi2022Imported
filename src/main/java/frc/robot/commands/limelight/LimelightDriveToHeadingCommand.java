/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants.LimelightConstants;
import frc.robot.sensors.Limelight;
import frc.robot.subsystems.DrivetrainSubsystem;

public class LimelightDriveToHeadingCommand extends CommandBase {
  private final DrivetrainSubsystem drivetrainSubsystem;
  private final Limelight limelight;
  private PIDController pid;
  
  public LimelightDriveToHeadingCommand(DrivetrainSubsystem drivetrainSubsystem, Limelight limelight) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.limelight = limelight;
    pid = new PIDController(LimelightConstants.kpAim0, LimelightConstants.kiAim0, LimelightConstants.kdAim0);
    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    System.out.println("Running Limelight Heading Command");
    pid.setTolerance(LimelightConstants.headingPositionTolerance, LimelightConstants.headingVelocityTolerance);
  }

  @Override
  public void execute() {
    double motorSpeed = pid.calculate(limelight.getXError(), 0.0);
    SmartDashboard.putNumber("Limelight Position X Error", pid.getPositionError());
    SmartDashboard.putNumber("Limelight Velocity X Error", pid.getVelocityError());

    // Don't exceed motor thresholds
    motorSpeed = MathUtil.clamp(motorSpeed, -LimelightConstants.maxLimelightTurnSpeed, LimelightConstants.maxLimelightTurnSpeed);
    
    drivetrainSubsystem.drive(motorSpeed, -motorSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stopDrive();
    System.out.println("Limelight Heading Command Interrupted");
  }

  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }
}
