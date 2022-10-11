/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.limelight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.LimelightConstants;
import frc.robot.sensors.Limelight;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.sensors.PIDParams;

public class LimelightDriveToDistanceCommand extends CommandBase {
  private final DrivetrainSubsystem drivetrainSubsystem;
  private final Limelight limelight;
  private PIDController pid;
  private PIDParams params;
  
  public LimelightDriveToDistanceCommand(DrivetrainSubsystem drivetrainSubsystem, Limelight limelight, PIDParams params) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.limelight = limelight;
    this.params = params;
    pid = new PIDController(params.getKp(), params.getKi(), params.getKd());
    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    System.out.println("Running Limelight Distance Command");
    pid.setTolerance(params.getPosTolerance(), params.getVelTolerance());
  }

  @Override
  public void execute() {
    double motorSpeed = pid.calculate(limelight.getYError(), params.getSetpoint());
    SmartDashboard.putNumber("Limelight Position Y Error: ", pid.getPositionError());
    SmartDashboard.putNumber("Limelight Velocity Y Error: ", pid.getVelocityError());

    // Don't exceed motor thresholds
    motorSpeed = MathUtil.clamp(motorSpeed, -LimelightConstants.maxLimelightDriveSpeed, LimelightConstants.maxLimelightDriveSpeed);

    drivetrainSubsystem.drive(motorSpeed, motorSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stopDrive();
    System.out.println("Limelight Distance Command Interrupted");
  }

  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }
}
