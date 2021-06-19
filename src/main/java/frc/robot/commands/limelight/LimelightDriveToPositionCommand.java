/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.sensors.Limelight;
import frc.robot.sensors.Limelight.*;
import frc.robot.subsystems.DrivetrainSubsystem;

public class LimelightDriveToPositionCommand extends CommandBase {

  private final DrivetrainSubsystem drivetrainSubsystem;
  private final Limelight limelight;
  private int pipeline;
  private double xError = 0;
  private double yError = 0;
  private double target = 0;
  private double distanceError = 0;
  private double headingError = 0;
  private double speedAdjustment = 0;
  private double turnAdjustment = 0;
  private double correctedLeftMotorSpeed = 0;
  private double correctedRightMotorSpeed = 0;

  public LimelightDriveToPositionCommand(int pipeline, DrivetrainSubsystem drivetrainSubsystem) {
    this.pipeline = pipeline;
    this.drivetrainSubsystem = drivetrainSubsystem;
    limelight = new Limelight();
    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    limelight.setLedMode(LedMode.ledOn);
    limelight.setCamMode(CamMode.vision);
    limelight.setPipeline(pipeline);
  }

  @Override
  public void execute() {
    System.out.println("Running Limelight Command");
    target = limelight.getTarget();

    if(target == 1){                    // If we have a valid target, do stuff
      xError = limelight.getXError();
      yError = limelight.getYError();
      SmartDashboard.putNumber("Limelight X Error: ", xError);
      SmartDashboard.putNumber("Limelight Y Error: ", yError);

      distanceError = -yError;          // Distance is directly proportional to the Y (vertical) error
      speedAdjustment = LimelightConstants.kpDistance*distanceError;

      headingError = -xError;
      turnAdjustment = LimelightConstants.kpAim*headingError;

      correctedLeftMotorSpeed = LimelightConstants.limelightDriveSpeed + speedAdjustment - turnAdjustment;
      System.out.println(correctedLeftMotorSpeed);
      correctedRightMotorSpeed = LimelightConstants.limelightDriveSpeed + speedAdjustment + turnAdjustment;
      System.out.println(correctedRightMotorSpeed);

      drivetrainSubsystem.drive(correctedLeftMotorSpeed, correctedRightMotorSpeed);
    }
  }

  @Override
  public void end(boolean interrupted) {
    limelight.setLedMode(LedMode.ledOff);
    System.out.println("Limelight Command Interrupted");
  }

  @Override
  public boolean isFinished() {
    return false;               // Should finish when xError and yError are "good enough"?
  }
}
