/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoDriveConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoDriveDistanceCommand extends CommandBase {
  private final DrivetrainSubsystem drivetrainSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private double inchesToDrive;
  private double straightnessError = 0;
  private double straightnessCorrection = 0;
  private double distanceError = 0;
  private double distanceCorrection = 0;
  private double distanceSum = 0;
  private double dt = 0;
  private double distanceErrorRate = 0;
  private double lastDistanceError = 0;
  private double lastTimestamp = 0;
  private double motorSpeedRatio = 0;
  private double correctedLeftMotorSpeed = 0;
  private double correctedRightMotorSpeed = 0;
  private double kpAdjustment = 0;
  private double kiAdjustment = 0;
  private double kdAdjustment = 0;

  public AutoDriveDistanceCommand(DrivetrainSubsystem drivetrainSubsystem, IntakeSubsystem intakeSubsystem, double inchesToDrive) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.inchesToDrive = inchesToDrive;
  }

  @Override
  public void initialize() {
    System.out.println("Running Auto Distance Command");  
    drivetrainSubsystem.resetEncoders();
    drivetrainSubsystem.resetGyro();
    lastTimestamp = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {
    intakeSubsystem.intakeIn(1);

    // Straightness PID calculations
    straightnessError = drivetrainSubsystem.getGyroAngle();
    
    // Add straightness correction when more than X inches away from target distance
    if (Math.abs(distanceError) >= AutoDriveConstants.autoDistanceIntegralWindow) {
      straightnessCorrection = AutoDriveConstants.straightkP * straightnessError;
    } else {
      straightnessCorrection = 0;
    }

    // Distance PID calculations
    dt = Timer.getFPGATimestamp() - lastTimestamp;
    distanceError = inchesToDrive - drivetrainSubsystem.getRightEncoder();
    SmartDashboard.putNumber("Distance Error: ", distanceError);

    // Integral Gain
    if (Math.abs(distanceError) < AutoDriveConstants.autoDistanceIntegralWindow) {
        distanceSum += distanceError * dt;
    } else {
        distanceSum = 0;
    }

    // Derivative Gain
    distanceErrorRate = (distanceError - lastDistanceError) / dt;

    // PID speed corrections
    kpAdjustment = AutoDriveConstants.distancekP * distanceError;
    kiAdjustment = AutoDriveConstants.distancekI * distanceSum;
    kdAdjustment = AutoDriveConstants.distancekD * distanceErrorRate;

    distanceCorrection = kpAdjustment + kiAdjustment + kdAdjustment;

    correctedLeftMotorSpeed = distanceCorrection - straightnessCorrection;
    correctedRightMotorSpeed = distanceCorrection + straightnessCorrection;
    motorSpeedRatio = correctedLeftMotorSpeed / correctedRightMotorSpeed;

    // Saturate motor speed to auto speed
    if (correctedLeftMotorSpeed > AutoDriveConstants.autoDriveSpeed) {
        correctedLeftMotorSpeed = AutoDriveConstants.autoDriveSpeed;
    } else if (correctedLeftMotorSpeed < -AutoDriveConstants.autoDriveSpeed) {
        correctedLeftMotorSpeed = -AutoDriveConstants.autoDriveSpeed;
    }

    if (correctedRightMotorSpeed > AutoDriveConstants.autoDriveSpeed) {
        correctedRightMotorSpeed = AutoDriveConstants.autoDriveSpeed;
    } else if (correctedRightMotorSpeed < -AutoDriveConstants.autoDriveSpeed) {
        correctedRightMotorSpeed = -AutoDriveConstants.autoDriveSpeed;
    }

    //Maintains speed ratio if motor speeds are saturated
    correctedRightMotorSpeed = correctedRightMotorSpeed / motorSpeedRatio;

//    drivetrainSubsystem.drive(correctedLeftMotorSpeed, correctedRightMotorSpeed);

    lastTimestamp = Timer.getFPGATimestamp();
    lastDistanceError = distanceError;
    }

    @Override
    public boolean isFinished() { //Finishes when our error is +- 5 inches and the derivative is low.
        return (((-0.001 <= AutoDriveConstants.distancekD * (distanceErrorRate)
                && 0.001 >= AutoDriveConstants.distancekD * (distanceErrorRate))
                && (inchesToDrive - 5) <= drivetrainSubsystem.getRightEncoder()
                && (inchesToDrive + 5) >= drivetrainSubsystem.getRightEncoder()));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.stopDrive();
        intakeSubsystem.intakeIn(0);
        System.out.println("Done with auto drive command.");
    }
}
