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
import frc.robot.Constants.AutoDriveConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoTurnAngleCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrainSubsystem;
    private double degreesToTurn;
    private double speedToDriveLeft = 0;
    private double speedToDriveRight = 0;
    private double turnError = 0;
    private double turnSum = 0;
    private double turnRange = 5;
    private double turnErrorRate = 0;
    private double lastTurnError = 0;
    private double lastTimestamp = 0;
    private double dt = 0;
    private double turnCorrection = 0;
    private double kpAdjustment = 0;
    private double kiAdjustment = 0;
    private double kdAdjustment = 0;

    public AutoTurnAngleCommand(DrivetrainSubsystem drivetrainSubsystem, double degreesToTurn) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.degreesToTurn = degreesToTurn;
    }

    @Override
    public void initialize() {
        System.out.println("Running Auto Turn Command");
        drivetrainSubsystem.resetEncoders();
        drivetrainSubsystem.resetGyro();
        lastTimestamp = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        // Turn PID calculations
        dt = Timer.getFPGATimestamp() - lastTimestamp;
        turnError = degreesToTurn - drivetrainSubsystem.getGyroAngle();
        SmartDashboard.putNumber("Turn Error: ", turnError);

        // Integral Gain
        if (Math.abs(turnError) < AutoDriveConstants.autoTurnIntegralWindow) {
            turnSum += turnError * dt;
        } else {
            turnSum = 0;
        }

        // Derivative Gain
        turnErrorRate = (turnError - lastTurnError) / dt;
        
        // Turn speed corrections
        kpAdjustment = AutoDriveConstants.turnkP * turnError;
        kiAdjustment = AutoDriveConstants.turnkI * turnSum;
        kdAdjustment = AutoDriveConstants.turnkD * turnErrorRate;

        turnCorrection = kpAdjustment + kiAdjustment + kdAdjustment;
        
        speedToDriveLeft = -turnCorrection;
        speedToDriveRight = turnCorrection;

        // Saturate motor speed to auto speed
        if (speedToDriveLeft > AutoDriveConstants.autoTurnSpeed) {
            speedToDriveLeft = AutoDriveConstants.autoTurnSpeed;
        } else if (speedToDriveLeft < -AutoDriveConstants.autoTurnSpeed) {
            speedToDriveLeft = -AutoDriveConstants.autoTurnSpeed;
        }

        if (speedToDriveRight > AutoDriveConstants.autoTurnSpeed) {
            speedToDriveRight = AutoDriveConstants.autoTurnSpeed;
        } else if (speedToDriveRight < -AutoDriveConstants.autoTurnSpeed) {
            speedToDriveRight = -AutoDriveConstants.autoTurnSpeed;
        }
        
        drivetrainSubsystem.drive(speedToDriveLeft, speedToDriveRight);

        lastTimestamp = Timer.getFPGATimestamp();
        lastTurnError = turnError;
    }

    @Override
    public boolean isFinished() { // Finishes when our turn error is within =-0.5 degrees and the derivative is low.
        return ((turnError <= 0.5)
                && (turnError >= -0.5)
                && (drivetrainSubsystem.getGyroRate() <= 1)
                && (drivetrainSubsystem.driveGyro.getRate() >= -1));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.stopDrive();
        System.out.println("Done with auto turn command.");
    }
}