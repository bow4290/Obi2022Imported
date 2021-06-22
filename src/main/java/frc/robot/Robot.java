/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.conveyor.ConveyorIndexBallCommand;

public class Robot extends TimedRobot {
  private Command autonomousCommand;
  public static CameraServer server;
  private RobotContainer robotContainer;

  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();

    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
    camera.setVideoMode(VideoMode.PixelFormat.kMJPEG, 200, 150, 30);
    //camera.setExposureManual(10);
    //camera.setWhiteBalanceManual(50);

    robotContainer.drivetrainSubsystem.driveGyro.calibrate();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    
    SmartDashboard.putBoolean("Button Value 1: ", robotContainer.conveyorSubsystem.getButton1());
    SmartDashboard.putBoolean("Button Value 2: ", robotContainer.conveyorSubsystem.getButton2());
    SmartDashboard.putNumber("Gyro Angle: ", robotContainer.drivetrainSubsystem.getGyroAngle());
    SmartDashboard.putNumber("Left Drive Encoder: ", robotContainer.drivetrainSubsystem.getLeftEncoder());
    SmartDashboard.putNumber("Right Drive Encoder: ", robotContainer.drivetrainSubsystem.getRightEncoder());
    SmartDashboard.putNumber("Shooter Encoder End: ", robotContainer.conveyorSubsystem.getEncoderRate());

    if((robotContainer.conveyorSubsystem.getButton1() == false) || (robotContainer.conveyorSubsystem.getButton2() == false)){
      new ConveyorIndexBallCommand(robotContainer.conveyorSubsystem).execute();
    }
    else {
      new ConveyorIndexBallCommand(robotContainer.conveyorSubsystem).end(true);
    }
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();
    
    robotContainer.drivetrainSubsystem.resetGyro();
    robotContainer.drivetrainSubsystem.resetEncoders();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }
}
