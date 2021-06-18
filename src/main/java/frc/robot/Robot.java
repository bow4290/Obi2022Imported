/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.conveyor.ConveyorIndexBallCommand;
import frc.robot.subsystems.ConveyorSubsystem;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private ConveyorSubsystem conveyorSubsystem;
  public static CameraServer server;
  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
    camera.setResolution(160, 120);
    m_robotContainer.drivetrainSubsystem.driveGyro.calibrate();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putBoolean("Button Value 1: ", m_robotContainer.conveyorSubsystem.getButton1());
    SmartDashboard.putBoolean("Button Value 2: ", m_robotContainer.conveyorSubsystem.getButton2());
    SmartDashboard.putNumber("Gyro Angle: ", m_robotContainer.drivetrainSubsystem.getGyroAngle());
    SmartDashboard.putNumber("Left Drive Encoder: ", m_robotContainer.drivetrainSubsystem.getLeftEncoder());
    SmartDashboard.putNumber("Right Drive Encoder: ", m_robotContainer.drivetrainSubsystem.getRightEncoder());

    if((m_robotContainer.conveyorSubsystem.getButton1() == false) || (m_robotContainer.conveyorSubsystem.getButton2() == false)){
      new ConveyorIndexBallCommand(conveyorSubsystem);
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
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    m_robotContainer.drivetrainSubsystem.resetGyro();
    m_robotContainer.drivetrainSubsystem.resetEncoders();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
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
