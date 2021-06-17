/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.RandomConstants;
import frc.robot.commands.conveyor.ConveyorIndexBallCommand;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private ConveyorSubsystem conveyorSubsystem;
  private DrivetrainSubsystem drivetrainSubsystem;
  private Compressor compressor;
  public static CameraServer server;
  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
    camera.setResolution(160, 120);
    compressor = new Compressor(RandomConstants.compressorCANID);
    drivetrainSubsystem.driveGyro.calibrate();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putBoolean("Button Value 1: ", conveyorSubsystem.getButton1());
    SmartDashboard.putBoolean("Button Value 2: ", conveyorSubsystem.getButton2());

    if((conveyorSubsystem.getButton1() == false) || (conveyorSubsystem.getButton2() == false)){
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
