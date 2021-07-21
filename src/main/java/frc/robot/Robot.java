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
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    
    switch(robotContainer.getDPad()){
      case 0:
        robotContainer.setShooterPosition(0);
        break;
      case 90:
        robotContainer.setShooterPosition(1);
        break;
      case 180:
        robotContainer.setShooterPosition(2);
        break;
      case 270:
        robotContainer.setShooterPosition(3);
        break;
      default:
        break;
    }

    SmartDashboard.putNumber("Shooting Position: ", robotContainer.getShooterPosition());

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
