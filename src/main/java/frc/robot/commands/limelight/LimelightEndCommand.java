/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.sensors.Limelight.*;
import frc.robot.sensors.Limelight;

public class LimelightEndCommand extends CommandBase {
  private final Limelight limelight;
  
  public LimelightEndCommand() {
    limelight = new Limelight();
  }

  @Override
  public void initialize() {
    System.out.println("Running Limelight End Command");
    limelight.setLedMode(LedMode.ledOff);

  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("Ready to fire: ", true);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
  
}
