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

public class LimelightInitCommand extends CommandBase {
  private final Limelight limelight;
  private final int pipeline;
  
  public LimelightInitCommand(int pipeline) {
    this.pipeline = pipeline;
    limelight = new Limelight();
  }

  @Override
  public void initialize() {
    System.out.println("Running Limelight Init Command");
    limelight.setLedMode(LedMode.ledOn);
    limelight.setCamMode(CamMode.vision);
    limelight.setPipeline(pipeline);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    SmartDashboard.putBoolean("Ready to fire", false);
    return true;
  }
  
}
