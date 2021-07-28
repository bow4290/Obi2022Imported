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
  
  public LimelightInitCommand() {
    limelight = new Limelight();
  }

  @Override
  public void initialize() {
    System.out.println("Running Limelight Init Command");
    SmartDashboard.putBoolean("Ready to fire: ", false);
    limelight.setLedMode(LedMode.ledOn);
    limelight.setCamMode(CamMode.vision);
  }
  
  @Override
  public void execute(){
    limelight.getTarget();
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {  
      return limelight.isTarget();
  }
  
}
