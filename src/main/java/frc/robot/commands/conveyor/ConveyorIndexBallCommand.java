/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.conveyor;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.subsystems.ConveyorSubsystem;

public class ConveyorIndexBallCommand extends CommandBase {
  private final ConveyorSubsystem conveyorSubsystem;

  public ConveyorIndexBallCommand(ConveyorSubsystem conveyorSubsystem) {
    this.conveyorSubsystem = conveyorSubsystem;
    addRequirements(conveyorSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if((conveyorSubsystem.getButton1() == false) || (conveyorSubsystem.getButton2() == false)){       // Buttons are wired NC, so returns false when button is pressed
    conveyorSubsystem.conveyBall(ConveyorConstants.conveyorIndexBallSpeed);                           // If one of the intake buttons is pressed, convey the balls
    } else{
      conveyorSubsystem.conveyorStop();
    }
  }

  @Override
  public void end(boolean interrupted) {
    conveyorSubsystem.conveyorStop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
