/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.conveyor;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ConveyorSubsystem;

public class ConveyorShootBallCommand extends CommandBase {
  private final ConveyorSubsystem conveyorSubsystem;
  private final int limelightShootingPosition;
  private double targetShooterRate = 0;
  private double shootSpeed = 0;

  public ConveyorShootBallCommand(ConveyorSubsystem conveyorSubsystem, int LimelightShootingPosition) {
    this.conveyorSubsystem = conveyorSubsystem;
    this.limelightShootingPosition = LimelightShootingPosition;
    addRequirements(conveyorSubsystem);
  }

  @Override
  public void initialize() {
    setTargetShooterSpeed();
  }

  @Override
  public void execute() {
    if(conveyorSubsystem.getEncoderRate() >= (targetShooterRate - 2000)){
      conveyorSubsystem.conveyBall(ConveyorConstants.conveyorShootBallSpeed);
    } else{
      conveyorSubsystem.conveyorStop();
    }
  }

  private void setTargetShooterSpeed(){
    switch(limelightShootingPosition){
      case 0:     // Trench Front
        this.shootSpeed = ShooterConstants.shootSpeedPosition0;
        break;
      case 1:     // Trench Back
        this.shootSpeed = ShooterConstants.shootSpeedPosition1;
        break;
      case 2:
        this.shootSpeed = ShooterConstants.shootSpeedPosition2;
        break;
      case 3:     // Auto start line
        this.shootSpeed = ShooterConstants.shootSpeedPosition3;
        break;
    }
    targetShooterRate = (ShooterConstants.shooterMotorToRateSlope * this.shootSpeed) - ShooterConstants.shooterMotorToRateIntercept;
  }

  public double getTargetShooterSpeed(){
    return this.shootSpeed;
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