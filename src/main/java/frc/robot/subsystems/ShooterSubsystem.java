/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class ShooterSubsystem extends SubsystemBase {
  
  private final WPI_VictorSPX leftShooterMotor;
  private final WPI_VictorSPX rightShooterMotor;
  private final DoubleSolenoid shooterSolenoid;
  private double targetShooterSpeed = 0.0;
  private double targetShooterRate = 0.0;

  public enum ShooterStatus{
    DOWN, UP
  }
  public static ShooterStatus shooterStatus;

  public ShooterSubsystem() {
    leftShooterMotor = new WPI_VictorSPX(ShooterConstants.leftShooterMotorChannel);
    rightShooterMotor = new WPI_VictorSPX(ShooterConstants.rightShooterMotorChannel);

    leftShooterMotor.setInverted(false);
    rightShooterMotor.setInverted(false);

    shooterSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, ShooterConstants.shooterUpChannel, ShooterConstants.shooterDownChannel);
    shooterStatus = ShooterStatus.DOWN;
  }

  public void shooterShoot(double shooterSpeed){
    leftShooterMotor.set(ControlMode.PercentOutput, shooterSpeed);
    rightShooterMotor.set(ControlMode.PercentOutput, shooterSpeed * ShooterConstants.shooterSpeedOffset);
    SmartDashboard.putNumber("Shooter Command Speed: ", shooterSpeed);
  }

  public void shooterStop(){
    leftShooterMotor.set(ControlMode.PercentOutput, 0);
    rightShooterMotor.set(ControlMode.PercentOutput, 0);
  }

  public void setTargetShooterSpeed(double targetShooterSpeed){
    this.targetShooterSpeed = targetShooterSpeed;
  }

  public double getTargetShooterSpeed(){
    return targetShooterSpeed;
  }

  public void setTargetShooterRate(double targetShooterRate){
    this.targetShooterRate = targetShooterRate;
  }

  public double getTargetShooterRate(){
    return targetShooterRate;
  }

  public void shooterDown(){
    shooterSolenoid.set(DoubleSolenoid.Value.kForward);
    shooterStatus = ShooterStatus.DOWN;
  }

  public void shooterUp(){
    shooterSolenoid.set(DoubleSolenoid.Value.kReverse);
    shooterStatus = ShooterStatus.UP;
  }

  public static ShooterStatus getShooterPosition(){
    return shooterStatus;
  }

  @Override
  public void periodic() {
    
  }
}
