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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class ClimberSubsystem extends SubsystemBase {
  
  private final WPI_VictorSPX climberMotor;
  private final DoubleSolenoid climberSolenoid;

  public enum ClimberStatus{
    DOWN, UP
  }
  public static ClimberStatus climberStatus;

  public ClimberSubsystem() {
    climberMotor = new WPI_VictorSPX(ClimberConstants.climberMotorChannel);
    climberSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, ClimberConstants.climberUpChannel, ClimberConstants.climberDownChannel);
  
    climberMotor.setInverted(false);
    climberStatus = ClimberStatus.DOWN;
  }

  public void extendClimber(){
    climberSolenoid.set(DoubleSolenoid.Value.kForward);
    climberStatus = ClimberStatus.UP;
  }

  public void retractClimber(){
    climberSolenoid.set(DoubleSolenoid.Value.kReverse);
    climberStatus = ClimberStatus.DOWN;
  }

  public void climb(double climberSpeed){
    climberMotor.set(ControlMode.PercentOutput, climberSpeed);
  }

  public void climberStop(){
    climberMotor.set(ControlMode.PercentOutput, 0);
  }

  public static ClimberStatus getClimberPosition(){
    return climberStatus;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
