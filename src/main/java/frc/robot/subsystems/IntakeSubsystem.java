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
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class IntakeSubsystem extends SubsystemBase {
  
  private final WPI_VictorSPX intakeMotor;
  private final DoubleSolenoid intakeSolenoid;

  public enum IntakeStatus{
    DOWN, UP
  }
  public static IntakeStatus intakeStatus;

  public IntakeSubsystem() {
    intakeMotor = new WPI_VictorSPX(IntakeConstants.intakeMotorChannel);
    intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, IntakeConstants.intakeUpChannel, IntakeConstants.intakeDownChannel);
  
    intakeMotor.setInverted(false);
    intakeStatus = IntakeStatus.UP;
  }

  public void retractIntake(){
    intakeSolenoid.set(DoubleSolenoid.Value.kForward);
    intakeStatus = IntakeStatus.UP;
  }

  public void extendIntake(){
    intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
    intakeStatus = IntakeStatus.DOWN;
  }

  public void intakeIn(double intakeSpeed){
    intakeMotor.set(ControlMode.PercentOutput, intakeSpeed);
  }

  public void intakeStop(){
    intakeMotor.set(ControlMode.PercentOutput, 0);
  }

  public static IntakeStatus getIntakePosition(){
    return intakeStatus;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
