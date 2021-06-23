/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.Constants.ShooterConstants;

public class ConveyorSubsystem extends SubsystemBase {
  
  private final WPI_VictorSPX topConveyorMotor;
  private final WPI_VictorSPX bottomConveyorMotor;
  
  private static DigitalInput conveyorButton1;
  private static DigitalInput conveyorButton2;

  private static BooleanSupplier conveyorCmdSelector;

  public Encoder shooterEncoder;

  public ConveyorSubsystem() {
    topConveyorMotor = new WPI_VictorSPX(ConveyorConstants.topConveyorMotorChannel);
    bottomConveyorMotor = new WPI_VictorSPX(ConveyorConstants.bottomConveyorMotorChannel);
    topConveyorMotor.setInverted(true);

    conveyorButton1 = new DigitalInput(ConveyorConstants.conveyorButton1Port);
    conveyorButton2 = new DigitalInput(ConveyorConstants.conveyorButton2Port);
  
    shooterEncoder = new Encoder(ShooterConstants.shooterEncoderChannelA, ShooterConstants.shooterEncoderChannelB, true, CounterBase.EncodingType.k4X);
    shooterEncoder.setSamplesToAverage(ShooterConstants.shooterEncoderAverageSamples);

    conveyorCmdSelector = new BooleanSupplier(){
      @Override
      public boolean getAsBoolean() {        
        System.out.println("I am updating the button boolean");
        return ((getButton1() == false) || (getButton2() == false));
      }
    };
  }

  public void conveyBall(double conveyorSpeed){
    topConveyorMotor.set(ControlMode.PercentOutput, conveyorSpeed);
    bottomConveyorMotor.set(ControlMode.PercentOutput, conveyorSpeed/ConveyorConstants.conveyorSpeedDivider);
  }

  public void reverseConveyBall(){
    conveyBall(ConveyorConstants.conveyorReverseSpeed);
  }

  public void conveyorStop(){
    conveyBall(0);
  }

  public boolean getButton1(){
    return conveyorButton1.get();
  }

  public boolean getButton2(){
    return conveyorButton2.get();
  }

  public double getEncoderRate(){
    return(shooterEncoder.getRate());
  }

  public BooleanSupplier getconveyorCmdSelector(){
    return conveyorCmdSelector;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Button Value 1: ", getButton1());
    SmartDashboard.putBoolean("Button Value 2: ", getButton2());
    SmartDashboard.putNumber("Shooter Encoder End: ", getEncoderRate());
  }
}
