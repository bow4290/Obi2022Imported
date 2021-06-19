/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import frc.robot.Constants.DriveConstants;

public class DrivetrainSubsystem extends SubsystemBase {

  private WPI_VictorSPX leftVictorSPX1, leftVictorSPX2, leftVictorSPX3, rightVictorSPX1, rightVictorSPX2, rightVictorSPX3;
  private DifferentialDrive m_drive;
  
  public Encoder driveTrainLeftEncoder;
  public Encoder driveTrainRightEncoder;
  public ADXRS450_Gyro driveGyro;

  private final DoubleSolenoid gearShiftSolenoid;

  public enum GearShiftStatus{
    HIGH, LOW
  }
  public static GearShiftStatus gearShiftStatus;

  public DrivetrainSubsystem() {
    leftVictorSPX1 = new WPI_VictorSPX(DriveConstants.leftVictorSPX1Channel);
    leftVictorSPX2 = new WPI_VictorSPX(DriveConstants.leftVictorSPX2Channel);
    leftVictorSPX3 = new WPI_VictorSPX(DriveConstants.leftVictorSPX3Channel);
    rightVictorSPX1 = new WPI_VictorSPX(DriveConstants.rightVictorSPX1Channel);
    rightVictorSPX2 = new WPI_VictorSPX(DriveConstants.rightVictorSPX2Channel);
    rightVictorSPX3 = new WPI_VictorSPX(DriveConstants.rightVictorSPX3Channel);

    leftVictorSPX1.setInverted(false);
    rightVictorSPX1.setInverted(false);

    leftVictorSPX2.follow(leftVictorSPX1);
    leftVictorSPX2.setInverted(InvertType.FollowMaster);
    leftVictorSPX3.follow(leftVictorSPX2);
    leftVictorSPX3.setInverted(InvertType.FollowMaster);

    rightVictorSPX2.follow(rightVictorSPX1);
    rightVictorSPX2.setInverted(InvertType.FollowMaster);
    rightVictorSPX3.follow(rightVictorSPX2);
    rightVictorSPX3.setInverted(InvertType.FollowMaster);

    // Using Speed Controller Groups caused output to motors to not update quick enough.
    //SpeedControllerGroup m_left = new SpeedControllerGroup(leftVictorSPX1, leftVictorSPX2, leftVictorSPX3);
    //SpeedControllerGroup m_right = new SpeedControllerGroup(rightVictorSPX1, rightVictorSPX2, rightVictorSPX3);

    //m_left.setInverted(true);
    //m_right.setInverted(true);

    m_drive = new DifferentialDrive(leftVictorSPX1, rightVictorSPX1);
    m_drive.setDeadband(0.05);
    m_drive.setMaxOutput(DriveConstants.driveMaxSpeed);

    driveTrainLeftEncoder = new Encoder(DriveConstants.driveTrainLeftEncoderChannelA, DriveConstants.driveTrainLeftEncoderChannelB, false, CounterBase.EncodingType.k4X);
    driveTrainLeftEncoder.setSamplesToAverage(DriveConstants.driveTrainLeftEncoderAverageSamples);
    driveTrainLeftEncoder.setMinRate(DriveConstants.driveTrainLeftEncoderMinRate);
    driveTrainLeftEncoder.setDistancePerPulse(DriveConstants.driveTrainLeftEncoderPulseDistance);

    driveTrainRightEncoder = new Encoder(DriveConstants.driveTrainRightEncoderChannelA, DriveConstants.driveTrainRightEncoderChannelB, true, CounterBase.EncodingType.k4X);
    driveTrainRightEncoder.setSamplesToAverage(DriveConstants.driveTrainRightEncoderAverageSamples);
    driveTrainRightEncoder.setMinRate(DriveConstants.driveTrainRightEncoderMinRate);
    driveTrainRightEncoder.setDistancePerPulse(DriveConstants.driveTrainRightEncoderPulseDistance);

    driveGyro = new ADXRS450_Gyro();

    gearShiftSolenoid = new DoubleSolenoid(DriveConstants.gearShiftHighChannel, DriveConstants.gearShiftLowChannel);
    gearShiftStatus = GearShiftStatus.LOW;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void drive(double leftSpeed, double rightSpeed){
    leftSpeed = leftSpeed * DriveConstants.driveSpeedMultiplier;
    rightSpeed = rightSpeed * DriveConstants.driveSpeedMultiplier;
    m_drive.tankDrive(leftSpeed, rightSpeed);
  }

  public void stopDrive(){
    m_drive.tankDrive(0, 0);
  }

  public void resetEncoders() {
    driveTrainLeftEncoder.reset();
    driveTrainRightEncoder.reset();
  }

  public double getLeftEncoder(){
    return(driveTrainLeftEncoder.getDistance());
  }

  public double getRightEncoder(){
    return(driveTrainRightEncoder.getDistance());
  }

  public double getAverageEncoder(){
    return((getLeftEncoder() + getRightEncoder()) / 2);
  }

  public void resetGyro(){
    driveGyro.reset();
  }

  public double getGyroAngle(){
    return(driveGyro.getAngle());
  }

  public void shiftDown(){
    gearShiftSolenoid.set(DoubleSolenoid.Value.kForward);   // kForward is Low (down) speed
    gearShiftStatus = GearShiftStatus.LOW;
  }

  public void shiftUp(){
    gearShiftSolenoid.set(DoubleSolenoid.Value.kReverse);   // kReverse is High (up) speed
    gearShiftStatus = GearShiftStatus.HIGH;
  }

  public static GearShiftStatus getGearShiftPosition(){
    return gearShiftStatus;
  }

}
