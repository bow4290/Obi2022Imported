/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

public final class Constants {

    public static final class DriveConstants{
        public static final int leftVictorSPX1Channel = 5;
        public static final int leftVictorSPX2Channel = 3;
        public static final int leftVictorSPX3Channel = 6;
        public static final int rightVictorSPX1Channel = 12;
        public static final int rightVictorSPX2Channel = 11;
        public static final int rightVictorSPX3Channel = 10;

        public static final double driveSpeedMultiplier = 0.75;
        public static final double driveMaxSpeed = 1;

        private static final double wheelDiameter = 7;
    
        public static final int driveTrainLeftEncoderChannelA = 4;
        public static final int driveTrainLeftEncoderChannelB = 5;
        public static final int driveTrainRightEncoderChannelA = 2;
        public static final int driveTrainRightEncoderChannelB = 3;

        public static final int driveTrainLeftEncoderAverageSamples = 5;
        public static final double driveTrainLeftEncoderMinRate = 1;
        public static final double driveTrainLeftEncoderPulseDistance = 1.0 / 8800 * Math.PI * wheelDiameter;
        public static final int driveTrainRightEncoderAverageSamples = 5;
        public static final double driveTrainRightEncoderMinRate = 1;
        public static final double driveTrainRightEncoderPulseDistance = 1.0 / 8800 * Math.PI * wheelDiameter;
    
        public static final int gearShiftHighChannel = 4;
        public static final int gearShiftLowChannel = 5;
    }

    public static final class AutoDriveConstants{
        public static final double autoDriveSpeed = 0.6;
        public static final double straightkP = 0.3;
        public static final double distancekP = 0.02;
        public static final double distancekI = 0;
        public static final double distancekD = 0.005;
        public static final double autoDistanceIntegralWindow = 12;
        
        public static final double autoTurnSpeed = 0.8;
        public static final double turnkP = 0.05;
        public static final double turnkI = 0.2;
        public static final double turnkD = 0.001;
        public static final double autoTurnIntegralWindow = 5;
    }

    public static final class OIConstants{
        public static final int XBOX_CONTROLLER = 0;
    }

    public static final class IntakeConstants{
        public static final int intakeMotorChannel = 4;
        public static final int intakeUpChannel = 0;
        public static final int intakeDownChannel = 1;
        public static final double intakeSpeed = 1;
    }

    public static final class ClimberConstants{
        public static final int climberMotorChannel = 4;
        public static final int climberUpChannel = 7;
        public static final int climberDownChannel = 6;
        public static final double climberSpeed = 1;
    }

    public static final class ConveyorConstants{
        public static final int topConveyorMotorChannel = 9;
        public static final int bottomConveyorMotorChannel = 7;
        public static final int conveyorButton1Port = 6;
        public static final int conveyorButton2Port = 7;
        public static final double conveyorSpeedDivider = 0.91;
        public static final double conveyorIndexBallSpeed = 0.80;   // 0.70 old balls, 0.80 new balls
        public static final double conveyorShootBallSpeed = 1;
        public static final double conveyorReverseSpeed = -0.25;
    }

    public static final class LimelightConstants{
        public static final double h1 = 22.25;                     // Distance from ground to limelight
        public static final double h2 = 98.25;                     // Distance from ground to target
        public static final double a1 = 22.407;                  // Limelight mount angle
        public static final double Lime2BumpDistance = 16.5;
        public static final double maxLimelightDriveSpeed = 0.6;
        public static final double maxLimelightTurnSpeed = 0.6;     // Make one for auto = 0.6
        public static final double distanceIntegralWindow = 1;
        public static final double turnIntegralWindow = 5;

        // Auto Position PID Values
        public static final double kpDistanceAuto = 0.5;
        public static final double kiDistanceAuto = 0.4;
        public static final double kdDistanceAuto = 0.05;
        public static final double kpAimAuto = 0.2;
        public static final double kiAimAuto = 0.25;
        public static final double kdAimAuto = 0.01;

        // Teleop Position PID Values
        public static final double kpAimTeleop = 0.2;
        public static final double kiAimTeleop = 0.3;
        public static final double kdAimTeleop = 0.01;

        public static final double distancePositionTolerance = 0.20;
        public static final double distanceVelocityTolerance = 0.025;
        public static final double headingPositionTolerance = 0.18;
        public static final double headingVelocityTolerance = 0.025;
    }

    public static final class ShooterConstants{
        public static final int leftShooterMotorChannel = 8;
        public static final int rightShooterMotorChannel = 1;
        public static final int shooterUpChannel = 2;
        public static final int shooterDownChannel = 3;
        public static final double shooterSpeedOffset = 0.85;
        public static final int shooterEncoderChannelA = 0;
        public static final int shooterEncoderChannelB = 1;
        public static final int shooterEncoderAverageSamples = 127;
        public static final int shooterMotorToRateSlope = 240000;
        public static final int shooterMotorToRateIntercept = 25000;
        public static final double shooterSpeedDefault = 0.95;
        public static final double shooterSpeedAuto = 0.95;

        public static final double minimumShooterDistance = 140.0;
        public static final double thresholdShooterDistance = 225.0;
        public static final double maximumShooterDistance = 335.0;

        public static final double shooterSpeedClose = 0.86;    // 0.95 old balls, 0.86 new balls
        public static final double shooterSpeedFar = 0.82;      // 0.85 old balls, 0.82 new balls
    }

}
