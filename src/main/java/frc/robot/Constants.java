/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

public final class Constants {

    public static final class DriveConstants{
        public static final int leftVictorSPX1Channel = 12;
        public static final int leftVictorSPX2Channel = 11;
        public static final int leftVictorSPX3Channel = 10;
        public static final int rightVictorSPX1Channel = 5;
        public static final int rightVictorSPX2Channel = 3;
        public static final int rightVictorSPX3Channel = 6;

        public static final double driveSpeedMultiplier = 1;
        public static final double driveMaxSpeed = 1;

        private static final double wheelDiameter = 7;
    
        public static final int driveTrainLeftEncoderChannelA = 2;
        public static final int driveTrainLeftEncoderChannelB = 3;
        public static final int driveTrainRightEncoderChannelA = 4;
        public static final int driveTrainRightEncoderChannelB = 5;

        public static final int driveTrainLeftEncoderAverageSamples = 5;
        public static final double driveTrainLeftEncoderMinRate = 1;
        public static final double driveTrainLeftEncoderPulseDistance = 1.0 / 8800 * Math.PI * wheelDiameter;
        public static final int driveTrainRightEncoderAverageSamples = 5;
        public static final double driveTrainRightEncoderMinRate = 1;
        public static final double driveTrainRightEncoderPulseDistance = 1.0 / 8800 * Math.PI * wheelDiameter;
    
        public static final int gearShiftHighChannel = 4;
        public static final int gearShiftLowChannel = 5;
    }

    public static final class OIConstants{
        public static final int LEFT_JOYSTICK = 0;
        public static final int RIGHT_JOYSTICK = 1;
        public static final int XBOX_CONTROLLER = 2;
    }

    public static final class IntakeConstants{
        public static final int intakeMotorChannel = 13;
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
        public static final double conveyorIndexBallSpeed = 0.67;
        public static final double conveyorShootBallSpeed = 1;
        public static final double conveyorReverseSpeed = -0.25;
    }

    public static final class LimelightConstants{
        public static final double h1 = 24;                     // Distance from ground to limelight
        public static final double h2 = 99;                     // Distance from ground to target
        public static final double a1 = 61.31;                  // Limelight mount angle
        public static final double maxLimelightDriveSpeed = 0.6;
        public static final double maxLimelightTurnSpeed = 0.8;     // Make one for auto = 0.6
        public static final double distanceIntegralWindow = 5;
        public static final double turnIntegralWindow = 8;

        // Auto Position PID Values
        public static final double kpDistance3 = 0.2;
        public static final double kiDistance3 = 0.25;
        public static final double kdDistance3 = 0;
        public static final double kpAim3 = 0.2;
        public static final double kiAim3 = 0.5;
        public static final double kdAim3 = 0;

        // Trench Position PID Values
        public static final double kpDistance0 = 0.5;
        public static final double kiDistance0 = 0.1;
        public static final double kdDistance0 = 0;
        public static final double kpAim0 = 0.2;
        public static final double kiAim0 = 0.2;
        public static final double kdAim0 = 0.001;
    }

    public static final class ShooterConstants{
        public static final int leftShooterMotorChannel = 8;
        public static final int rightShooterMotorChannel = 1;
        public static final int shooterUpChannel = 2;
        public static final int shooterDownChannel = 3;
        public static final double shootSpeedPosition0 = 0.97;
        public static final double shootSpeedPosition1 = 0.94;
        public static final double shootSpeedPosition2 = 0;
        public static final double shootSpeedPosition3 = 0.69;       // Auto shooter speed
        public static final double shooterSpeedOffset = 0.85;
        public static final int shooterEncoderChannelA = 0;
        public static final int shooterEncoderChannelB = 1;
        public static final int shooterEncoderAverageSamples = 127;
    }

}
