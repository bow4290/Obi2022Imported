/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.conveyor.ConveyorIndexBallCommand;
import frc.robot.commands.conveyor.ConveyorShootBallCommand;
import frc.robot.commands.conveyor.ReverseConveyorCommand;
import frc.robot.commands.auto.AutoDriveDistanceCommand;
import frc.robot.commands.auto.AutoTurnAngleCommand;
import frc.robot.commands.shooter.ShootCommand;
import frc.robot.subsystems.*;
import frc.robot.sensors.PIDParams;

public class RobotContainer {

  public static Joystick xboxController;

  private DrivetrainSubsystem drivetrainSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private ConveyorSubsystem conveyorSubsystem;
  private ShooterSubsystem shooterSubsystem;

  private Command AutoShootAndCollect;
  private Command AutoShootOnly;
  private Command AutoDriveOnly;

  SendableChooser<Command> chooser = new SendableChooser<>();

  public RobotContainer() {

    xboxController = new Joystick(OIConstants.XBOX_CONTROLLER);

    drivetrainSubsystem = new DrivetrainSubsystem();
    intakeSubsystem = new IntakeSubsystem();
    conveyorSubsystem = new ConveyorSubsystem();
    shooterSubsystem = new ShooterSubsystem();

    // Change setpoint to equal distance from calibration point (start line) to shooting distance in auto.
/*
    AutoShootAndCollect =
      new SequentialCommandGroup(
        new LimelightInitCommand(shooterSubsystem),
        new LimelightDriveToDistanceCommand(drivetrainSubsystem, limelight, autoDistanceParams),
        new LimelightDriveToHeadingCommand(drivetrainSubsystem, limelight, autoHeadingParams),
        new LimelightEndCommand(),
        new ParallelRaceGroup(
          new ShootCommand(shooterSubsystem, conveyorSubsystem),
          new ConveyorShootBallCommand(conveyorSubsystem, shooterSubsystem),
          new WaitCommand(4)),
        new AutoTurnAngleCommand(drivetrainSubsystem, 45),
        new ParallelRaceGroup(
          new AutoDriveDistanceCommand(drivetrainSubsystem, intakeSubsystem, 36),
          new WaitCommand(5))
      );

    AutoShootOnly =
      new SequentialCommandGroup(
        new LimelightInitCommand(shooterSubsystem),
        new LimelightDriveToDistanceCommand(drivetrainSubsystem, limelight, autoDistanceParams),
        new LimelightDriveToHeadingCommand(drivetrainSubsystem, limelight, autoHeadingParams),
        new LimelightEndCommand(),
        new ParallelRaceGroup(
          new ShootCommand(shooterSubsystem, conveyorSubsystem),
          new ConveyorShootBallCommand(conveyorSubsystem, shooterSubsystem),
          new WaitCommand(4))
      );

    AutoDriveOnly =
      new ParallelRaceGroup(
        new RunCommand(() -> drivetrainSubsystem.drive(0.6, 0.6), drivetrainSubsystem),
        new WaitCommand(2)
      );
*/
    chooser.setDefaultOption("Auto Shoot Only", AutoShootOnly);
    chooser.addOption("Auto Shoot and Collect", AutoShootAndCollect);
    chooser.addOption("Auto Drive Only", AutoDriveOnly);
    SmartDashboard.putData(chooser);
    
    drivetrainSubsystem.setDefaultCommand(new RunCommand(() -> drivetrainSubsystem.drive(getLeftY(), -getRightX(), xboxController.getRawButton(7)), drivetrainSubsystem));   // Negate the values because dumb joysticks
    // Left bumper intakes
    intakeSubsystem.setDefaultCommand(new RunCommand(() -> intakeSubsystem.intakeIn(xboxController.getRawButton(5)?1:0), intakeSubsystem));                      // Intake balls   = hold xbox Right Trigger
    conveyorSubsystem.setDefaultCommand(new ConveyorIndexBallCommand(conveyorSubsystem)); 
    
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // RightJoystick Buttons
    PIDParams teleopHeadingParams = new PIDParams(
      LimelightConstants.kpAimTeleop,
      LimelightConstants.kiAimTeleop,
      LimelightConstants.kdAimTeleop,
      LimelightConstants.headingPositionTolerance,
      LimelightConstants.headingVelocityTolerance,
      0.0);
    
    // Left Joystick Buttons
    // setJoystickButtonWhenPressed(joystickRight, 1, new ShiftGearCommand(drivetrainSubsystem));            // Shift gear         = press Right Joystick Trigger

    // Xbox Controller Buttons
    // Right Bumper
    setJoystickButtonWhileHeld(xboxController, 6, new ParallelCommandGroup(                               // Shoot balls        = hold xbox Right Bumper
      new ShootCommand(shooterSubsystem, conveyorSubsystem),
      new ConveyorShootBallCommand(conveyorSubsystem, shooterSubsystem)
      ));
    // B button
    setJoystickButtonWhileHeld(xboxController, 2, new ReverseConveyorCommand(conveyorSubsystem));        // Reverse conveyor   = hold xbox Right Stick in
  }

  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }

  public double getLeftY(){
    return -getAxisValue(1);   // Joystick Y axis provides -1 for forward, so invert this
  }

  public double getLeftX(){
    return getAxisValue(0);
  }

  public double getRightY(){
    return -getAxisValue(5);  // Joystick Y axis provides -1 for forward, so invert this
  }

  public double getRightX(){
    return getAxisValue(4);
  }

  public static double getAxisValue(int axis){
    return xboxController.getRawAxis(axis);
  }

  public int getDPad(){
    return xboxController.getPOV();
  }

  // WhenPressed runs the command once at the moment the button is pressed.
  private void setJoystickButtonWhenPressed(Joystick joystick, int button, CommandBase command) {
    new JoystickButton(joystick, button).whenPressed(command);
  }

  // WhileHeld constantly starts the command and repeatedly schedules while the button is held. Cancels when button is released.
  private void setJoystickButtonWhileHeld(Joystick joystick, int button, CommandBase command) {
    new JoystickButton(joystick, button).whileHeld(command);
  }

  // WhenHeld starts the command once when the button is first pressed. Command runs until button is released or command interrupted.
  private void setJoystickButtonWhenHeld(Joystick joystick, int button, CommandBase command) {
    new JoystickButton(joystick, button).whenHeld(command);
  }

}
