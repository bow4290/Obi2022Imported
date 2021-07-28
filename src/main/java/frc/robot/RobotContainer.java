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
import frc.robot.Constants.OIConstants;
import frc.robot.commands.climber.ClimbCommand;
import frc.robot.commands.climber.ToggleClimberSolenoidCommand;
import frc.robot.commands.conveyor.ConveyorIndexBallCommand;
import frc.robot.commands.conveyor.ConveyorShootBallCommand;
import frc.robot.commands.conveyor.ReverseConveyorCommand;
import frc.robot.commands.drivetrain.ShiftGearCommand;
import frc.robot.commands.intake.ToggleIntakeSolenoidCommand;
import frc.robot.commands.auto.AutoDriveDistanceCommand;
import frc.robot.commands.auto.AutoTurnAngleCommand;
import frc.robot.commands.auto.LimelightAutoDriveToDistanceCommand;
import frc.robot.commands.auto.LimelightAutoDriveToHeadingCommand;
import frc.robot.commands.limelight.LimelightDriveToDistanceCommand;
import frc.robot.commands.limelight.LimelightDriveToHeadingCommand;
import frc.robot.commands.limelight.LimelightEndCommand;
import frc.robot.commands.limelight.LimelightInitCommand;
import frc.robot.commands.shooter.ShootCommand;
import frc.robot.commands.shooter.ToggleShooterSolenoidCommand;
import frc.robot.sensors.Limelight;
import frc.robot.subsystems.*;

public class RobotContainer {

  public static Joystick joystickLeft;
  public static Joystick joystickRight;
  public static Joystick xboxController;

  private DrivetrainSubsystem drivetrainSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private ClimberSubsystem climberSubsystem;
  private ConveyorSubsystem conveyorSubsystem;
  private ShooterSubsystem shooterSubsystem;

  private Command AutoShootAndCollect;
  private Command AutoShootOnly;
  private Command AutoDriveOnly;

  public Limelight limelight;

  SendableChooser<Command> chooser = new SendableChooser<>();

  public RobotContainer() {

    joystickLeft = new Joystick(OIConstants.LEFT_JOYSTICK);
    joystickRight = new Joystick(OIConstants.RIGHT_JOYSTICK);
    xboxController = new Joystick(OIConstants.XBOX_CONTROLLER);

    drivetrainSubsystem = new DrivetrainSubsystem();
    intakeSubsystem = new IntakeSubsystem();
    climberSubsystem = new ClimberSubsystem();
    conveyorSubsystem = new ConveyorSubsystem();
    shooterSubsystem = new ShooterSubsystem();

    limelight = new Limelight();
    limelight.setPipeline(3);  // Default Position 3 (Start Line)

    AutoShootAndCollect =
      new SequentialCommandGroup(
        new LimelightInitCommand(),
        new LimelightAutoDriveToDistanceCommand(drivetrainSubsystem, limelight),
        new LimelightAutoDriveToHeadingCommand(drivetrainSubsystem, limelight),
        new LimelightEndCommand(),
        new ParallelRaceGroup(                                      // Once the TimeCommand ends, this entire group interrupts
          new ShootCommand(shooterSubsystem, conveyorSubsystem, limelight),
          new ConveyorShootBallCommand(conveyorSubsystem,limelight.getPipeline()),
          new WaitCommand(4)),
        new AutoTurnAngleCommand(drivetrainSubsystem, 45),
        new ParallelRaceGroup(
          new AutoDriveDistanceCommand(drivetrainSubsystem, intakeSubsystem, 36),
          new WaitCommand(5))
      );

    AutoShootOnly =
      new SequentialCommandGroup(new LimelightInitCommand(),
        new LimelightAutoDriveToDistanceCommand(drivetrainSubsystem, limelight),
        new LimelightAutoDriveToHeadingCommand(drivetrainSubsystem, limelight), new LimelightEndCommand(),
        new ParallelRaceGroup( // Once the TimeCommand ends, this entire group interrupts
          new ShootCommand(shooterSubsystem, conveyorSubsystem, limelight),
          new ConveyorShootBallCommand(conveyorSubsystem, limelight.getPipeline()), new WaitCommand(4)));

    AutoDriveOnly = new AutoDriveDistanceCommand(drivetrainSubsystem, intakeSubsystem, -24);

    chooser.setDefaultOption("Auto Shoot and Collect", AutoShootAndCollect);
    chooser.addOption("Auto Shoot Only", AutoShootOnly);
    chooser.addOption("Auto Drive Only", AutoDriveOnly);
    SmartDashboard.putData(chooser);
    
    drivetrainSubsystem.setDefaultCommand(new RunCommand(() -> drivetrainSubsystem.drive(-getLeftY(), -getRightY()), drivetrainSubsystem));   // Negate the values because dumb joysticks
    intakeSubsystem.setDefaultCommand(new RunCommand(() -> intakeSubsystem.intakeIn(getAxisValue(3)), intakeSubsystem));                      // Intake motor follows xbox Right Trigger
    conveyorSubsystem.setDefaultCommand(new ConveyorIndexBallCommand(conveyorSubsystem)); 
    
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // Left Joystick Buttons
    setJoystickButtonWhenHeld(joystickLeft, 1, new SequentialCommandGroup(               // Limelight track and shoot = hold Left Joystick Trigger
        new LimelightInitCommand(),
        new LimelightDriveToHeadingCommand(drivetrainSubsystem, limelight),
        new LimelightDriveToDistanceCommand(drivetrainSubsystem, limelight),
        new LimelightDriveToHeadingCommand(drivetrainSubsystem, limelight),
        new LimelightEndCommand()
    ));
    
    // Right Joystick Buttons
    setJoystickButtonWhenPressed(joystickRight, 1, new ShiftGearCommand(drivetrainSubsystem));            // Shift gear         = press Right Joystick Trigger

    // Xbox Controller Buttons
    setJoystickButtonWhenPressed(xboxController, 1, new ToggleShooterSolenoidCommand(shooterSubsystem));  // Shooter pneumatics = press xbox A Button
    setJoystickButtonWhenPressed(xboxController, 2, new ToggleIntakeSolenoidCommand(intakeSubsystem));    // Intake pneumatics  = press xbox B Button
    setJoystickButtonWhileHeld(xboxController, 3, new ClimbCommand(climberSubsystem));                    // To climb           = hold xbox X Button
    setJoystickButtonWhenPressed(xboxController, 4, new ToggleClimberSolenoidCommand(climberSubsystem));  // Climber pneumatics = press xbox Y Button
    setJoystickButtonWhileHeld(xboxController, 6, new ParallelCommandGroup(                               // Shoot balls        = hold xbox Right Bumper
      new ShootCommand(shooterSubsystem, conveyorSubsystem, limelight),
      new ConveyorShootBallCommand(conveyorSubsystem, limelight.getPipeline())
      ));
    setJoystickButtonWhileHeld(xboxController, 10, new ReverseConveyorCommand(conveyorSubsystem));        // Reverse conveyor   = hold xbox Right Stick in
  }

  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }

  public double getLeftY(){
    return -joystickLeft.getY();   // Joystick Y axis provides -1 for forward, so invert this
  }

  public double getLeftX(){
    return joystickLeft.getX();
  }

  public double getRightY(){
    return -joystickRight.getY();  // Joystick Y axis provides -1 for forward, so invert this
  }

  public double getRightX(){
    return joystickRight.getX();
  }

  public static double getAxisValue(int axis){
    return xboxController.getRawAxis(axis);
  }

  public int getDPad(){
    return xboxController.getPOV();
  }

  public void setShooterPosition(int position){
    limelight.setPipeline(position);
  }

  public double getShooterPosition(){
    return limelight.getPipeline();
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
