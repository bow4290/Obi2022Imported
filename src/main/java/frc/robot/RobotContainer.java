/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.climber.ClimbCommand;
import frc.robot.commands.climber.ToggleClimberSolenoidCommand;
import frc.robot.commands.conveyor.ConveyorShootBallCommand;
import frc.robot.commands.conveyor.ReverseConveyorCommand;
import frc.robot.commands.drivetrain.ShiftGearCommand;
import frc.robot.commands.intake.ToggleIntakeSolenoidCommand;
import frc.robot.commands.shooter.ShootCommand;
import frc.robot.commands.shooter.ToggleShooterSolenoidCommand;
import frc.robot.subsystems.*;

public class RobotContainer {

  public static Joystick joystickLeft;
  public static Joystick joystickRight;
  public static Joystick xboxController;
  public DrivetrainSubsystem drivetrainSubsystem;
  public IntakeSubsystem intakeSubsystem;
  public ClimberSubsystem climberSubsystem;
  public ConveyorSubsystem conveyorSubsystem;
  public ShooterSubsystem shooterSubsystem;

  public RobotContainer() {

    joystickLeft = new Joystick(OIConstants.LEFT_JOYSTICK);
    joystickRight = new Joystick(OIConstants.RIGHT_JOYSTICK);
    xboxController = new Joystick(OIConstants.XBOX_CONTROLLER);

    drivetrainSubsystem = new DrivetrainSubsystem();
    intakeSubsystem = new IntakeSubsystem();
    climberSubsystem = new ClimberSubsystem();
    conveyorSubsystem = new ConveyorSubsystem();
    shooterSubsystem = new ShooterSubsystem();
    
    drivetrainSubsystem.setDefaultCommand(new RunCommand(() -> drivetrainSubsystem.drive(getLeftY(), getRightY()), drivetrainSubsystem));
    intakeSubsystem.setDefaultCommand(new RunCommand(() -> intakeSubsystem.intakeIn(getAxisValue(3)), intakeSubsystem));      // Intake motor follows xbox Right Trigger

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    setJoystickButtonWhenPressed(joystickRight, 1, new ShiftGearCommand(drivetrainSubsystem));            // Shift gear         = press Right Joystick Trigger
    
    setJoystickButtonWhenPressed(xboxController, 1, new ToggleShooterSolenoidCommand(shooterSubsystem));  // Shooter pneumatics = press xbox A Button
    setJoystickButtonWhenPressed(xboxController, 2, new ToggleIntakeSolenoidCommand(intakeSubsystem));    // Intake pneumatics  = press xbox B Button
    setJoystickButtonWhileHeld(xboxController, 3, new ClimbCommand(climberSubsystem));                    // To climb           = hold xbox X Button
    setJoystickButtonWhenPressed(xboxController, 4, new ToggleClimberSolenoidCommand(climberSubsystem));  // Climber pneumatics = press xbox Y Button

    // To shoot balls hold down the Right Bumper. Balls should automatically convey.
    setJoystickButtonWhileHeld(xboxController, 6, new ParallelCommandGroup(
      new ShootCommand(shooterSubsystem),
      new ConveyorShootBallCommand(conveyorSubsystem)
    ));
    setJoystickButtonWhileHeld(xboxController, 10, new ReverseConveyorCommand(conveyorSubsystem));       // Reverse conveyor   = press xbox Right Stick in
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

  // WhenPressed runs the command once at the moment the button is pressed.
  private void setJoystickButtonWhenPressed(Joystick joystick, int button, CommandBase command) {
    new JoystickButton(joystick, button).whenPressed(command);
  }

  // WhileHeld constantly starts the command and repeatedly schedules while the button is held. Cancels when button is released.
  private void setJoystickButtonWhileHeld(Joystick joystick, int button, CommandBase command) {
    new JoystickButton(joystick, button).whileHeld(command);
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
