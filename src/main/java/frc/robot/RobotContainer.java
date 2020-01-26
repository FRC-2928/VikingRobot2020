/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.ShooterHoodSubsystem;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final FlywheelSubsystem flywheelsubsystem = new FlywheelSubsystem();
  private final ShooterHoodSubsystem shooterhoodsubsystem = new ShooterHoodSubsystem();

  private final XboxController driveController = new XboxController(0);
  private final JoystickButton openLoopFlywheel = new JoystickButton(driveController, 5);
  private final JoystickButton velocityControlFlywheel = new JoystickButton(driveController, 6);

  private final JoystickButton positionControlHood = new JoystickButton(driveController, 1);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // shooterhoodsubsystem.setDefaultCommand(
    //   new RunCommand(() -> {
    //       double speed = -driveController.getY(Hand.kRight);
    //       shooterhoodsubsystem.setPower(speed);
    //   }, 
    //   shooterhoodsubsystem)
    // );

    velocityControlFlywheel.whileHeld(
      new RunCommand(() -> {
        double targetRPM = SmartDashboard.getNumber("Target RPM", 0);
        flywheelsubsystem.setFlywheelRPM(targetRPM);
      }, 
      flywheelsubsystem)
    );

    openLoopFlywheel.whileHeld(new RunCommand(() -> flywheelsubsystem.setPower(0.85),flywheelsubsystem));

    positionControlHood.whileHeld(new RunCommand(() -> shooterhoodsubsystem.setHoodDegrees(), shooterhoodsubsystem));
  }

  public void onInitialize(){
    flywheelsubsystem.configFeedbackGains();
    shooterhoodsubsystem.configPIDGains();
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new PrintCommand("screw u ");
  }
}
