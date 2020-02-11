/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.commands.climber.ClimbHigh;
import frc.robot.commands.climber.ClimbMid;
import frc.robot.commands.climber.DeployClimber;
import frc.robot.commands.climber.ClimbLow;


/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final XboxController driveController = new XboxController(0);
  private final ClimberSubsystem m_climber = new ClimberSubsystem();

  XboxController m_driverController = new XboxController(Constants.OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(Constants.OIConstants.kOperatorControllerPort);
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
   *
   * 
   */
  private void configureButtonBindings() {

    ConfigureClimberButtons();
  
  }

  public void ConfigureClimberButtons() {
      
    // Deploy to top
    new JoystickButton(m_operatorController, Button.kA.value)
        .whenPressed(new DeployClimber(m_climber));

    // Deploy to high point
    new JoystickButton(m_operatorController, Button.kB.value)
        .whenPressed(new DeployClimber(m_climber)
          .andThen(new ClimbHigh(m_climber)));
     
    // Deploy to Mid point
    new JoystickButton(m_operatorController, Button.kX.value)
        .whenPressed(new DeployClimber(m_climber)
          .andThen(new ClimbMid(m_climber)));
      
    // Deploy to low point
    new JoystickButton(m_operatorController, Button.kY.value)
        .whenPressed(new DeployClimber(m_climber)
          .andThen(new ClimbLow(m_climber)));      
  }

  public void onInitialize(){   
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new PrintCommand("Default autonomous command");
  }
}
