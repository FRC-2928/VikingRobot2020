/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.ConversionConstants;
import frc.robot.commands.controlpanel.RotateSegments;
import frc.robot.commands.controlpanel.RotateToColor;
import frc.robot.subsystems.controlpanel.ControlPanelSubsystem;


/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final ControlPanelSubsystem m_controlPanel = new ControlPanelSubsystem();

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

    ConfigureControlButtons(); 

  }

  public void ConfigureControlButtons () {

    // Spin the control panel three times
    new JoystickButton(m_operatorController, Button.kY.value)
      .whenPressed(new RotateSegments(m_controlPanel, ConversionConstants.threeTurns));

    // Spin the control panel to target color
    new JoystickButton(m_operatorController, Button.kX.value)
      .whenPressed(new ConditionalCommand(

       // TRUE - the detected color is unknown so rotate half a segment 
       new RotateSegments(m_controlPanel, 0.5)
         // and then rotate to color
         .andThen(new RotateToColor(m_controlPanel)),

       // FALSE - the color is known so rotate to target color
       new RotateToColor(m_controlPanel),

       // CONDITION - is the color unknown?
       m_controlPanel.unknownColor()
     ));
    
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
    return new PrintCommand("Default auto command");
  }
}
