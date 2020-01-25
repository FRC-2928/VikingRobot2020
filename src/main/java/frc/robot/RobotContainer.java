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
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.RobotMap;
import frc.robot.subsystems.controlpanel.ControlPanelSubsystem;
import frc.robot.subsystems.intake.Intake;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;


/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final FlywheelSubsystem flywheelsubsystem = new FlywheelSubsystem();

  private final XboxController driveController = new XboxController(0);
  private final JoystickButton openLoopFlywheel = new JoystickButton(driveController, 5);
  private final JoystickButton velocityControlFlywheel = new JoystickButton(driveController, 6);
  private final Intake m_intake = new Intake();
  private final ControlPanelSubsystem m_controlPanel = new ControlPanelSubsystem();

  XboxController m_driverController = new XboxController(Constants.OIConstants.kDriverControllerPort);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_intake.setDefaultCommand( new InstantCommand(m_intake::stopMotor, m_intake));
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    velocityControlFlywheel.whileHeld(
      new RunCommand(() -> {
        double targetRPM = SmartDashboard.getNumber("Target RPM", 0);
        flywheelsubsystem.setFlywheelRPM(targetRPM);
      }, 
      flywheelsubsystem)
    );

    openLoopFlywheel.whileHeld(new RunCommand(() -> flywheelsubsystem.setPower(0.75),flywheelsubsystem));

    // Spin the control panel three times
    new JoystickButton(m_driverController, Button.kY.value)
        .whenPressed(() -> m_controlPanel.rotateSegments(RobotMap.threeTurns));
  }

  public void onInitialize(){
    flywheelsubsystem.configFeedbackGains();
    //buttons for the intake
    configureIntakeButtons();
    
  }

  private void configureIntakeButtons() {
    // Pickup balls from the ground
    new JoystickButton(m_driverController, Button.kA.value).whenPressed(new SequentialCommandGroup(
      //extend intake
      new InstantCommand(m_intake::groundPickup, m_intake ),
      //wait until intake deploys
      new WaitCommand(1),
      // run motors
      new RunCommand(m_intake::startMotor, m_intake)
    ));


    // Stow the intake
    new JoystickButton(m_driverController, Button.kB.value).whenReleased(new SequentialCommandGroup(
      //stop motors
      new InstantCommand(m_intake::stopMotor, m_intake),
      //retract intake
      new InstantCommand(m_intake::Stowed, m_intake )
    ));



    // Pickup balls from the Player Station
    new JoystickButton(m_driverController, Button.kX.value).whenPressed(new SequentialCommandGroup(
      //extend intake
      new InstantCommand(m_intake::StationPickup, m_intake ),
      //wait until intake deploys
      new WaitCommand(1),
      // run motors
      new RunCommand(m_intake::startMotor, m_intake)
    ));

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
