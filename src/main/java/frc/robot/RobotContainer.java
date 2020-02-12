package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.intake.FastForwardFeeder;
import frc.robot.commands.intake.StartFeeder;
import frc.robot.commands.intake.StopFeeder;
import frc.robot.oi.DriverOI;
import frc.robot.oi.OperatorOI;
import frc.robot.oi.impl.AbbyOperatorOI;
import frc.robot.oi.impl.JettDriverOI;
import frc.robot.subsystems.intake.FeederSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;


/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final FeederSubsystem m_feeder = new FeederSubsystem();
 
  private final DriverOI m_driverOI;
  private final OperatorOI m_operatorOI;

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    m_driverOI = new JettDriverOI(new XboxController(OIConstants.kDriverControllerPort));
    m_operatorOI = new AbbyOperatorOI(new XboxController(OIConstants.kOperatorControllerPort));

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

    configureFeederButtons();
    
  }

  public void configureFeederButtons() {
    // Also need to pass in the flywheel
    //m_driverOI.getAutoShootingButton().whenPressed(new FastForwardFeeder(m_feeder));

    m_operatorOI.getEnableFeederButton().whenPressed(new StartFeeder(m_feeder));
    m_operatorOI.getDisableFeederButton().whenPressed(new StopFeeder(m_feeder));
    m_operatorOI.getReverseFeederButton().whileHeld(new RunCommand(() -> m_feeder.reverseFeeder()));
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
