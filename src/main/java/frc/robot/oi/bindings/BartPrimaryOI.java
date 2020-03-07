package frc.robot.oi.bindings;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.chassis.drivetrain.DrivetrainArcadeDriveCommand;
import frc.robot.commands.chassis.transmission.TransmissionSetLowGearCommand;
import frc.robot.oi.Bindable;
import frc.robot.subsystems.Subsystems;

/**
 * Add your docs here.
 */
public class BartPrimaryOI implements Bindable {
  private Subsystems m_subsystems;

  private DoubleSupplier throttleSupplier;
  private DoubleSupplier wheelSupplier;

  private Button toggleGearButton;

  public BartPrimaryOI(XboxController controller, Subsystems subsystems) {
    m_subsystems = subsystems;

    throttleSupplier = () -> controller.getTriggerAxis(Hand.kRight) - controller.getTriggerAxis(Hand.kLeft);
    wheelSupplier = () -> controller.getX(Hand.kLeft);
    toggleGearButton = new JoystickButton(controller, XboxController.Button.kStickLeft.value);
  }

  @Override
  public void bind() {
    toggleGearButton.whenPressed(new ConditionalCommand(
      new TransmissionSetLowGearCommand(m_subsystems.chassis.transmission),
      new TransmissionSetLowGearCommand(m_subsystems.chassis.transmission),
      m_subsystems.chassis.transmission::isHighGear
    ));
  }
}
