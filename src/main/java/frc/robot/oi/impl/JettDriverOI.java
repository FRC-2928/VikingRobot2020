package frc.robot.oi.impl;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.drivetrain.DrivetrainArcadeDriveCommand;
import frc.robot.commands.drivetrain.TransmissionSetGearCommand;
import frc.robot.commands.intake.GroundIntakeCommand;
import frc.robot.commands.intake.OpenIntakeCommand;
import frc.robot.commands.shooter.SetShooter;
import frc.robot.oi.DriverOI;
import frc.robot.subsystems.SubsystemContainer;
import frc.robot.subsystems.drivetrain.TransmissionSubsystem.GearState;

public class JettDriverOI{


    public static void bindDriverButtons(XboxController controller, SubsystemContainer subsystems) {
        /*------------------------------------  Buttons  --------------------------------------- */

        //Drivetrain buttons
        subsystems.drivetrain.setDefaultCommand(
            new DrivetrainArcadeDriveCommand(
                subsystems.drivetrain, 
                () -> -controller.getY(Hand.kLeft), 
                () -> controller.getX(Hand.kRight)
            )
        );
        Button shiftLowButton = new JoystickButton(controller, XboxController.Button.kX.value);
        Button shiftHighButton = new JoystickButton(controller, XboxController.Button.kY.value);

        //Intake buttons
        Button groundIntakeButton = new Button(() -> controller.getTriggerAxis(Hand.kLeft) > 0.1);
        Button flipIntakeButton = new JoystickButton(controller, XboxController.Button.kBumperLeft.value);
        
        //Climber buttons
        Button climbButton = new JoystickButton(controller, XboxController.Axis.kRightTrigger.value);

        //Shooting buttons
        Button autoShootingButton = new JoystickButton(controller, XboxController.Button.kBumperRight.value);
        Button enableDistanceMapButton = new Button(() -> controller.getPOV() == 180);
        Button shooterDebugButton = new Button(() -> controller.getPOV() == 90);
        Button feedButton = new Button(() -> controller.getTriggerAxis(Hand.kRight) > 0.1);

        /*------------------------------------  Bindings  --------------------------------------- */

        //Drivetrain bindings
        shiftLowButton.whenPressed(new TransmissionSetGearCommand(subsystems.transmission, GearState.LOW));
        shiftHighButton.whenPressed(new TransmissionSetGearCommand(subsystems.transmission, GearState.HIGH));

        //Intake bindings
        groundIntakeButton.whileHeld(new GroundIntakeCommand(subsystems.roller, subsystems.arm));
        flipIntakeButton.whileHeld(new OpenIntakeCommand(subsystems.roller, subsystems.arm));

        // climbButton.whileHeld()

        autoShootingButton.whileHeld(new SetShooter(subsystems));

    }
}