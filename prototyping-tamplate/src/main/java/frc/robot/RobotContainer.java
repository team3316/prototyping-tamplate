// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.constants.DriveConstants;
import frc.robot.constants.JoysticksConstants;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.drive.GyroIOPigeon;
import frc.robot.subsystems.drive.Module.ModuleLocation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private boolean isFieldRelative;
  private final Drivetrain m_drivetrain;
  private final CommandPS5Controller m_controller = new CommandPS5Controller(JoysticksConstants.DRIVER_CONTROLLER_PORT);

  public RobotContainer() {
          isFieldRelative = true;

      m_drivetrain = new Drivetrain(
        new GyroIOPigeon(DriveConstants.GYRO_PORT),
        new ModuleIOSpark(ModuleLocation.FL),
        new ModuleIOSpark(ModuleLocation.FR),
        new ModuleIOSpark(ModuleLocation.BL),
        new ModuleIOSpark(ModuleLocation.BR)
      );

  
    m_drivetrain.setDefaultCommand(Commands.run(() -> m_drivetrain.runVelocity(
      m_controller.getLeftY() * DriveConstants.MAX_SPEED_METERS_PER_SECOND,
      m_controller.getLeftX() * DriveConstants.MAX_SPEED_METERS_PER_SECOND,
      m_controller.getRightX() * DriveConstants.MAX_ANGULAR_VELOCITY_RAD_PER_SEC
      ,isFieldRelative
    ),
    m_drivetrain));
    configureBindings();
  }


  private void configureBindings() {
;   
  }

  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
