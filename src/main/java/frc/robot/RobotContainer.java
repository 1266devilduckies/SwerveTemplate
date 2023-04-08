// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.DuckSwerveDrive;

public class RobotContainer {
  private final CommandPS4Controller driverJoystick = new CommandPS4Controller(Constants.DriveControllerCharacteristics.port);

  public final DuckSwerveDrive swerveSubsystem = new DuckSwerveDrive(
    Constants.SwerveDriveCharacteristics.chassisDimensionsMeters, Constants.SwerveDriveCharacteristics.ksAngularVolts,
    Constants.SwerveDriveCharacteristics.ksLinearVolts, Constants.SwerveDriveCharacteristics.kvLinearVoltMeters, Constants.SwerveDriveCharacteristics.kaLinearVoltMeters,
    Constants.CANIDS.SwerveModules.FrontLeft.turnMotor, Constants.CANIDS.SwerveModules.FrontLeft.driveMotor, Constants.CANIDS.SwerveModules.FrontLeft.CANcoder,
    Constants.CANIDS.SwerveModules.FrontRight.turnMotor, Constants.CANIDS.SwerveModules.FrontRight.driveMotor, Constants.CANIDS.SwerveModules.FrontRight.CANcoder,
    Constants.CANIDS.SwerveModules.BackLeft.turnMotor, Constants.CANIDS.SwerveModules.BackLeft.driveMotor, Constants.CANIDS.SwerveModules.BackLeft.CANcoder,
    Constants.CANIDS.SwerveModules.BackRight.turnMotor, Constants.CANIDS.SwerveModules.BackRight.driveMotor, Constants.CANIDS.SwerveModules.BackRight.CANcoder
  );

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new TeleopSwerve(swerveSubsystem, driverJoystick::getLeftX, driverJoystick::getLeftY, driverJoystick::getRightX));
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
