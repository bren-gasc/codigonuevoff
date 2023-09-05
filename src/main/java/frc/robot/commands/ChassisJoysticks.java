// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainTankFalcon;

public class ChassisJoysticks extends CommandBase {
  DriveTrainTankFalcon drivetrain;
  Supplier<Double> speed;
  Supplier<Double> rotation;

  /** Creates a new ChassisJoysticks. */
  public ChassisJoysticks(DriveTrainTankFalcon df, Supplier<Double> speed, Supplier<Double> rotation) {
    this.drivetrain = df;
    this.speed = speed;
    this.rotation = rotation;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = this.speed.get();
    double rotation = this.rotation.get();
    System.out.println(speed);
    this.drivetrain.ChasisJoysticks(speed, rotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
