// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Brazo;

public class MoveEncoder extends CommandBase {
  Brazo brazo;
  boolean extend;
  /** Creates a new MoveEncoder. */
  public MoveEncoder(Brazo b, boolean extend) {
    this.brazo = b;
    this.extend = extend;
    addRequirements(brazo);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while t he command is scheduled.
  @Override
  public void execute() {
    if (extend) {
      brazo.MoveEncoder(Constants.SPEED);
    } else {
      brazo.MoveEncoder(-Constants.SPEED);
    }
    SmartDashboard.putNumber("Valor Encoder", brazo.Positionencoder());
    SmartDashboard.putNumber("Valor Encoder", brazo.Positionencoder());
  }

  //PRUEBA EXTENSION
  public void extend() {
  SmartDashboard.putNumber("Valor Encoder", brazo.Positionencoder());
  while (brazo.Positionencoder()<30);
  brazo.MoveEncoder(Constants.SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override 
  public void end(boolean interrupted) {
    brazo.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (extend) {
      return brazo.Positionencoder() < 32;
    } else {
      return brazo.Positionencoder() < 0.1;
    }
  }
}
