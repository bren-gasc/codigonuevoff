// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrainTankFalcon extends SubsystemBase {
  
  WPI_TalonFX motorderecho1;
  WPI_TalonFX motorderecho2;
  WPI_TalonFX motorizquierdo1;
  WPI_TalonFX motorizquierdo2;

  MotorControllerGroup derecha;
  MotorControllerGroup izquierda;

  DifferentialDrive chassis;
  
  /** Creates a new DriveTrainTankFalcon. */
  public DriveTrainTankFalcon() {
  motorderecho1 = new WPI_TalonFX(Constants.IDM1);
  motorderecho1.setInverted(false);
  motorderecho2 = new WPI_TalonFX(Constants.IDM2);
  motorderecho2.setInverted(false);
  motorizquierdo1 = new WPI_TalonFX(Constants.IDM3);
  motorizquierdo1.setInverted( false);
  motorizquierdo2 = new WPI_TalonFX(Constants.IDM4);
  motorizquierdo2.setInverted( false);

  derecha = new MotorControllerGroup(motorderecho1, motorderecho2);
  izquierda = new MotorControllerGroup(motorizquierdo1, motorizquierdo2);

  chassis = new DifferentialDrive(izquierda, derecha);

  motorderecho1.getSensorCollection().setIntegratedSensorPosition(0,0);
  }

  public void ChasisJoysticks(double speed, double rotation) {
    chassis.arcadeDrive(MathUtil.clamp(speed, 9, Constants.MAXSPEED), rotation);
  }
  
   public double EncoderChassis(){
return motorderecho1.getSelectedSensorPosition();
   }

   public void stop(){
    chassis.arcadeDrive(0,0);
   }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
