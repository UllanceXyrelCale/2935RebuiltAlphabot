// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.ObjectInputFilter.Config;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Variables;

public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX topShooterMotor;
  private final TalonFX bottomShooterMotor; 
  private final VelocityVoltage velocityRequest;

  /** Creates a new ShooterSubsytem. */
  public ShooterSubsystem() {
    topShooterMotor = new TalonFX(40);
    bottomShooterMotor = new TalonFX(41);
    
    velocityRequest = new VelocityVoltage(0).withSlot(0);
    topShooterMotor.getConfigurator().apply(Configs.shooterMotor.shooterConfig);

    bottomShooterMotor.setControl(   
      new Follower(topShooterMotor.getDeviceID(), MotorAlignmentValue.Aligned)
    );
  }

  public double getShooterSpeed () {
    return topShooterMotor.getVelocity().getValueAsDouble();
  }

  public boolean atTargetSpeed () {
    return Math.abs(getShooterSpeed() - Variables.shooter.shooterRPS) < 1;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    topShooterMotor.setControl(velocityRequest.withVelocity(Variables.shooter.shooterRPS));
    SmartDashboard.putNumber("Variable", Variables.shooter.shooterRPS);
    SmartDashboard.putBoolean("At Target Speed", atTargetSpeed());
  }
}