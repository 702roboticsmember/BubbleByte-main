// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.*;

public class CoralIntakeSubsystem extends SubsystemBase {
  private SparkMax leftMotor = new SparkMax(Constants.CoralIntakeConstants.LeftMotorID, MotorType.kBrushless);
  private SparkMax rightMotor = new SparkMax(Constants.CoralIntakeConstants.RightMotorID, MotorType.kBrushless);
  // private Spark LeftMotor = new Spark(Constants.CoralIntakeConstants.LeftMotorID);
  // private Spark RightMotor = new Spark(Constants.CoralIntakeConstants.RightMotorID);
  private DigitalInput sensor = new DigitalInput(Constants.LIMIT_SWITCH_INTAKE);
  /** Creates a new ClimbSubsystem. */
  public CoralIntakeSubsystem() {
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    SparkMaxConfig rightConfig = new SparkMaxConfig();
    

    leftConfig.inverted(Constants.CoralIntakeConstants.LeftMotorInverted);
    rightConfig.inverted(Constants.CoralIntakeConstants.RightMotorInverted);
    leftConfig.smartCurrentLimit(20);
    rightConfig.smartCurrentLimit(20);
    //leftConfig.voltageCompensation(0);
    
    leftMotor.configure(leftConfig, Constants.CoralIntakeConstants.Reset, Constants.CoralIntakeConstants.Persist);
    rightMotor.configure(rightConfig, Constants.CoralIntakeConstants.Reset, Constants.CoralIntakeConstants.Persist);
   
    

    // TalonFXConfigurator LeftConfig = LeftMotor.se
    // CurrentLimitsConfigs configs = new CurrentLimitsConfigs();

    // configs.StatorCurrentLimit = constants.STATOR_CURRENT_LIMIT;
    // configs.SupplyCurrentLimit = constants.CURRENT_LIMIT;
    // configs.StatorCurrentLimitEnable = constants.ENABLE_STATOR_CURRENT_LIMIT;
    // configs.SupplyCurrentLimitEnable = constants.ENABLE_CURRENT_LIMIT;

    // talonFXConfigurator.apply(configs);

    // Ive searched for a while but couldnt find any way to currnet limit sparks
  }

  @Override
  public void periodic() {
    //SmartDashboard.putNumber("ClimbPose", getDegrees());
    SmartDashboard.putBoolean("beam", getSensor());
    // This method will be called once per schedule.putr run
  }

  /**
   * @return true if the beam is broken, false if otherwise
   * @deprecated
   * This sensor was improperly wired. Do not use.
   * This method won't work at Hueneme. Talk to James or Aleph to
   * wire this properly for LAR.
   * 
   * <img src="https://docs.wpilib.org/en/stable/_images/ir-beam-break-sensor-to-roborio.svg" alt="proper wiring" />
   */
  @Deprecated
  public boolean getSensor(){
    return sensor.get();
  }

  public void setSpeed(double speed) {
    leftMotor.set(speed);
    rightMotor.set(speed);
  }

  public Command run(DoubleSupplier input){
    return this.runEnd(() -> this.setSpeed(MathUtil.applyDeadband(input.getAsDouble(), 0.1)), () -> this.setSpeed(0.0));
  }

}
