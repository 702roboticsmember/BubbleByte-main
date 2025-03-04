// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

//might be removed idk just putting this here

//unnescesary import, idk what it was


import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);
  ClimbSubsystem c_ClimbSubsystem;
  public TalonFX elevmotor1 = new TalonFX(Constants.ElevatorConstants.Motor1ID);
  public TalonFX elevmotor2 = new TalonFX(Constants.ElevatorConstants.Motor2ID);

  
      
  /** Creates a new Elevator. */
  public ElevatorSubsystem(ClimbSubsystem c_ClimbSubsystem) {
    this.c_ClimbSubsystem = c_ClimbSubsystem;
    m_request.OverrideBrakeDurNeutral = true;
    
    // Elevator PID :D Will most likely be moved to Elevator PID later and errors will be fixed trust
    //Add current limits to constants??
    //CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs();
    
    
  

    var talonFXConfigs = new TalonFXConfiguration();

    var currentConfigs = talonFXConfigs.CurrentLimits;
    var motorConfigs = talonFXConfigs.MotorOutput;

    var limitConfigs = talonFXConfigs.SoftwareLimitSwitch;
// set slot 0 gains
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0; // no output for error derivative
    // slot0Configs.kG = 0;
    // slot0Configs.GravityType = GravityTypeValue.Elevator_Static;

    // set Motion Magic Velocity settings
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = Constants.ElevatorConstants.MaxVelocity;
    motionMagicConfigs.MotionMagicAcceleration = Constants.ElevatorConstants.MaxAcceleration; // Target acceleration of 400 rps/s (0.25 seconds to max)
    motionMagicConfigs.MotionMagicJerk = 0; // Target jerk of 4000 rps/s/s (0.1 seconds)

    
    limitConfigs.ForwardSoftLimitThreshold = Constants.ElevatorConstants.ForwardLimit;
    limitConfigs.ForwardSoftLimitEnable = Constants.ElevatorConstants.LimitEnable;
    
    limitConfigs.ReverseSoftLimitThreshold = Constants.ElevatorConstants.ReverseLimit;
    limitConfigs.ReverseSoftLimitEnable = Constants.ElevatorConstants.LimitEnable;
    
    

    currentConfigs.StatorCurrentLimit = Constants.ElevatorConstants.STATOR_CURRENT_LIMIT;
    currentConfigs.StatorCurrentLimitEnable = Constants.ElevatorConstants.ENABLE_STATOR_CURRENT_LIMIT;
    currentConfigs.SupplyCurrentLimit = Constants.ElevatorConstants.CURRENT_LIMIT;
    currentConfigs.SupplyCurrentLimitEnable = Constants.ElevatorConstants.ENABLE_CURRENT_LIMIT; 

    motorConfigs.Inverted = Constants.ElevatorConstants.MotorInverted;
    motorConfigs.NeutralMode = Constants.ElevatorConstants.MotorMode;

    
    elevmotor1.getConfigurator().apply(currentConfigs);
    elevmotor2.getConfigurator().apply(currentConfigs);
    elevmotor1.getConfigurator().apply(motorConfigs);
    elevmotor2.getConfigurator().apply(motorConfigs);
    // elevmotor1.getConfigurator().apply(limitConfigs);
    // elevmotor2.getConfigurator().apply(limitConfigs);
    elevmotor1.getConfigurator().apply(talonFXConfigs);
    elevmotor2.getConfigurator().apply(talonFXConfigs);
    
    //elevmotor1.config_kP(0, Constants.kElevatorP, Constants.TimeoutMs);
    //elevmotor1.config_kI(0, Constants.kElevatorI, Constants.TimeoutMs);
    //elevmotor1.config_kD(0, Constants.kElevatorD, Constants.TimeoutMs);
    //Probably not needed but just in case:
    // elevmotor1.config_kP(0, Constants.kElevatorP, Constants.TimeoutMs);
    // elevmotor1.config_kI(0, Constants.kElevatorI, Constants.TimeoutMs);
    // elevmotor1.config_kD(0, Constants.kElevatorD, Constants.TimeoutMs);
    
    // elevmotor2.config_kP(0, Constants.kElevatorP, Constants.TimeoutMs);
    // elevmotor2.config_kI(0, Constants.kElevatorI, Constants.TimeoutMs);
    // elevmotor2.config_kD(0, Constants.kElevatorD, Constants.TimeoutMs);
  }



  public double tickToRev(double tick){
    return tick * Constants.ElevatorConstants.GearRatio;
}

  public double tickToDeg(double tick){
      return tickToRev(tick) * 360;
  }

  public double DegToTick(double tick) {
    return tick / 360;
}
// trying to get it to figure out height of elevator based off motor postition. I think something is wrong.
// Instead, maybe figure out a constant from tick to distance up? Like multiply tick by a constant we figure out.

// seems to work fine to me, it should return the height, but just in case I would use getRaw() for PIDs and
// reserve this for smartdashboard
  public double getElevatorHeight(/*double tick*/) {
    return getRaw();
  }

  

  public double getRaw() {
    return elevmotor1.getPosition().getValueAsDouble();
  }

  public void setSpeed(double value) {
    if(c_ClimbSubsystem.getAngle() > Constants.ElevatorConstants.ClimbLimit){
    elevmotor1.setControl(m_request.withVelocity(MathUtil.clamp(value, Constants.ElevatorConstants.MinSpeed, 1) * Constants.ElevatorConstants.MaxVelocity));
    elevmotor2.setControl(m_request.withVelocity(MathUtil.clamp(value, Constants.ElevatorConstants.MinSpeed, 1) * Constants.ElevatorConstants.MaxVelocity));
    }else{
      elevmotor1.setControl(m_request.withVelocity(MathUtil.clamp(value, Constants.ElevatorConstants.MinSpeed, 0) * Constants.ElevatorConstants.MaxVelocity));
      elevmotor2.setControl(m_request.withVelocity(MathUtil.clamp(value, Constants.ElevatorConstants.MinSpeed, 0) * Constants.ElevatorConstants.MaxVelocity));
    }
  }

  public void set1(double speed){
    elevmotor1.set(speed);
  }

  public void set2(double speed){
    elevmotor2.set(speed);
  }

  public Command run(DoubleSupplier input){
    return this.runEnd(() -> this.setSpeed(MathUtil.applyDeadband(input.getAsDouble(), 0.1)), () -> this.setSpeed(0));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Height", getElevatorHeight());
    SmartDashboard.putNumber("ElevatorHeight2", elevmotor2.getPosition().getValueAsDouble());
    
    // This method will be called once per scheduler run
  }
}
