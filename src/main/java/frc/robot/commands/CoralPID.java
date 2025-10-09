// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CoralIntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralPID extends Command {
  private PIDController coralPID = new PIDController (
    // 1/22/25 PID constants will be added later
    Constants.CoralIntakeConstants.kP,
    Constants.CoralIntakeConstants.kI,
    Constants.CoralIntakeConstants.kD
  );


  public CoralIntakeSubsystem c_CoralIntakeSubsystem;
  public boolean stay;
  public double setpoint;
  public double set;
  /** Creates a new CoralPID. will keep current position*/
  public CoralPID(CoralIntakeSubsystem c_CoralIntakeSubsystem) {
    this.c_CoralIntakeSubsystem = c_CoralIntakeSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    this.stay = true;
    
    addRequirements(c_CoralIntakeSubsystem);
  }
  /** Creates a new CoralPID. 
   * 
   * @param stepoint will go to certain position or setpoint*/
  public CoralPID(CoralIntakeSubsystem c_CoralIntakeSubsystem, double stepoint){
    this.c_CoralIntakeSubsystem = c_CoralIntakeSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    this.set = setpoint;
    this.stay = false;
    addRequirements(c_CoralIntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(stay){
      setpoint = c_CoralIntakeSubsystem.getPosition();
    }else{
      setpoint = c_CoralIntakeSubsystem.getPosition() + set;
    }

    coralPID.setSetpoint(setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
 
      c_CoralIntakeSubsystem.setSpeed(coralPID.calculate(c_CoralIntakeSubsystem.getPosition()));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    c_CoralIntakeSubsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
