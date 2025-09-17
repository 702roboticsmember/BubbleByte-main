// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LimelightSubsystemRight;
import frc.robot.subsystems.Swerve;

public class AlignPath extends Command {
  boolean interrupted;

  private PIDController TranslatePID = new PIDController(

      Constants.AutoFollowConstants.kP,
      Constants.AutoFollowConstants.kI,
      Constants.AutoFollowConstants.kD);

  private PIDController RotatePID = new PIDController(
      Constants.AutoAimConstants.kP,
      Constants.AutoAimConstants.kI,
      Constants.AutoAimConstants.kD);
  private PIDController StrafePID = new PIDController(

      Constants.AutoFollowConstants.kP,
      Constants.AutoFollowConstants.kI,
      Constants.AutoFollowConstants.kD);
  
  
  DoubleSupplier TX;
  DoubleSupplier TZ;
  DoubleSupplier RY;
  BooleanSupplier tv;
  DoubleSupplier tx;
  Swerve s_Swerve;
  LimelightSubsystemRight l_LimelightSubsystem;
  Rotation2d headingprev;
  final double x;
  final double z;
  final double ry;
  

  /** Creates a new AutoAim. */
  public AlignPath(LimelightSubsystemRight l_LimelightSubsystem, double x, double z, double ry, Swerve s_Swerve) {
    this.TX = ()-> l_LimelightSubsystem.getTargetPos(0);
    this.TZ = ()-> l_LimelightSubsystem.getTargetPos(2);
    this.RY = ()-> l_LimelightSubsystem.getTargetPos(4);
    this.tv = ()-> l_LimelightSubsystem.IsTargetAvailable();
    this.tx = ()-> l_LimelightSubsystem.getTargetX();
    this.s_Swerve = s_Swerve;
    this.l_LimelightSubsystem = l_LimelightSubsystem;
    

    this.x = x;
    this.z = z;
    this.ry = ry;

    addRequirements(s_Swerve);
    addRequirements(l_LimelightSubsystem);
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.robotCentric = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double Tx =  l_LimelightSubsystem.getCameraPos(0);
    double Tz =  l_LimelightSubsystem.getCameraPos(2);
    double distance = new Translation2d(Tx, Tz).getDistance(new Translation2d(x, z));
    boolean Target =  l_LimelightSubsystem.IsTargetAvailable();
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   RobotContainer.robotCentric = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RotatePID.atSetpoint() && TranslatePID.atSetpoint() && StrafePID.atSetpoint();
  }
}
