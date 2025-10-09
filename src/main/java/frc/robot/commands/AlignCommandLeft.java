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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.LimelightSubsystemRight;
import frc.robot.subsystems.Swerve;

public class AlignCommandLeft extends Command {
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
  public AlignCommandLeft(LimelightSubsystemRight l_LimelightSubsystem2, double x, double z, double ry, Swerve s_Swerve) {
    this.TX = ()-> l_LimelightSubsystem2.getTargetPos(0);
    this.TZ = ()-> l_LimelightSubsystem2.getTargetPos(2);
    this.RY = ()-> l_LimelightSubsystem2.getTargetPos(4);
    this.tv = ()-> l_LimelightSubsystem2.IsTargetAvailable();
    this.tx = ()-> l_LimelightSubsystem2.getTargetX();
    this.s_Swerve = s_Swerve;
    this.l_LimelightSubsystem = l_LimelightSubsystem2;
    

    this.x = x;
    this.z = z;
    this.ry = ry;

    addRequirements(s_Swerve);
    addRequirements(l_LimelightSubsystem2);
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
    // boolean inzone = Math.abs(Tx) < 0.3;
    
    TranslatePID.setSetpoint(x);
    TranslatePID.setTolerance(0.01);
    StrafePID.setSetpoint(z);
    StrafePID.setTolerance(0.01);
    RotatePID.setSetpoint(ry);
    RotatePID.setTolerance(1);

    

    SmartDashboard.putNumber("distance", distance);
    

    
    boolean Target =  l_LimelightSubsystem.IsTargetAvailable();
    double value = TranslatePID.calculate(Tx);
    //double result = Math.copySign(Math.abs(value) + 0.01, value); 
    double Tranlate = (Target && !TranslatePID.atSetpoint()  ? MathUtil.clamp(value, -0.47, 0.47) : 0);
    SmartDashboard.putNumber("TPID", value);
    SmartDashboard.putNumber("TTX", x);

    
    double value1 = StrafePID.calculate(Tz);
    //double result1 = Math.copySign(Math.abs(value1) + 0.0955, value1); 
    double Strafe = (Target && !StrafePID.atSetpoint()? MathUtil.clamp(value1, -0.47, 0.47) : 0);
    SmartDashboard.putNumber("SPID", value1);
    SmartDashboard.putNumber("STZ", z);
    //Cameron Trux Team 702 :3
    //double angle = Math.tanh(x/z);

    double a =  l_LimelightSubsystem.getTargetPos(4);
    double tx = l_LimelightSubsystem.getTargetX();
    double value2 =  RotatePID.calculate(a);
    //double result2 = Math.copySign(Math.abs(value2) + 0.0955, value2); 
    double Rotate = (Target && !RotatePID.atSetpoint() ? MathUtil.clamp(value2, -0.17, 0.17) : 0);
    
    // if(Rotate > 0){
    //   if(tx.getAsDouble() > 12){
    //     Rotate = 0;
    //   }
    // }
    // if(Rotate < 0){
    //   if(tx.getAsDouble() < -17){
    //     Rotate = 0;
    //   }
    // }
    SmartDashboard.putNumber("RRY", a);
    SmartDashboard.putNumber("RPID", Rotate);
    s_Swerve.drive(
                new Translation2d(Strafe, -Tranlate).times(Constants.Swerve.MAX_SPEED),
                Rotate * Constants.Swerve.MAX_ANGULAR_VELOCITY,
                !true,
                true);
                SmartDashboard.putNumber("RRPID", Rotate* (1 + Strafe));

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
