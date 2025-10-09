// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LimelightSubsystemRight;
import frc.robot.subsystems.Swerve;

public class Tes extends Command {
  boolean interrupted;
  public static final PPHolonomicDriveController PATHPLANNER_FOLLOWER_CONFIG = new PPHolonomicDriveController(
                new PIDConstants(5.2, 3, 0), 
                new PIDConstants(3, 0, 0)
                // MAX_SPEED,
                // DRIVEBASE_RADIUS,
                // new ReplanningConfig()
                );

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
  
  
   Swerve s_Swerve;
   LimelightSubsystemRight l_LimelightSubsystem;
  // Rotation2d headingprev;
  // final double x;
  // final double z;
  // final double ry;
  final LTVUnicycleController controller = new LTVUnicycleController(VecBuilder.fill(0.0625, 0.125, 2.0), VecBuilder.fill(1.0, 2.0), 0.02, 9);
  /* 
  Pose2d Start = new Pose2d(Units.feetToMeters(1.54), Units.feetToMeters(23.23),
  Rotation2d.fromDegrees(-180));

  Pose2d End = new Pose2d(Units.feetToMeters(23.7), Units.feetToMeters(6.8),
        Rotation2d.fromDegrees(-160));
        */
  Pose2d Start;

  Pose2d End;

  Trajectory trajectory;

  double startTime;

  double currentTime;

  double distanceToEnd;
  
  int Id;
  
  /** Creates a new AutoAim. */
  public Tes(Pose2d Start, Pose2d End, Swerve s_Swerve) {
    
    this.s_Swerve = s_Swerve;
    //this.l_LimelightSubsystem = l_LimelightSubsystem;
    this.Start = Start;
    this.End = End;
    RobotContainer.robotCentric = false;
    double angle = new Translation2d(Start.relativeTo(End).getX(), Start.relativeTo(End).getX()).getAngle().getDegrees();
    
    
    

    //var interiorWaypoints = new ArrayList<Translation2d>();
    //interiorWaypoints.add(new Translation2d(End.getX() + Constants.AlignConstants.XOffset,End.getY() + Constants.AlignConstants.YOffset));
    //interiorWaypoints.add(new Translation2d(Units.feetToMeters(21.04), Units.feetToMeters(18.23)));

    TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(2), Units.feetToMeters(2));
    config.setReversed(false);
    List<Pose2d> waypoints = new ArrayList<Pose2d>();
    waypoints.add(new Pose2d(Start.getX(), Start.getY(), new Rotation2d(45)));
    waypoints.add(new Pose2d(End.getX(), End.getY(), new Rotation2d(45)));
    

    this.trajectory = TrajectoryGenerator.generateTrajectory(
        waypoints,
        config);
      
    
    addRequirements(s_Swerve);
    
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    s_Swerve.setAlignPose(Start);
    //Id = s_Swerve.setAlignPoseAdjustedtoLimelight();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentTime = Timer.getFPGATimestamp() - startTime;
    
    Pose2d alignpose = s_Swerve.getAlignPose();
    Trajectory.State reference = trajectory.sample(currentTime);
    //PathPlannerTrajectoryState state = new PathPlannerTrajectoryState().;
     // sample the trajectory at 3.4 seconds from the beginning
    
  ChassisSpeeds adjustedSpeeds = controller.calculate(new Pose2d(alignpose.getX(), alignpose.getY(), new Rotation2d(reference.poseMeters.getRotation().getRadians())), reference);//s_Swerve.getAlignPoseAdjusted(Id)
  //ChassisSpeeds speeds = PATHPLANNER_FOLLOWER_CONFIG.calculateRobotRelativeSpeeds(alignpose, new Pose2d(reference.poseMeters.getX(), reference.poseMeters.getY(), new Rotation2d(0)));
  //s_Swerve.move(speeds, 0, false);
    SmartDashboard.putNumber("Aposex", alignpose.getX());
    SmartDashboard.putNumber("Aposey", alignpose.getY());
    SmartDashboard.putNumber("Aposea", alignpose.getRotation().getDegrees());
    SmartDashboard.putNumber("Tposex", trajectory.getInitialPose().getX());
    SmartDashboard.putNumber("Tposey", trajectory.getInitialPose().getY());
    SmartDashboard.putNumber("Tposea", trajectory.getInitialPose().getRotation().getDegrees());
    SmartDashboard.putNumber("Rposex", reference.poseMeters.getX());
    SmartDashboard.putNumber("Rposey", reference.poseMeters.getY());
    SmartDashboard.putNumber("Rposea", reference.poseMeters.getRotation().getDegrees());
    SmartDashboard.putNumber("Rposec", reference.curvatureRadPerMeter);
    SmartDashboard.putNumber("Aangle", adjustedSpeeds.omegaRadiansPerSecond);
    SmartDashboard.putNumber("Avx", adjustedSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Avy", adjustedSpeeds.vyMetersPerSecond);
    double duration = trajectory.getTotalTimeSeconds();
   
    //double distance = new Translation2d(Tx, Tz).getDistance(new Translation2d(x, z));
    //boolean Target =  l_LimelightSubsystem.IsTargetAvailable();
    distanceToEnd = new Translation2d(s_Swerve.getAlignPose().getX(), s_Swerve.getAlignPose().getY()).getDistance(new Translation2d(End.getX(),End.getY()));


    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   RobotContainer.robotCentric = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return trajectory.getTotalTimeSeconds() <= currentTime;
  }
}
