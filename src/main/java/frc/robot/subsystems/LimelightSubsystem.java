// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
  public NetworkTable table;
  public NetworkTableEntry tx;
  public NetworkTableEntry ty;
  public NetworkTableEntry tv;
  public NetworkTableEntry ta;
  public NetworkTableEntry pipeline;
  public NetworkTableEntry getpipe;
  public NetworkTableEntry tclass;
  public NetworkTableEntry botpose_wpiblue;
  public NetworkTableEntry botpose_wpired;
  public NetworkTableEntry botpose;
  public NetworkTableEntry camMode;
  public NetworkTableEntry targetpose_cameraspace;
  public NetworkTableEntry camerapose_targetspace;
  public NetworkTableEntry botpose_orb;
  public NetworkTableEntry botpose_targetspace;
  public NetworkTableEntry tid;

  public NetworkTable righttable;
  public NetworkTableEntry righttx;
  public NetworkTableEntry rightty;
  public NetworkTableEntry righttv;
  public NetworkTableEntry rightta;
  public NetworkTableEntry rightpipeline;
  public NetworkTableEntry rightgetpipe;
  public NetworkTableEntry righttclass;
  public NetworkTableEntry rightbotpose_wpiblue;
  public NetworkTableEntry rightbotpose_wpired;
  public NetworkTableEntry rightbotpose;
  public NetworkTableEntry rightcamMode;
  public NetworkTableEntry righttargetpose_cameraspace;
  public NetworkTableEntry rightcamerapose_targetspace;
  public NetworkTableEntry rightbotpose_orb;
  public NetworkTableEntry rightbotpose_targetspace;
  public NetworkTableEntry righttid;

  /** Creates a new LimelightSubsystem. */
  public LimelightSubsystem() {
    table = NetworkTableInstance.getDefault().getTable("limelight-front");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    tv = table.getEntry("tv");
    pipeline = table.getEntry("pipeline");
    getpipe = table.getEntry("getpipe");
    tclass = table.getEntry("tclass");
    camMode = table.getEntry("camMode");
    botpose = table.getEntry("botpose");
    botpose_wpiblue = table.getEntry("botpose_wpiblue");
    botpose_wpired = table.getEntry("botpose_wpired");
    targetpose_cameraspace = table.getEntry("targetpose_cameraspace");
    camerapose_targetspace = table.getEntry("camerapose_targetspace");
    botpose_orb = table.getEntry("botpose_orb");
    botpose_targetspace = table.getEntry("botpose_targetspace");

    righttable = NetworkTableInstance.getDefault().getTable("limelight-front");
    righttx = righttable.getEntry("tx");
    rightty = righttable.getEntry("ty");
    rightta = righttable.getEntry("ta");
    righttv = righttable.getEntry("tv");
    rightpipeline = righttable.getEntry("pipeline");
    rightgetpipe = righttable.getEntry("getpipe");
    righttclass = righttable.getEntry("tclass");
    rightcamMode = righttable.getEntry("camMode");
    rightbotpose = righttable.getEntry("botpose");
    rightbotpose_wpiblue = righttable.getEntry("botpose_wpiblue");
    rightbotpose_wpired = righttable.getEntry("botpose_wpired");
    righttargetpose_cameraspace = righttable.getEntry("targetpose_cameraspace");
    rightcamerapose_targetspace = righttable.getEntry("camerapose_targetspace");
    rightbotpose_orb = righttable.getEntry("botpose_orb");
    rightbotpose_targetspace = righttable.getEntry("botpose_targetspace");

    NetworkTableInstance.getDefault().getTable("limelight-front").getEntry("imumode_set").setNumber(0);
    
    tid = table.getEntry("tid");
  }

  public double getTargetX() {
    return tx.getNumber(0).doubleValue();
  }

  public double getTargetY() {
    return ty.getNumber(0).doubleValue();
  }

  public double getTargetA() {
    return ta.getNumber(0).doubleValue();
  }

  public boolean IsTargetAvailable() {
    return tv.getNumber(0).intValue() == 1 ? true : false;
  }

  public void setPipeline(int value) {
    pipeline.setNumber(value);
  }

  public int getPipeline() {
    return getpipe.getNumber(0).intValue();
  }

  public int getClassifier() {
    return tclass.getNumber(0).intValue();
  }

  public double getBotPoseX() {
    double pose[] = botpose.getDoubleArray(new double[6]);
    return pose[0];
  }

  public double getBotPoseY() {
    double pose[] = botpose.getDoubleArray(new double[6]);
    return pose[1];
  }

  public double[] getBotPoseTeamRelative() {
    return botpose_wpiblue.getDoubleArray(new double[7]);
    // var alliance = DriverStation.getAlliance();
    //   if (alliance.isPresent()) {
    //     if(alliance.get() == DriverStation.Alliance.Red){
          
    //       return botpose_wpired.getDoubleArray(new double[7]);
    //     }else{
    //       return botpose_wpiblue.getDoubleArray(new double[7]);
    //     }
    //   }
    //   return new double[7];
  }

  public Pose2d getBotPose2d(){
    double[] pose = getBotPoseTeamRelative();
    Pose2d botpose = pose.equals(new double[7]) || !IsTargetAvailable()? null: new Pose2d(pose[0], pose[1], new Rotation2d(pose[5]));
    return botpose;
  }

  /**
   * 3D transform of the robot in the coordinate system of the primary in-view AprilTag (array (6))
   * @return Returns a double array of 6 [tx, ty, tz, pitch, yaw, roll] (meters, degrees)
   */
  public double[] getBotPose_TargetSpace(){
    double[] pose = botpose_targetspace.getDoubleArray(new double[6]);
    return pose;
  }

  /**
   * ID of the primary in-view AprilTag
   * @return double but ID should be an integer if there is no value it will send -1.
   */
  public double getTid(){
    return tid.getDouble(-1);
  }


  // public double getBotPoseYTeamRelative() {
  //   double pose[] = RobotContainer.color == Color.kRed ? botpose_wpired.getDoubleArray(new double[6])
  //       : botpose_wpiblue.getDoubleArray(new double[6]);
  //   return pose[1];
  // }

  

  public double getTargetPos(int value){
    double pos[] = targetpose_cameraspace.getDoubleArray(new double[6]);
    //double posR[] = targetpose_cameraspace.getDoubleArray(new double[6]);
    return pos[value];
  }

  public double getCameraPos(int value){
    double pos[] = camerapose_targetspace.getDoubleArray(new double[6]);
    return pos[value];
  }

  public double TargetDistance(){
    return Math.sqrt(Math.pow(getTargetPos(0), 2) + Math.pow(getTargetPos(1), 2));
  }

  public void setCamMode(int value) {
    camMode.setDouble(value);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("LLX", getTargetPos(0));
    SmartDashboard.putNumber("LLZ", getTargetPos(2));
    SmartDashboard.putNumber("LLRY", getTargetPos(4));
    

    SmartDashboard.putNumber("tclass", getClassifier());
    SmartDashboard.putNumber("BotPoseX", getBotPoseX());
    SmartDashboard.getNumberArray("Limelightposeeeee", getBotPoseTeamRelative());
    SmartDashboard.getNumber("LimelightposeX", getBotPose2d()== null? 5: getBotPose2d().getX());
  }
}
