package frc.robot.subsystems;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.SwerveModule;

/**
 * Our main drive subsystem
 */
public class Swerve extends SubsystemBase {
    
    public SwerveDriveOdometry swerveOdometry;
    public SwerveDriveOdometry AlignOdometry;
    
    public SwerveModule[] swerveModules;
    public AHRS gyro;
    public  RobotConfig config;
    public LimelightSubsystemLeft l_LimelightSubsystemLeft;
    public LimelightSubsystemRight l_LimelightSubsystemRight;

    public Swerve(LimelightSubsystemLeft l_LimelightSubsystemLeft, LimelightSubsystemRight l_LimelightSubsystemRight) {
        this.l_LimelightSubsystemLeft = l_LimelightSubsystemLeft;
        this.l_LimelightSubsystemRight = l_LimelightSubsystemRight;
        gyro = new AHRS( NavXComType.kMXP_SPI);
        
        
        gyro.reset();
        try {
            config = RobotConfig.fromGUISettings();
        } catch (IOException | ParseException e) {
            e.printStackTrace();
        }

        swerveModules = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.Mod0.constants),
                new SwerveModule(1, Constants.Swerve.Mod1.constants),
                new SwerveModule(2, Constants.Swerve.Mod2.constants),
                new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        

        
    
        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.KINEMATICS, getGyroYaw(), getModulePositions());
        AlignOdometry = new SwerveDriveOdometry(Constants.Swerve.KINEMATICS, getGyroYaw(), getModulePositions());

        AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            Constants.Swerve.PATHPLANNER_FOLLOWER_CONFIG,
            config,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
      
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                      return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                  },
                  this // Reference to this subsystem to set requirements
          );
    }

    public Command followPathPlanner(PathPlannerPath path){
        SmartDashboard.putBoolean("iscalled", true);
        return new FollowPathCommand(
                path,
                this::getAlignPose, // Robot pose supplier
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds, AND feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                ),
                config, // The robot configuration
                () -> false,
                this);
    
  
    }

    //AutoBuilder.

    public void driveRobotRelative(ChassisSpeeds speeds) {
        var swerveModuleStates = Constants.Swerve.KINEMATICS.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, Constants.Swerve.MAX_SPEED);

        for (int i = 0; i < swerveModuleStates.length; i++) {
            swerveModules[i].setDesiredState(swerveModuleStates[i], false);
        }
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return Constants.Swerve.KINEMATICS.toChassisSpeeds(getModuleStates());
    }

    private void resetPose(Pose2d startingPosition) {
        swerveOdometry.resetPosition(
                //new Rotation2d(Math.toRadians(gyro.getAngle())),
                getGyroYaw(),
                this.getModulePositions(),
                startingPosition);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.KINEMATICS.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getHeading())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.MAX_SPEED);

        for (SwerveModule mod : swerveModules) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], false);
        }
    }

    public void move(ChassisSpeeds chassisSpeeds, double rotation, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.KINEMATICS.toSwerveModuleStates(new ChassisSpeeds(
            chassisSpeeds.vxMetersPerSecond,
            chassisSpeeds.vyMetersPerSecond,
            rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.MAX_SPEED);

        for (SwerveModule mod : swerveModules) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], false);
        }
    }


    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.MAX_SPEED);

        for (SwerveModule mod : swerveModules) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : swerveModules) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : swerveModules) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public Pose2d getAlignPose() {
        
        return AlignOdometry.getPoseMeters();
    }

    /**
     * 
     * @param  Id april tag number ID
     * @return Pose2d of current robot postion based on april tag.
     */
    public Pose2d getAlignPoseAdjusted(int Id){
        int LId = (int)l_LimelightSubsystemLeft.getTid();
        int RId = (int)l_LimelightSubsystemRight.getTid();
        double[] RightPose = l_LimelightSubsystemRight.getBotPose_TargetSpace();
        double[] LeftPose = l_LimelightSubsystemLeft.getBotPose_TargetSpace();
        Pose2d odometrypose = AlignOdometry.getPoseMeters();
        if(Id == LId && Id == RId){
            double x = (odometrypose.getX() + RightPose[2] + LeftPose[2])/3;
            double y = (odometrypose.getY() + RightPose[0] + LeftPose[0])/3;
            double yaw = (odometrypose.getRotation().getDegrees() + RightPose[4] + LeftPose[4])/3;

            return new Pose2d(x, y, new Rotation2d(yaw));

        }else if(Id == LId){
            double x = (odometrypose.getX() + LeftPose[2])/2;
            double y = (odometrypose.getY() + LeftPose[0])/2;
            double yaw = (odometrypose.getRotation().getDegrees() + LeftPose[4])/2;
            return new Pose2d(x, y, new Rotation2d(yaw));
        }else if(Id == RId){
            double x = (odometrypose.getX() + RightPose[2])/2;
            double y = (odometrypose.getY() + RightPose[0])/2;
            double yaw = (odometrypose.getRotation().getDegrees() + RightPose[4])/2;
            return new Pose2d(x, y, new Rotation2d(yaw));
        }else{
            return odometrypose;
        }
    }


    public int setAlignPoseAdjustedtoLimelight(){
        int Id = (int)l_LimelightSubsystemLeft.getTid();
        int RId = (int)l_LimelightSubsystemRight.getTid();
        double[] RightPose = l_LimelightSubsystemRight.getBotPose_TargetSpace();
        double[] LeftPose = l_LimelightSubsystemLeft.getBotPose_TargetSpace();
        if(Id == RId){
            double x = (LeftPose[2] + RightPose[2])/2;
            double y = (LeftPose[0] + RightPose[0])/2;
            double yaw = (LeftPose[4] + RightPose[4])/2;
            AlignOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(x, y, new Rotation2d(yaw)));
        }else{
            double x = LeftPose[2];
            double y = LeftPose[0];
            double yaw = LeftPose[4];
            AlignOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(x, y, new Rotation2d(yaw)));
        }
        return Id;
    }


    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public void setAlignPose(Pose2d pose) {
        AlignOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
        
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading() {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        return (Constants.Swerve.INVERT_GYRO) ? Rotation2d.fromDegrees((360- gyro.getYaw()))
                : Rotation2d.fromDegrees(gyro.getYaw());
    }

    // public Pose2d getRobotPose(){
    //     Translation2d offset = new Translation2d(Constants.Swerve.NAVX_X, Constants.Swerve.NAVX_Y);
    //     Rotation2d angleOffset = new Rotation2d(offset.getX(), offset.getY());
    //     double angle = getGyroYaw().getRadians() + angleOffset.getRadians();
        
    //     double x = gyro.getDisplacementX() + (Math.cos(angle)* offset.getNorm());
    //     double y = gyro.getDisplacementY()+ (Math.sin(angle)* offset.getNorm());
    //     return new Pose2d(x, y, getGyroYaw());
    // }

    public void resetGyroPose(){
        gyro.resetDisplacement();
    }

    public double getAcc() {
        return gyro.getAccelFullScaleRangeG();
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : swerveModules) {
            mod.resetToAbsolute();
        }
    }

    public void setPoseToReef(String reefpose){
        var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                      if(alliance.get() == DriverStation.Alliance.Red){
                        if(reefpose.toLowerCase().equals("c")){
                            setPose(new Pose2d(Constants.Swerve.fieldlengthX - Constants.Swerve.cx, Constants.Swerve.fieldlengthY -Constants.Swerve.cy, getHeading()));//3.65, 3
                            //SmartDashboard.putBoolean("posereefresetc", true);
                        }
                        if(reefpose.toLowerCase().equals("d")){
                            setPose(new Pose2d(Constants.Swerve.fieldlengthX - Constants.Swerve.dx, Constants.Swerve.fieldlengthY - Constants.Swerve.dy, getHeading()));//4, 2.8
                            //SmartDashboard.putBoolean("posereefresetd", true);
                        }
                        if(reefpose.toLowerCase().equals("e")){
                            setPose(new Pose2d(Constants.Swerve.fieldlengthX - Constants.Swerve.ex, Constants.Swerve.fieldlengthY - Constants.Swerve.ey, getHeading()));//5, 2.8
                            //SmartDashboard.putBoolean("posereefresete", true);
                        }
                        if(reefpose.toLowerCase().equals("j")){
                            setPose(new Pose2d(Constants.Swerve.fieldlengthX - Constants.Swerve.jx, Constants.Swerve.fieldlengthY - Constants.Swerve.jy, getHeading()));//5, 2.8
                            //SmartDashboard.putBoolean("posereefresete", true);
                        }
                        if(reefpose.toLowerCase().equals("k")){
                            setPose(new Pose2d(Constants.Swerve.fieldlengthX - Constants.Swerve.kx, Constants.Swerve.fieldlengthY - Constants.Swerve.ky, getHeading()));//5, 2.8
                            //SmartDashboard.putBoolean("posereefresete", true);
                        }
                        if(reefpose.toLowerCase().equals("l")){
                            setPose(new Pose2d(Constants.Swerve.fieldlengthX - Constants.Swerve.lx, Constants.Swerve.fieldlengthY - Constants.Swerve.ly, getHeading()));//5, 2.8
                            //SmartDashboard.putBoolean("posereefresete", true);
                        }

                      }else{
                        if(reefpose.toLowerCase().equals("c")){
                            setPose(new Pose2d(Constants.Swerve.cx, Constants.Swerve.cy, getHeading()));//3.65, 3
                            //SmartDashboard.putBoolean("posereefresetc", true);
                        }
                        if(reefpose.toLowerCase().equals("d")){
                            setPose(new Pose2d(Constants.Swerve.dx, Constants.Swerve.dy, getHeading()));//4, 2.8
                            //SmartDashboard.putBoolean("posereefresetd", true);
                        }
                        if(reefpose.toLowerCase().equals("e")){
                            setPose(new Pose2d(Constants.Swerve.ex, Constants.Swerve.ey, getHeading()));//5, 2.8
                            //SmartDashboard.putBoolean("posereefresete", true);
                        }
                        if(reefpose.toLowerCase().equals("j")){
                            setPose(new Pose2d(Constants.Swerve.jx, Constants.Swerve.jy, getHeading()));//4, 2.8
                            //SmartDashboard.putBoolean("posereefresetd", true);
                        }
                        if(reefpose.toLowerCase().equals("k")){
                            setPose(new Pose2d(Constants.Swerve.kx, Constants.Swerve.ky, getHeading()));//4, 2.8
                            //SmartDashboard.putBoolean("posereefresetd", true);
                        }
                        if(reefpose.toLowerCase().equals("l")){
                            setPose(new Pose2d(Constants.Swerve.lx, Constants.Swerve.ly, getHeading()));//5, 2.8
                            //SmartDashboard.putBoolean("posereefresete", true);
                        }
                      }
                    }
                    
        
        
    }

    public void updatePoseLimelight(){
        Pose2d poseb = l_LimelightSubsystemRight.getBotPose2d();
        Pose2d posef = l_LimelightSubsystemLeft.getBotPose2d();
        
        SmartDashboard.putBoolean("Limelight poseUpF", false);
        if(posef == null){
            SmartDashboard.putBoolean("Limelight null", true);
        }
        if(posef != null && posef.getX() != 0 && posef.getY() != 0){
            SmartDashboard.putNumber("Limelight poseX", posef.getX());
        SmartDashboard.putNumber("Limelight poseY", posef.getY());
            setPose(new Pose2d(posef.getX(), posef.getY(), getHeading()));
            SmartDashboard.putBoolean("Limelight poseUpF", true);
        }
        else if(poseb != null && poseb.getX() != 0 && poseb.getY() != 0){
            setPose(new Pose2d(poseb.getX(), poseb.getY(), getHeading()));
            SmartDashboard.putBoolean("Limelight poseUpB", true);
        }
        // if(poseb != null && poseb.getX() != 0 && poseb.getY() != 0 && posef != null && posef.getX() != 0 && posef.getY() != 0){
        //     setPose(new Pose2d((poseb.getX() + posef.getX())/2, (poseb.getY() + posef.getY())/2, getHeading()));
        // }else{
        //     if(posef != null && posef.getX() != 0 && posef.getY() != 0){
        //         setPose(new Pose2d(posef.getX(), posef.getY(), getHeading()));
        //     }
        //     if(poseb != null && poseb.getX() != 0 && poseb.getY() != 0){
        //         setPose(new Pose2d(poseb.getX(), poseb.getY(), getHeading()));
        //     }
        // }
    }

    @Override
    public void periodic() {
        swerveOdometry.update(getGyroYaw(), getModulePositions());
        AlignOdometry.update(getGyroYaw(), getModulePositions());
        //updatePoseLimelight();
        
        SmartDashboard.putNumber("Acc",this.getAcc());
        SmartDashboard.putNumber("gyrow", gyro.getYaw());
        //SmartDashboard.putNumber("gyroYaw", gyro.getgy);
        //SmartDashboard.putNumber("gyrorol", gyro.getRoll());
        //SmartDashboard.putNumber("gyropitch", gyro.getYaw());

        SmartDashboard.putNumber("heading", this.getPose().getRotation().getDegrees());
        RobotContainer.field.setRobotPose(getPose());
        SmartDashboard.putNumber("SwervePoseX", swerveOdometry.getPoseMeters().getX());
        SmartDashboard.putNumber("SwervePoseY", swerveOdometry.getPoseMeters().getY());
        // SmartDashboard.putNumber("llPoseX", l_LimelightBackSubsystem.getBotPose2d().getX());
        // SmartDashboard.putNumber("llPoseY", l_LimelightBackSubsystem.getBotPose2d().getX());

        //SmartDashboard.putNumber("posex", );
        
        for (SwerveModule mod : swerveModules) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Speed", mod.getAngleSpeed());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }
    }
}