package frc.robot;

import static frc.lib.util.COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

//import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
//import com.pathplanner.lib.util.PIDConstants;
// import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios;
import frc.lib.util.SwerveModuleConstants;
public final class Constants {
    public static final double CONTROLLER_DEADBAND = 0.05;

    /**
     * Corresponds to port zero on the Roborio DIO. 
     */
    public static final int LIMIT_SWITCH_INTAKE = 9;

    public static final int BEAM_LED_ID = 1;

    public static final class Swerve {
        /**
         * Whether gyroscope values should be inverted.
         */
        public static final boolean INVERT_GYRO = true;

        /**
         * Constants for the motor setup that we're using.
         */
        public static final COTSTalonFXSwerveConstants FALCON_500_CONSTANTS = Falcon500(driveRatios.L1);

        
        //

        /**
         * Units: Meters
         */
        public static final double TRACK_WIDTH = Units.inchesToMeters(23.5);//bubble byte 21.75 

        /**
         * Units: Meters
         */
        public static final double BASE_WIDTH = Units.inchesToMeters(23.5);//bubble byte 21.75 

        /**
         * Units: Meters
         */
        public static final double DRIVEBASE_DIAMETER = Math.sqrt(TRACK_WIDTH * TRACK_WIDTH + BASE_WIDTH * BASE_WIDTH);

        /**
         * Units: Meters
         */
        public static final double DRIVEBASE_RADIUS = DRIVEBASE_DIAMETER / 2f;

        public static final double WHEEL_CIRCUMFERENCE = 0.050 * Math.PI * 2;///KRAKEN_X60_CONSTANTS.wheelCircumference;

        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(BASE_WIDTH / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(BASE_WIDTH / 2.0, -TRACK_WIDTH / 2.0),
                new Translation2d(-BASE_WIDTH / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(-BASE_WIDTH / 2.0, -TRACK_WIDTH / 2.0));
                
        public static final double NAVX_Y = 0;
        public static final double NAVX_X = 0.17;
                        
        
//         DifferentialDriveKinematics kinematics =
//   new DifferentialDriveKinematics(Units.inchesToMeters(27.0));

        /* Module Gear Ratios */
        public static final double DRIVE_GEAR_RATIO = 5.36;//L1 7.13 - L2 5.9 - L3 5.36
        public static final double ANGLE_GEAR_RATIO = 18.75;

        public static final InvertedValue ANGLE_MOTOR_INVERT = FALCON_500_CONSTANTS.angleMotorInvert;
        public static final InvertedValue DRIVE_MOTOR_INVERT = FALCON_500_CONSTANTS.driveMotorInvert;

        public static final SensorDirectionValue CANCODER_INVERT = FALCON_500_CONSTANTS.cancoderInvert;


        public static final double fieldlengthX = 17.55;//(meters)
        public static final double fieldlengthY = 8.05;//(meters)

        public static final double ax = 0;
        public static final double ay = 0;
        public static final double bx = 0;
        public static final double by = 0;
        public static final double cx = 3.8;
        public static final double cy = 2.950;
        public static final double dx = 4.050;
        public static final double dy = 2.790;
        public static final double ex = 5.025;
        public static final double ey = 2.84;
        public static final double fx = 0;
        public static final double fy = 0;
        public static final double gx = 0;
        public static final double gy = 0;
        public static final double hx = 0;
        public static final double hy = 0;
        public static final double ix = 0;
        public static final double iy = 0;
        public static final double jx = 4.9;
        public static final double jy = 5.27;
        public static final double kx = 3.9;
        public static final double ky = 5.180;
        public static final double lx = 3.6;
        public static final double ly = 5;
        
        /**
         * Units: Volts
         */
        public static final int ANGLE_STATOR_CURRENT_LIMIT = 40;
        public static final int ANGLE_CURRENT_LIMIT = 25;
        public static final int ANGLE_CURRENT_THRESHOLD = 40;
        public static final double ANGLE_CURRENT_THRESHOLD_TIME = 0.1;
        public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;
        public static final boolean ANGLE_ENABLE_STATOR_CURRENT_LIMIT = true;

        public static final int DRIVE_STATOR_CURRENT_LIMIT = 50;
        public static final int DRIVE_CURRENT_LIMIT = 35;//35
        public static final int DRIVE_CURRENT_THRESHOLD = 50;//60
        public static final double DRIVE_CURRENT_THRESHOLD_TIME = 0.1;
        public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;
        public static final boolean DRIVE_ENABLE_STATOR_CURRENT_LIMIT = true;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        public static final double OPEN_LOOP_RAMP = 0.35;
        public static final double CLOSED_LOOP_RAMP = 0;

        public static final PIDConstants ANGLE_PID = new PIDConstants(FALCON_500_CONSTANTS.angleKP,
                FALCON_500_CONSTANTS.angleKI, FALCON_500_CONSTANTS.angleKD);
        public static final PIDConstants DRIVE_PID = new PIDConstants(0.12, 0.0, 0.0);

        /* Drive Motor Characterization Values From SYSID */
        public static final double DRIVE_KS = 0.32;
        public static final double DRIVE_KV = 1.51;
        public static final double DRIVE_KA = 0.27;

        public static final PathConstraints constraints = new PathConstraints(
        1, 2.5,
        Units.degreesToRadians(360), Units.degreesToRadians(360));

        /** Units: m/s */
        public static final double MAX_SPEED = 7;
        /** Units: radians/s */
        public static final double MAX_ANGULAR_VELOCITY = 8.0;

        /* Neutral Modes */
        public static final NeutralModeValue ANGLE_NEUTRAL_MODE = NeutralModeValue.Brake;
        public static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-1.49 +180);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 6;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(26.104);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-122.6953 + 180);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 10;
            public static final int angleMotorID = 11;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(144.580);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }
        public static final PPHolonomicDriveController PATHPLANNER_FOLLOWER_CONFIG = new PPHolonomicDriveController(
                new PIDConstants(5.2, 3, 0), 
                new PIDConstants(3, 0, 0)
                // MAX_SPEED,
                // DRIVEBASE_RADIUS,
                // new ReplanningConfig()
                );
        //public static final Rotation2d NAVX_A = null;

        public static final double MIN_SPEED = 1;

    }

    public static final class ClimberConstants {
        public static final int MotorID = 13;

        public static final InvertedValue MotorInverted = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue LiftMotorMode = NeutralModeValue.Brake;
        
        public static final double MaxLiftSpeed = 0.45;

        public static final double kP = 0.07;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final double Tolerance = 0.1;

        // public static boolean LiftLimitEnable = true;
        // public static final double LiftPIDTolerance = .5;
        // public static final double LeftLiftPosInValue = -41.7;
        // public static final double LeftLiftPosOutValue = 0;   
        // public static final double RightLiftPosInValue = -13;
        // public static final double RightLiftPosOutValue = 0;     

        public static final int STATOR_CURRENT_LIMIT = 200;
        public static final int CURRENT_LIMIT = 100;
        public static final boolean ENABLE_CURRENT_LIMIT = true;
        public static final boolean ENABLE_STATOR_CURRENT_LIMIT = true;

        public static final double ForwardLimit = 80;

        public static final boolean LimitEnable = true;

        public static final double ReverseLimit = -92;

        public static final double DefaultPose = 0;


        public static final double OutPose = 90;

        public static final double InPose = -90;

        public static final double GearRatio = 1/338.5;

        public static final double ElevatorLimit = 4;

        // public static final int CURRENT_THRESHOLD = 35;
        // public static final double CURRENT_THRESHOLD_TIME = 0.1;
    }


    public static final class AutoAimConstants {
        public static final double kP = 0.004537;
        public static final double kI = 0.0000;
        public static final double kD = 0.000;

        public static final double AutoAimPIDTolerance = 1.0;
        // public static final double DeflectorPosInValue = 0.0;
        // public static final double DeflectorPosOutValue = 0.0;

    }

    public static final class AutoFollowConstants {
        public static final double kP = 0.301;
        public static final double kI = 0;
        public static final double kD = 0.0;

       

        public static final double AutoFollowPIDTolerance = 1.0;
        // public static final double DeflectorPosInValue = 0.0;
        // public static final double DeflectorPosOutValue = 0.0;

    }

    public static final class AutoRotateConstants {
        public static final double kP = 0.002037;
        public static final double kI = 0.0000665;
        public static final double kD = 0.0003333;

        public static final double Tolerance = 6.0;
        // public static final double DeflectorPosInValue = 0.0;
        // public static final double DeflectorPosOutValue = 0.0;

    }

    public static final class AutoTranslateConstants {
        public static final double kP = 1.17471;
        public static final double kI = 0.0665;
        public static final double kD = 0.001333;

       

        public static final double Tolerance = 0.3;
        public static final double Setpoint = 1.2;
        // public static final double DeflectorPosInValue = 0.0;
        // public static final double DeflectorPosOutValue = 0.0;

    }

    public static final class AutoStrafeConstants {
        public static final double kP = 0.05471;
        public static final double kI = 0.000665;
        public static final double kD = 0.001333;

       

        public static final double Tolerance = 1;
        

    }

    public static final class AutoConstants { 

        public static final double kHeadingOffset = 90;
                                              // tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 7.5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 5.2;
        public static final double kMaxAngularSpeedRadiansPerSecond = 3 * Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = 4* Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        
        /**
         * Config for PathPlanner to follow auto paths
         */

        /* Constraint for the motion profilied robot angle controller */
        // public static final TrapezoidProfile.Constraints kThetaControllerConstraints
        // =
        // new TrapezoidProfile.Constraints(
        // kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class LEDConstants {

        public static final int LED_1_PwmID = 6;
        public static final int LED_1_Length = 105;
        public static final int Q1 = LED_1_Length/4;
        public static final int H1 = LED_1_Length/2;
   



    }
    public static final class ElevatorConstants{

        public static final int Motor1ID = 14;
        public static final int Motor2ID = 15;
        public static final double ConversionConstant = 0.0;
        public static final InvertedValue MotorInverted = InvertedValue.CounterClockwise_Positive;
        public static final NeutralModeValue MotorMode = NeutralModeValue.Brake;
        public static final int STATOR_CURRENT_LIMIT = 60;
        public static final int CURRENT_LIMIT = 50;
        public static final boolean ENABLE_CURRENT_LIMIT = true;
        public static final boolean ENABLE_STATOR_CURRENT_LIMIT = true;
        public static final int CURRENT_THRESHOLD = 30;
        public static final double CURRENT_THRESHOLD_TIME = 0.1;
        public static final double Radius = 1;
        public static final double kS = 0.25; // Add 0.25 V output to overcome static friction
        public static final double kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        public static final double kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        public static final double kP = 3; // An error of 1 rps results in 0.11 V output
        public static final double kI = 0; // no output for integrated error
        public static final double kD = 0.0;
        public static final double kG = 1.07;
        public static final GravityTypeValue GravityType = GravityTypeValue.Elevator_Static;
        public static final double MaxVelocity = 100;
        public static final double MaxAcceleration = 200;

        public static final double Tolerance = 0.2;
        public static final boolean LimitEnable = true;
        public static final double ForwardLimit = 29.5;
        public static final double ReverseLimit = 0.5;
        public static final double DefaultPose = 1.6;
        
        public static final double L1Pose = 5.5;
        public static final double L2Pose = 11;
        public static final double L3Pose = 18.2;
        public static final double L4Pose = 27.2;
        public static final double GearRatio = 1;//6.88:1
        public static final double StallSpeed = 0.34;
        public static final double MaxSpeed = 0.3;
        public static final double MinSpeed = -0.45;
        public static final double ClimbLimit = -1;
        public static final double L0Pose = 5.4;
        public static final double A1Pose = 6.5;
        public static final double A2Pose = 13.0;
        public static final double A1Timeout = 0.3;
        public static final double A2Timeout = 0.6;

        
        public static final double L4Timeout = 2.1;
        public static final double L3Timeout = 1.7;
        public static final double L2Timeout = 1.1;
        public static final double L1Timeout = 1;
    }
    
    public static final class CoralIntakeConstants {

        public static final int MotorID = 16;
        public static final double ConversionConstant = 0.0;
        public static final InvertedValue MotorInverted = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue MotorMode = NeutralModeValue.Brake;
        public static final int STATOR_CURRENT_LIMIT = 200;
        public static final int CURRENT_LIMIT = 100;
        public static final boolean ENABLE_CURRENT_LIMIT = true;
        public static final boolean ENABLE_STATOR_CURRENT_LIMIT = true;
        public static final double OuttakeSpeed = 0.3;
        // public static final ResetMode Reset = ResetMode.kResetSafeParameters;
        // public static final PersistMode Persist = PersistMode.kNoPersistParameters;
        public static final double IntakeSpeed = 0.36;
        public static final double outtakeTime = 0.5;
        public static final double IntakeSpeedFine = 0.10;
        public static final double kP = 0.2;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        //public static final double IntakeSpeedFine = -0.05;
    }
    
    public static final class AlignConstants{

        public static final double rightRY = 2;
        public static final double rightX = 0.155;
        public static final double rightZ = -.44;

        public static final double leftRY = 12;
        public static final double leftX = 0.1;
        public static final double leftZ = -0.20;
        public static double centerRY = 1;
        public static double centerTX = 0.0;
        public static double centerTZ = -0.52;

        //interior Waypoints offset from Reef
        public static double XOffset = Units.feetToMeters(0);
        public static double YOffset = Units.feetToMeters(3);


    }


}
