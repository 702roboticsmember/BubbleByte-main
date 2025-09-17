package frc.robot;



import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.AutoFollowCommand;
import frc.robot.commands.ClimbPID;
import frc.robot.commands.ElevatorPID;
import frc.robot.commands.FollowPath;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.BackLimelightSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightSubsystemRight;
import frc.robot.subsystems.Swerve;
/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final XboxController driver = new XboxController(0);
    private final XboxController codriver = new XboxController(1);

    

    

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton leftstation = new JoystickButton(driver, XboxController.Button.kBack.value);
    private final JoystickButton heightReset = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton fastMode = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton slowMode = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton align = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton alignLeft = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton alignRight = new JoystickButton(driver, XboxController.Button.kRightBumper.value);


    

    /* CoDriver Buttons */
    private final JoystickButton algae1ReefIntake = new JoystickButton(codriver, XboxController.Button.kStart.value);
    private final JoystickButton algae2ReefIntake = new JoystickButton(codriver, XboxController.Button.kBack.value);
    private final JoystickButton climbIn = new JoystickButton(codriver, XboxController.Button.kB.value);
    private final JoystickButton climbOut = new JoystickButton(codriver, XboxController.Button.kA.value);
    private final JoystickButton coralOuttake = new JoystickButton(codriver, XboxController.Button.kX.value);
    //private final JoystickButton coralIntake = new JoystickButton(codriver, XboxController.Button.kStart.value);
    private final JoystickButton nest = new JoystickButton(codriver, XboxController.Button.kY.value);
    //private final JoystickButton start = new JoystickButton(codriver, XboxController.Button.kA.value);
    private final JoystickButton algaeGroundIntake = new JoystickButton(codriver, XboxController.Button.kRightBumper.value);
    private final JoystickButton algaeOuttake = new JoystickButton(codriver, XboxController.Button.kLeftBumper.value);
    //private final JoystickButton back = new JoystickButton(codriver, XboxController.Button.kB.value);
    private final POVButton alignLeft2 = new POVButton(driver, 270);
    private final POVButton alignRight2 = new POVButton(driver, 90);


    private final POVButton L1 = new POVButton(codriver, 90);
    private final POVButton L2 = new POVButton(codriver, 0);
    private final POVButton L3 = new POVButton(codriver, 270);
    private final POVButton L4 = new POVButton(codriver, 180);

    //public static double power = 0.60;
    public static boolean overrideHeight = false;
    public static boolean robotCentric = false;
    public static boolean climbing = false;

    private final SendableChooser<Command> autoChooser;
    private final SendableChooser<Command> teamChooser;
  

    //private static final Orchestra orchestra = new Orchestra("mario.chrp");

    /* Subsystems */
    private final LimelightSubsystemRight l_LimelightSubsystem = new LimelightSubsystemRight();
    private final BackLimelightSubsystem l_LimelightBackSubsystem = new BackLimelightSubsystem();
    private final Swerve s_Swerve = new Swerve(l_LimelightBackSubsystem, l_LimelightSubsystem);
    private final LEDSubsystem l_LEDSubsystem = new LEDSubsystem();
    private final CoralIntakeSubsystem c_CoralIntakeSubsystem = new CoralIntakeSubsystem();
    private final ElevatorSubsystem e_ElevatorSubsytem = new ElevatorSubsystem(new ClimbSubsystem(null)); 
    private final ClimbSubsystem c_ClimbSubsystem = new ClimbSubsystem(e_ElevatorSubsytem);
    public static Field2d field = new Field2d();

    /**
     * The container for the robot. Contains subsystems, IO devices, and commands.
     */

    /* Commands */
    

     public Command Align_Driver(double x, double z, double ry) {
        return new AlignCommand(l_LimelightSubsystem,
                        x, 
                        z, 
                        ry, 
                        s_Swerve); 
    }

    public Command AutoFollow_Driver(double turn) {
        return new AutoFollowCommand(()->l_LimelightSubsystem.getTargetX(), 
            ()->l_LimelightSubsystem.getTargetA(), 
            ()->l_LimelightSubsystem.IsTargetAvailable(), 
            s_Swerve, 
            turn);        
    }

    // public Command Nest() {
    //     return new ParallelCommandGroup(
    //         new AlgaeArmPID(a_AlgaeArmSubsystem, Constants.AlgaeArmConstants.DefaultPose),
    //         Commands.run(()-> e_ElevatorSubsytem.set(Constants.ElevatorConstants.DefaultPose, true), e_ElevatorSubsytem),
    //         new ClimbPID(c_ClimbSubsystem, Constants.ClimberConstants.DefaultPose)
    //     );
    // }
    public Command AlignRight_Driver(){
        return new AlignCommand(l_LimelightSubsystem, 
                        Constants.AlignConstants.rightX,
                        Constants.AlignConstants.rightZ, 
                        Constants.AlignConstants.rightRY, 
                        s_Swerve); 
                    }
    public Command AlignLeft_Driver(){
        return new AlignCommand(l_LimelightSubsystem, 
                        Constants.AlignConstants.leftX,
                        Constants.AlignConstants.leftZ, 
                        Constants.AlignConstants.leftRY, 
                        s_Swerve); 
    }

    public Command AlignLeft_Driver2(){
        return new AlignCommand(l_LimelightSubsystem, 
                        Constants.AlignConstants.leftX - 0.01,
                        Constants.AlignConstants.leftZ, 
                        Constants.AlignConstants.leftRY, 
                        s_Swerve); 
    }

    public Command AlignRight_Driver2(){
        return new AlignCommand(l_LimelightSubsystem, 
                        Constants.AlignConstants.rightX + 0.01,
                        Constants.AlignConstants.rightZ, 
                        Constants.AlignConstants.rightRY, 
                        s_Swerve); 
                    }

    public Command AlignRightOffset_Driver(double xoff, double zoff, double ryoff){
        return new AlignCommand(l_LimelightSubsystem, 
                        Constants.AlignConstants.rightX + xoff,
                        Constants.AlignConstants.rightZ + zoff, 
                        Constants.AlignConstants.rightRY + ryoff, 
                        s_Swerve); 
                    }
    public Command AlignLeftOffset_Driver(double xoff, double zoff, double ryoff){
        return new AlignCommand(l_LimelightSubsystem, 
                        Constants.AlignConstants.leftX + xoff,
                        Constants.AlignConstants.leftZ + zoff, 
                        Constants.AlignConstants.leftRY + ryoff, 
                        s_Swerve); 
    }

    public Command AlignCenter_Driver(){
        return new AlignCommand(l_LimelightSubsystem, 
                        Constants.AlignConstants.centerTX,
                        Constants.AlignConstants.centerTZ, 
                        Constants.AlignConstants.centerRY, 
                        s_Swerve); 
    }


    public Command ClimbOutPID(){
        return new ParallelCommandGroup(new ClimbPID(c_ClimbSubsystem, Constants.ClimberConstants.OutPose));
    }

    public Command ClimbInPID(){
        return Commands.either(new ClimbPID(c_ClimbSubsystem, Constants.ClimberConstants.InPose), new ClimbPID(c_ClimbSubsystem, Constants.ClimberConstants.InPose), ()-> (e_ElevatorSubsytem.getElevatorHeight() < 5));
        
    }
    public Command CoralOuttake_coDriver(){
        return c_CoralIntakeSubsystem.run(()->Constants.CoralIntakeConstants.OuttakeSpeed);
    }

    public Command CoralOuttake_coDriver(DoubleSupplier speed){
        return c_CoralIntakeSubsystem.run(speed);
    }

    public Command CoralOn(){
        return Commands.run(()-> c_CoralIntakeSubsystem.setSpeed(Constants.CoralIntakeConstants.OuttakeSpeed), c_CoralIntakeSubsystem);
    }

    public Command CoralOuttake(){
        return new SequentialCommandGroup(CoralOn().withTimeout(Constants.CoralIntakeConstants.outtakeTime), CoralOff());
    }

    public Command CoralIntake(){
        return Commands.either(CoralOuttake(), CoralOff(), ()-> !c_CoralIntakeSubsystem.getSensor());
    }

    public Command CoralIntake_coDriver(DoubleSupplier speed){
        return Commands.either(CoralOuttake_coDriver(speed), CoralOff(), ()-> !c_CoralIntakeSubsystem.getSensor());
    }

    public Command CoralOff(){
        return Commands.run(()-> c_CoralIntakeSubsystem.setSpeed(0), c_CoralIntakeSubsystem);
    }

    public Command LimitSwitchDeadline(){
        return Commands.waitUntil(()-> !c_CoralIntakeSubsystem.getSensor());
    }

    public Command L4Deadline(){
        return Commands.race(Commands.waitUntil(()-> e_ElevatorSubsytem.getElevatorHeight() > Constants.ElevatorConstants.L4Pose - 1),  new WaitCommand(Constants.ElevatorConstants.L4Timeout));
    }

    public Command L3Deadline(){
        return Commands.race(
            Commands.waitUntil(()-> e_ElevatorSubsytem.getElevatorHeight() > Constants.ElevatorConstants.L3Pose - Constants.ElevatorConstants.Tolerance && 
            e_ElevatorSubsytem.getElevatorHeight() < Constants.ElevatorConstants.L3Pose + Constants.ElevatorConstants.Tolerance),
            new WaitCommand(Constants.ElevatorConstants.L3Timeout));
    }
    public Command L2Deadline(){
        return Commands.race(
            Commands.waitUntil(()-> e_ElevatorSubsytem.getElevatorHeight() > Constants.ElevatorConstants.L2Pose - Constants.ElevatorConstants.Tolerance && 
            e_ElevatorSubsytem.getElevatorHeight() < Constants.ElevatorConstants.L2Pose + Constants.ElevatorConstants.Tolerance),
            new WaitCommand(Constants.ElevatorConstants.L2Timeout));
    }
    public Command L1Deadline(){
        return Commands.race(
            Commands.waitUntil(()-> e_ElevatorSubsytem.getElevatorHeight() > Constants.ElevatorConstants.L1Pose - Constants.ElevatorConstants.Tolerance && 
            e_ElevatorSubsytem.getElevatorHeight() < Constants.ElevatorConstants.L1Pose + Constants.ElevatorConstants.Tolerance),
            new WaitCommand(Constants.ElevatorConstants.L1Timeout));
    }

    public Command A1Deadline(){
        return new SequentialCommandGroup(Commands.race(
            Commands.waitUntil(()-> e_ElevatorSubsytem.getElevatorHeight() > Constants.ElevatorConstants.A1Pose - Constants.ElevatorConstants.Tolerance && 
            e_ElevatorSubsytem.getElevatorHeight() < Constants.ElevatorConstants.A1Pose + Constants.ElevatorConstants.Tolerance),
            new WaitCommand(Constants.ElevatorConstants.A1Timeout)), new WaitCommand(1));
    }
    public Command A2Deadline(){
        return Commands.race(
            Commands.waitUntil(()-> e_ElevatorSubsytem.getElevatorHeight() > Constants.ElevatorConstants.A2Pose - Constants.ElevatorConstants.Tolerance && 
            e_ElevatorSubsytem.getElevatorHeight() < Constants.ElevatorConstants.A2Pose + Constants.ElevatorConstants.Tolerance),
            new WaitCommand(Constants.ElevatorConstants.A2Timeout));
    }

    public Command L1(){
        return Commands.run(()-> e_ElevatorSubsytem.set(Constants.ElevatorConstants.L1Pose, true), e_ElevatorSubsytem);
    }

    public Command L2(){
        return Commands.run(()-> e_ElevatorSubsytem.set(Constants.ElevatorConstants.L2Pose, true), e_ElevatorSubsytem);
    }

    public Command L3(){
        return Commands.run(()-> e_ElevatorSubsytem.set(Constants.ElevatorConstants.L3Pose, true), e_ElevatorSubsytem);
    }

    public Command L4(){
        return Commands.run(()-> e_ElevatorSubsytem.set(Constants.ElevatorConstants.L4Pose, true), e_ElevatorSubsytem);
    }
    public Command L0(){
        return Commands.run(()-> e_ElevatorSubsytem.set(Constants.ElevatorConstants.L0Pose, true), e_ElevatorSubsytem);
    }
    public Command L1_5(){
        return Commands.run(()-> e_ElevatorSubsytem.set(Constants.ElevatorConstants.L3Pose, true), e_ElevatorSubsytem);
    }

    
// Create the constraints to use while pathfinding. The constraints defined in the path will only be used for the path.


// Since AutoBuilder is configured, we can use it to build pathfinding commands


    public RobotContainer() {
        //Nest();
        
        //Pathfinding.setDynamicObstacles(null, null);
        //beamLED.set(true);
        
        SmartDashboard.putData(new InstantCommand(() -> {
            s_Swerve.gyro.reset();
        }));

        SmartDashboard.putData("Field", field);
        SmartDashboard.putNumber("robotposex", s_Swerve.getPose().getTranslation().getX());
        SmartDashboard.putNumber("robotposey", s_Swerve.getPose().getTranslation().getY());
      
        
        //Logging callback for current robot pose
        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            field.setRobotPose(pose);
        });

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            field.getObject("target pose").setPose(pose);
        });

        // Logging callback for the active path, this is sent as a list of poses
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            field.getObject("path").setPoses(poses);
        });

        NamedCommands.registerCommand("align", Align_Driver(0, .75, 0));
        NamedCommands.registerCommand("Disconect", new FollowPath(s_Swerve, "P3Disconect"));

        NamedCommands.registerCommand("AlignRight", AlignRight_Driver());
        NamedCommands.registerCommand("AlignLeft", AlignLeft_Driver());
        NamedCommands.registerCommand("AlignCenter", AlignCenter_Driver());

        NamedCommands.registerCommand("L0", L0());
        NamedCommands.registerCommand("L1", L1());
        NamedCommands.registerCommand("L2", L2());
        NamedCommands.registerCommand("L3", L3());
        NamedCommands.registerCommand("L4", L4());
        
        NamedCommands.registerCommand("LimitSwitchDeadline", LimitSwitchDeadline());
        NamedCommands.registerCommand("L1Deadline", L1Deadline());
        NamedCommands.registerCommand("L2Deadline", L2Deadline());
        NamedCommands.registerCommand("L3Deadline", L3Deadline());
        NamedCommands.registerCommand("L4Deadline", L4Deadline());
        NamedCommands.registerCommand("A1Deadline", A1Deadline());
        NamedCommands.registerCommand("A2Deadline", A2Deadline());
        NamedCommands.registerCommand("CoralOuttake", CoralOuttake());
        NamedCommands.registerCommand("CoralOff", CoralOff());
        NamedCommands.registerCommand("CoralIntake", CoralIntake());
        //NamedCommands.registerCommand("Nest", Nest());
        NamedCommands.registerCommand("Cpose", new InstantCommand(()-> s_Swerve.setPoseToReef("c")));
        NamedCommands.registerCommand("Dpose", new InstantCommand(()-> s_Swerve.setPoseToReef("d")));
        NamedCommands.registerCommand("Epose", new InstantCommand(()-> s_Swerve.setPoseToReef("e")));
        NamedCommands.registerCommand("Jpose", new InstantCommand(()-> s_Swerve.setPoseToReef("j")));
        NamedCommands.registerCommand("Kpose", new InstantCommand(()-> s_Swerve.setPoseToReef("k")));
        NamedCommands.registerCommand("Lpose", new InstantCommand(()-> s_Swerve.setPoseToReef("l")));
        l_LEDSubsystem.DoTheRainbow(false);
        s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, 
        ()-> -driver.getRawAxis(1), 
        ()-> -driver.getRawAxis(0),
        ()-> -driver.getRawAxis(4), 
        ()->robotCentric, 
        ()->e_ElevatorSubsytem.getElevatorHeight(), 
        ()-> overrideHeight));

        
        c_ClimbSubsystem.setDefaultCommand(c_ClimbSubsystem.run(()-> -codriver.getRawAxis(0) * Constants.ClimberConstants.MaxLiftSpeed));
        //c_CoralIntakeSubsystem.setDefaultCommand(a_AlgaeIntakeSubsystem.run(()-> codriver.getRawAxis(2)));
        e_ElevatorSubsytem.setDefaultCommand(e_ElevatorSubsytem.run((()-> -codriver.getRawAxis(5) + Constants.ElevatorConstants.StallSpeed )));
        l_LEDSubsystem.setDefaultCommand(new InstantCommand(()->l_LEDSubsystem.DoTheRainbow(false), l_LEDSubsystem));

        
        l_LEDSubsystem.setDefaultCommand(new InstantCommand(() -> {
        l_LEDSubsystem.setColor(Color.kYellow);
        }, l_LEDSubsystem
        ));
        configureButtonBindings();
        
        

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();
        teamChooser = new SendableChooser<>();
        

        SmartDashboard.putData("Auto Chooser", autoChooser);
        SmartDashboard.putData("Team Chooser", teamChooser);
        //SmartDashboard.putNumber("gyroYaaaww", s_Swerve.getGyroYaw());
    }

    

    

    // private Command wrapSpeedChange(Runnable r) {
    //     return Commands.runOnce(() -> {
    //         r.run();
    //         RobotContainer.this.debugSpeeds();
    //     });
    // }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

        /* Driver Buttons */
        zeroGyro.onTrue(new SequentialCommandGroup(new InstantCommand(()->s_Swerve.gyro.reset()), new InstantCommand(() -> s_Swerve.zeroHeading())));
        slowMode.onTrue(new InstantCommand(() -> RobotContainer.overrideHeight = false));
        fastMode.onTrue(new InstantCommand(() -> RobotContainer.overrideHeight = true));
        // back.onTrue(e_ElevatorSubsytem.run(() -> e_ElevatorSubsytem.set1(0.2)));
        // back.onFalse(e_ElevatorSubsytem.run(() -> e_ElevatorSubsytem.set1(0)));
        // start.whileTrue(e_ElevatorSubsytem.run(() -> e_ElevatorSubsytem.set2(0.2)));
        // start.whileFalse(e_ElevatorSubsytem.run(() -> e_ElevatorSubsytem.set2(0)));

        align.whileTrue(new ParallelCommandGroup(
                 new InstantCommand(()-> l_LimelightSubsystem.setCamMode(0)),new SequentialCommandGroup( AlignCenter_Driver())));

        align.onFalse(new ParallelCommandGroup(new InstantCommand(()-> l_LimelightSubsystem.setCamMode(0))));
        // follow.whileTrue(new SequentialCommandGroup(
        //     new InstantCommand(()-> l_LimelightSubsystem.setCamMode(0)), AutoFollow_Driver(0.3)));

        // follow.onFalse(new ParallelCommandGroup(new InstantCommand(()-> l_LimelightSubsystem.setCamMode(0))));
        alignLeft.whileTrue(new ParallelCommandGroup(new InstantCommand(()-> l_LimelightSubsystem.setCamMode(0)), new SequentialCommandGroup( AlignLeft_Driver())));
        alignRight.whileTrue(new ParallelCommandGroup(new InstantCommand(()-> l_LimelightSubsystem.setCamMode(0)), new SequentialCommandGroup( AlignRight_Driver())));
        alignLeft2.whileTrue(new ParallelCommandGroup(new InstantCommand(()-> l_LimelightSubsystem.setCamMode(0)), new SequentialCommandGroup( AlignLeft_Driver2())));
        alignRight2.whileTrue(new ParallelCommandGroup(new InstantCommand(()-> l_LimelightSubsystem.setCamMode(0)), new SequentialCommandGroup( AlignRight_Driver2())));
//resetpose.onTrue(new InstantCommand(()->field.setRobotPose(0, 0, s_Swerve.getHeading())));
        // leftstation.whileTrue(new SequentialCommandGroup(new FollowPath(s_Swerve, "PathTest")));
        // rightstation.whileTrue(new SequentialCommandGroup(new FollowPath(s_Swerve, "PathTest")));
        heightReset.onTrue(new InstantCommand(()-> e_ElevatorSubsytem.resetHeight()));

        /*CoDriver Buttons*/
        
        //nest.whileTrue(Nest());
        // climbOut.whileTrue(ClimbOutPID());
        // climbIn.whileTrue(Commands.either(new ClimbPID(c_ClimbSubsystem, Constants.ClimberConstants.InPose), new ClimbPID(c_ClimbSubsystem, Constants.ClimberConstants.InPose), ()-> (e_ElevatorSubsytem.getElevatorHeight() < 5)));
        coralOuttake.whileTrue(CoralOuttake_coDriver());
        //coralIntake.whileTrue(CoralIntake_coDriver(Constants.CoralIntakeConstants.IntakeSpeed));
        
        L1.whileTrue(L1());
        L2.whileTrue(L2());
        L3.whileTrue(L3());
        L4.whileTrue(L4());

    }
        

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(autoChooser.getSelected());
        
    }

}
