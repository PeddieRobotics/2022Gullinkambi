package frc.robot.subsystems;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Enumeration;
import java.util.Hashtable;
import java.util.List;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.AutoCommands.FiveBallRight;
import frc.robot.commands.AutoCommands.FiveBallRight;
import frc.robot.commands.AutoCommands.FourBallLeft;
import frc.robot.commands.AutoCommands.FourBallRight;
import frc.robot.commands.AutoCommands.FourBallRightRude;
import frc.robot.commands.AutoCommands.SplitFFRamseteCommand;
import frc.robot.commands.AutoCommands.ThreeBallRightLL;
import frc.robot.commands.AutoCommands.ThreeBallRightLayup;
import frc.robot.commands.AutoCommands.TwoBallLeftRude;
import frc.robot.commands.AutoCommands.TwoBallLonger;
import frc.robot.commands.AutoCommands.TwoBallShorter;
import frc.robot.commands.DriveCommands.ResetOdometry;
import frc.robot.utils.Constants;

public class Autonomous extends SubsystemBase {

    private final Drivetrain m_drivetrain;
    private static Autonomous autonomous;

    private SendableChooser<Command> autoRoutineSelector;
    private Hashtable<String,Command> autoRoutines;

    private Trajectory sanityCheck, pathVerification, pathVerification_2, pathVerification_2b, pathVerification_3, pathVerification_4, pathVerification_5, pathVerification_6, straightLine_PW, lPath_manual, lPath_PW, lollipopRight_PW;
    private Trajectory twoBallLeftUpShoot, twoBallRightDownShoot, twoBallLeftRude_1, twoBallLeftRude_2, twoBallLeftRude_3;
    private Trajectory threeBallRight_LL_1, threeBallRight_LL_2, threeBallRight_Layup_1, threeBallRight_Layup_2;
    private Trajectory fourBallRight_1, fourBallRight_2, fourBallRight_3, fourBallLeft_1, fourBallLeft_2, fourBallLeft_3;
    private Trajectory fiveBallRight_1, fiveBallRight_2, fiveBallRight_3, fiveBallRight_4;

    public Autonomous() {
        autoRoutines = new Hashtable<String,Command>();
        autoRoutineSelector = new SendableChooser<Command>();

        m_drivetrain = Drivetrain.getInstance();

        defineAutoPaths();
        setupAutoRoutines();
        setupAutoSelector();
    }

    public static Autonomous getInstance() {
        if (autonomous == null) {
            autonomous = new Autonomous();
        }
        return autonomous;
    }

    public void setupAutoSelector() {
        Enumeration<String> e = autoRoutines.keys();

        while(e.hasMoreElements()){
            String autoRoutineName = e.nextElement();
            autoRoutineSelector.addOption(autoRoutineName, autoRoutines.get(autoRoutineName));
        };

        SmartDashboard.putData("Auto Routines", autoRoutineSelector);
    }

    public void setupAutoRoutines() {
        // autoRoutines.put("Sanity Check", new SequentialCommandGroup(new ResetOdometry(sanityCheck.getInitialPose()),createCommandFromTrajectory(sanityCheck)));
        // autoRoutines.put("Path Verification", new SequentialCommandGroup(new ResetOdometry(pathVerification.getInitialPose()),createCommandFromTrajectory(pathVerification)));
        // autoRoutines.put("Path Verification 2", new SequentialCommandGroup(new ResetOdometry(pathVerification_2.getInitialPose()), createCommandFromTrajectory(pathVerification_2)));
        // autoRoutines.put("Path Verification 2b", new SequentialCommandGroup(new ResetOdometry(pathVerification_2b.getInitialPose()), createCommandFromTrajectory(pathVerification_2b)));
        // autoRoutines.put("Path Verification 2+2b", new SequentialCommandGroup(new ResetOdometry(pathVerification_2.getInitialPose()), createCommandFromTrajectory(pathVerification_2) , createCommandFromTrajectory(pathVerification_2b)));
        // autoRoutines.put("Path Verification 3", new SequentialCommandGroup(new ResetOdometry(pathVerification_3.getInitialPose()), createCommandFromTrajectory(pathVerification_3)));
        // autoRoutines.put("Path Verification 4", new SequentialCommandGroup(new ResetOdometry(pathVerification_4.getInitialPose()), createCommandFromTrajectory(pathVerification_4)));
        // autoRoutines.put("Path Verification 5", new SequentialCommandGroup(new ResetOdometry(pathVerification_5.getInitialPose()), createCommandFromTrajectory(pathVerification_5)));
        // autoRoutines.put("Path Verification 6", new SequentialCommandGroup(new ResetOdometry(pathVerification_6.getInitialPose()), createCommandFromTrajectory(pathVerification_6)));

        //autoRoutines.put("Straight Line (PW)", new SequentialCommandGroup(new ResetOdometry(straightLine_PW.getInitialPose()), createCommandFromTrajectory(straightLine_PW)));
        //autoRoutines.put("L Path (PW)", new SequentialCommandGroup(new ResetOdometry(lPath_PW.getInitialPose()), createCommandFromTrajectory(lPath_PW)));
        //autoRoutines.put("Lollipop Right (PW)", new SequentialCommandGroup(new ResetOdometry(lollipopRight_PW.getInitialPose()), createCommandFromTrajectory(lollipopRight_PW)));

        autoRoutines.put("CMD Group: 2 Ball (longer)", new TwoBallLonger(twoBallLeftUpShoot.getInitialPose(), createCommandFromTrajectory(twoBallLeftUpShoot)));
        autoRoutines.put("CMD Group: 2 Ball (shorter)", new TwoBallShorter(twoBallRightDownShoot.getInitialPose(), createCommandFromTrajectory(twoBallRightDownShoot)));
        autoRoutines.put("CMD Group: 2 Ball Left Rude", new TwoBallLeftRude(twoBallLeftRude_1.getInitialPose(), createCommandFromTrajectory(twoBallLeftRude_1), createCommandFromTrajectory(twoBallLeftRude_2), createCommandFromTrajectory(twoBallLeftRude_3)));

        //autoRoutines.put("CMD Group: 3 Ball Right Layup", new ThreeBallRightLayup(threeBallRight_Layup_1.getInitialPose(), createCommandFromTrajectory(threeBallRight_Layup_1), createCommandFromTrajectory(threeBallRight_Layup_2)));
        //autoRoutines.put("CMD Group: 3 Ball Right LL", new ThreeBallRightLL(threeBallRight_LL_1.getInitialPose(),  createCommandFromTrajectory(threeBallRight_LL_1), createCommandFromTrajectory(threeBallRight_LL_2)));

        autoRoutines.put("CMD Group: 4 Ball (Right)", new FourBallRight(fourBallRight_1.getInitialPose(), createCommandFromTrajectory(fourBallRight_1), createCommandFromTrajectory(fourBallRight_2)));
        autoRoutines.put("CMD Group: 4 Ball (Left)", new FourBallLeft(fourBallLeft_1.getInitialPose(), createCommandFromTrajectory(fourBallLeft_1), createCommandFromTrajectory(fourBallLeft_2), createCommandFromTrajectory(fourBallLeft_3)));

        autoRoutines.put("CMD Group: 4 Ball Rude (Right)", new FourBallRightRude(fourBallRight_1.getInitialPose(), createCommandFromTrajectory(fourBallRight_1), createCommandFromTrajectory(fourBallRight_2), createCommandFromTrajectory(fourBallRight_3)));

        autoRoutines.put("CMD Group: 5 Ball (Right)", new FiveBallRight(fiveBallRight_1.getInitialPose(), createCommandFromTrajectory(fiveBallRight_1), createCommandFromTrajectory(fiveBallRight_2), createCommandFromTrajectory(fiveBallRight_3), createCommandFromTrajectory(fiveBallRight_4)));

    }

    public Command returnAutonomousCommand() {
        return autoRoutineSelector.getSelected();
    }

    private void defineAutoPaths(){
        // sanityCheck = PathPlanner.loadPath("SanityCheck", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared);

        // pathVerification = PathPlanner.loadPath("PathVerification", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared);
        // pathVerification_2 = PathPlanner.loadPath("PathVerification2", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared);
        // pathVerification_2b = PathPlanner.loadPath("PathVerification2b", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared);
        // pathVerification_3 = PathPlanner.loadPath("PathVerification3", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared);
        // pathVerification_4 = PathPlanner.loadPath("PathVerification4", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared);
        // pathVerification_5 = PathPlanner.loadPath("PathVerification5", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared);
        // pathVerification_6 = PathPlanner.loadPath("PathVerification6", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared);

        //straightLine_PW = loadPathWeaverJSON("output/StraightLine1.wpilib.json");
        //lPath_PW = loadPathWeaverJSON("output/LPath1.wpilib.json");
        //lollipopRight_PW = loadPathWeaverJSON("output/LollipopRight1.wpilib.json");

        twoBallLeftUpShoot = PathPlanner.loadPath("2BallLeftUpShoot", Constants.kMaxSpeedMetersPerSecond*0.5, Constants.kMaxAccelerationMetersPerSecondSquared*0.5);
        twoBallRightDownShoot = PathPlanner.loadPath("2BallRightDownShoot", Constants.kMaxSpeedMetersPerSecond*0.5, Constants.kMaxAccelerationMetersPerSecondSquared*0.5);

        twoBallLeftRude_1 = PathPlanner.loadPath("2BallLeftRude_Part1", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared);
        twoBallLeftRude_2 = PathPlanner.loadPath("2BallLeftRude_Part2", Constants.kMaxSpeedMetersPerSecond*0.8, Constants.kMaxAccelerationMetersPerSecondSquared*0.8);
        twoBallLeftRude_3 = PathPlanner.loadPath("2BallLeftRude_Part3", Constants.kMaxSpeedMetersPerSecond*0.8, Constants.kMaxAccelerationMetersPerSecondSquared*0.8);

        //threeBallRight_Layup_1 = PathPlanner.loadPath("3BallRight_Layup_Part1", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared);
        //threeBallRight_Layup_2 = PathPlanner.loadPath("3BallRight_Layup_Part2", Constants.kMaxSpeedMetersPerSecond*1.2, Constants.kMaxAccelerationMetersPerSecondSquared*1.2);
        
        //threeBallRight_LL_1 = PathPlanner.loadPath("3BallRight_LL_Part1", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared);
        //threeBallRight_LL_2 = PathPlanner.loadPath("3BallRight_LL_Part2", Constants.kMaxSpeedMetersPerSecond*1.2, Constants.kMaxAccelerationMetersPerSecondSquared*1.2);

        fourBallRight_1 = PathPlanner.loadPath("4BallRight_Part1", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared);
        fourBallRight_2 = PathPlanner.loadPath("4BallRight_Part2", Constants.kMaxSpeedMetersPerSecond*1.2, Constants.kMaxAccelerationMetersPerSecondSquared*1.2);
        
        fourBallRight_3 = PathPlanner.loadPath("4BallRight_Part3", Constants.kMaxSpeedMetersPerSecond*1.2, Constants.kMaxAccelerationMetersPerSecondSquared*1.2);

        fourBallLeft_1 = PathPlanner.loadPath("4BallLeft_Part1", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared);
        fourBallLeft_2 = PathPlanner.loadPath("4BallLeft_Part2", Constants.kMaxSpeedMetersPerSecond*1.3, Constants.kMaxAccelerationMetersPerSecondSquared*1.3);
        fourBallLeft_3 = PathPlanner.loadPath("4BallLeft_Part3", Constants.kMaxSpeedMetersPerSecond*1.3, Constants.kMaxAccelerationMetersPerSecondSquared*1.3, true);

        fiveBallRight_1 = PathPlanner.loadPath("5BallRight_Part1", Constants.kMaxSpeedMetersPerSecond*1.3, Constants.kMaxAccelerationMetersPerSecondSquared*1.3);
        fiveBallRight_2 = PathPlanner.loadPath("5BallRight_Part2", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared);
        fiveBallRight_3 = PathPlanner.loadPath("5BallRight_Part3", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared);
        fiveBallRight_4 = PathPlanner.loadPath("5BallRight_Part4", Constants.kMaxSpeedMetersPerSecond*1.5, Constants.kMaxAccelerationMetersPerSecondSquared*1.5, true);

    }
 
    private Trajectory loadPathWeaverJSON(String trajectoryJSON) {
        Trajectory trajectory = null;
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
            }
        return trajectory;
    }

    public RamseteCommand createCommandFromTrajectory(Trajectory trajectory){
        var ramseteController = new RamseteController();
        //ramseteController.setEnabled(false);
        // var table = NetworkTableInstance.getDefault().getTable("troubleshooting");
        // var leftReference = table.getEntry("left_reference");
        // var leftMeasurement = table.getEntry("left_measurement");

        // var rightReference = table.getEntry("right_reference");
        // var rightMeasurement = table.getEntry("right_measurement");
        
        RamseteCommand autoCommand = 
            new RamseteCommand(trajectory,
                                m_drivetrain::getPose,
                                ramseteController,
                                Constants.kDriveKinematics,
                                (leftWheelSpeed, rightWheelSpeed) -> {
                                m_drivetrain.updateDrivePIDControllers(leftWheelSpeed.doubleValue(), rightWheelSpeed.doubleValue()); 
                                // leftMeasurement.setNumber(m_drivetrain.getWheelSpeeds().leftMetersPerSecond);
                                // leftReference.setNumber(leftWheelSpeed.doubleValue());
              
                                // rightMeasurement.setNumber(m_drivetrain.getWheelSpeeds().rightMetersPerSecond);
                                // rightReference.setNumber(rightWheelSpeed.doubleValue());       
                                },
                                m_drivetrain);

        return autoCommand;
      }
    
    public Trajectory getTransformedTrajectory(Trajectory t){
        Pose2d newOrigin = t.getInitialPose();
        Trajectory transformed = t.relativeTo(newOrigin);
        return transformed;
    }

}
