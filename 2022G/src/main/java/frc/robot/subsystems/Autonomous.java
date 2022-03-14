package frc.robot.subsystems;

import java.util.Enumeration;
import java.util.Hashtable;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.AutoCommands.FiveBallPathRight;
import frc.robot.commands.AutoCommands.FiveBallPathRightv2;
import frc.robot.commands.AutoCommands.FourBallPathLeft;
import frc.robot.commands.AutoCommands.FourBallPathRight;
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

    private Trajectory sanityCheck, pathVerification;
    private Trajectory twoBallLeftUpShoot, twoBallRightDownShoot, twoBallLeftRude_1, twoBallLeftRude_2;
    private Trajectory threeBallRight_LL_1, threeBallRight_LL_2, threeBallRight_Layup_1, threeBallRight_Layup_2;
    private Trajectory fourBallRight_1, fourBallRight_2, fourBallLeft_1, fourBallLeft_2, fourBallLeft_3;
    private Trajectory fiveBallRight_1, fiveBallRight_2, fiveBallRight_3, fiveBallRight_4, fiveBallRight_v2_1, fiveBallRight_v2_2, fiveBallRight_v2_3, fiveBallRight_v2_4;

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
        autoRoutines.put("Sanity Check", new SequentialCommandGroup(new ResetOdometry(sanityCheck.getInitialPose()),createCommandFromTrajectory(sanityCheck)));
        autoRoutines.put("Path Verification", new SequentialCommandGroup(new ResetOdometry(pathVerification.getInitialPose()),createCommandFromTrajectory(pathVerification)));

        autoRoutines.put("CMD Group: 2 Ball (longer)", new TwoBallLonger(twoBallLeftUpShoot.getInitialPose(), createCommandFromTrajectory(twoBallLeftUpShoot)));
        autoRoutines.put("CMD Group: 2 Ball (shorter)", new TwoBallShorter(twoBallRightDownShoot.getInitialPose(), createCommandFromTrajectory(twoBallRightDownShoot)));
        autoRoutines.put("CMD Group: 2 Ball Left Rude", new TwoBallLeftRude(twoBallRightDownShoot.getInitialPose(), createCommandFromTrajectory(twoBallLeftRude_1), createCommandFromTrajectory(twoBallLeftRude_2)));

        autoRoutines.put("CMD Group: 3 Ball Right Layup", new ThreeBallRightLayup(threeBallRight_Layup_1.getInitialPose(), createCommandFromTrajectory(threeBallRight_Layup_1), createCommandFromTrajectory(threeBallRight_Layup_2)));
        autoRoutines.put("CMD Group: 3 Ball Right LL", new ThreeBallRightLL(threeBallRight_LL_1.getInitialPose(),  createCommandFromTrajectory(threeBallRight_LL_1), createCommandFromTrajectory(threeBallRight_LL_2)));

        autoRoutines.put("CMD Group: 4 Ball Path (Right)", new FourBallPathRight(fourBallRight_1.getInitialPose(), createCommandFromTrajectory(fourBallRight_1), createCommandFromTrajectory(fourBallRight_2)));
        autoRoutines.put("CMD Group: 4 Ball Path (Left)", new FourBallPathLeft(fourBallLeft_1.getInitialPose(), createCommandFromTrajectory(fourBallLeft_1), createCommandFromTrajectory(fourBallLeft_2), createCommandFromTrajectory(fourBallLeft_3)));

        autoRoutines.put("CMD Group: 5 Ball Path (Right)", new FiveBallPathRight(fiveBallRight_1.getInitialPose(), createCommandFromTrajectory(fiveBallRight_1), createCommandFromTrajectory(fiveBallRight_2), createCommandFromTrajectory(fiveBallRight_3), createCommandFromTrajectory(fiveBallRight_4)));
        autoRoutines.put("CMD Group: 5 Ball Path v2 (Right)", new FiveBallPathRightv2(fiveBallRight_v2_1.getInitialPose(), createCommandFromTrajectory(fiveBallRight_v2_1), createCommandFromTrajectory(fiveBallRight_v2_2), createCommandFromTrajectory(fiveBallRight_v2_3), createCommandFromTrajectory(fiveBallRight_v2_4)));


    }

    public Command returnAutonomousCommand() {
        return autoRoutineSelector.getSelected();
    }

    private void defineAutoPaths(){
        sanityCheck = PathPlanner.loadPath("SanityCheck", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared);

        pathVerification = PathPlanner.loadPath("PathVerification", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared);

        twoBallLeftUpShoot = PathPlanner.loadPath("2BallLeftUpShoot", Constants.kMaxSpeedMetersPerSecond*0.5, Constants.kMaxAccelerationMetersPerSecondSquared*0.5);
        twoBallRightDownShoot = PathPlanner.loadPath("2BallRightDownShoot", Constants.kMaxSpeedMetersPerSecond*0.5, Constants.kMaxAccelerationMetersPerSecondSquared*0.5);

        twoBallLeftRude_1 = PathPlanner.loadPath("2BallLeftRude_Part1", Constants.kMaxSpeedMetersPerSecond*0.8, Constants.kMaxAccelerationMetersPerSecondSquared*0.8);
        twoBallLeftRude_2 = PathPlanner.loadPath("2BallLeftRude_Part2", Constants.kMaxSpeedMetersPerSecond*0.8, Constants.kMaxAccelerationMetersPerSecondSquared*0.8);

        threeBallRight_Layup_1 = PathPlanner.loadPath("3BallRight_Layup_Part1", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared);
        threeBallRight_Layup_2 = PathPlanner.loadPath("3BallRight_Layup_Part2", Constants.kMaxSpeedMetersPerSecond*1.2, Constants.kMaxAccelerationMetersPerSecondSquared*1.2);
        
        threeBallRight_LL_1 = PathPlanner.loadPath("3BallRight_LL_Part1", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared);
        threeBallRight_LL_2 = PathPlanner.loadPath("3BallRight_LL_Part2", Constants.kMaxSpeedMetersPerSecond*1.2, Constants.kMaxAccelerationMetersPerSecondSquared*1.2);

        fourBallRight_1 = PathPlanner.loadPath("4BallRight_Part1", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared);
        fourBallRight_2 = PathPlanner.loadPath("4BallRight_Part2", Constants.kMaxSpeedMetersPerSecond*1.2, Constants.kMaxAccelerationMetersPerSecondSquared*1.2);
        
        fourBallLeft_1 = PathPlanner.loadPath("4BallLeft_Part1", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared);
        fourBallLeft_2 = PathPlanner.loadPath("4BallLeft_Part2", Constants.kMaxSpeedMetersPerSecond*1.3, Constants.kMaxAccelerationMetersPerSecondSquared*1.3);
        fourBallLeft_3 = PathPlanner.loadPath("4BallLeft_Part3", Constants.kMaxSpeedMetersPerSecond*1.3, Constants.kMaxAccelerationMetersPerSecondSquared*1.3, true);

        fiveBallRight_1 = PathPlanner.loadPath("5BallRight_Part1", Constants.kMaxSpeedMetersPerSecond*1.5, Constants.kMaxAccelerationMetersPerSecondSquared*1.2);
        fiveBallRight_2 = PathPlanner.loadPath("5BallRight_Part2", Constants.kMaxSpeedMetersPerSecond*1.5, Constants.kMaxAccelerationMetersPerSecondSquared);
        fiveBallRight_3 = PathPlanner.loadPath("5BallRight_Part3", Constants.kMaxSpeedMetersPerSecond*2, Constants.kMaxAccelerationMetersPerSecondSquared*1.2, true);
        fiveBallRight_4 = PathPlanner.loadPath("5BallRight_Part4", Constants.kMaxSpeedMetersPerSecond*2, Constants.kMaxAccelerationMetersPerSecondSquared*1.2);

        fiveBallRight_v2_1 = PathPlanner.loadPath("5BallRight_v2_Part1", Constants.kMaxSpeedMetersPerSecond*1.3, Constants.kMaxAccelerationMetersPerSecondSquared*1.3);
        fiveBallRight_v2_2 = PathPlanner.loadPath("5BallRight_v2_Part2", Constants.kMaxSpeedMetersPerSecond*1.2, Constants.kMaxAccelerationMetersPerSecondSquared*1.2);
        fiveBallRight_v2_3 = PathPlanner.loadPath("5BallRight_v2_Part3", Constants.kMaxSpeedMetersPerSecond*0.8, Constants.kMaxAccelerationMetersPerSecondSquared*0.8);
        fiveBallRight_v2_4 = PathPlanner.loadPath("5BallRight_v2_Part4", Constants.kMaxSpeedMetersPerSecond*1.25, Constants.kMaxAccelerationMetersPerSecondSquared*1.25, true);


    }
 
    public SplitFFRamseteCommand createCommandFromTrajectory(Trajectory trajectory){
        var ramseteController = new RamseteController();
        var leftController = new PIDController(Constants.kPDriveVel, 0, 0);
        var rightController = new PIDController(Constants.kPDriveVel, 0, 0);
        var table = NetworkTableInstance.getDefault().getTable("troubleshooting");
        var leftReference = table.getEntry("left_reference");
        var leftMeasurement = table.getEntry("left_measurement");

        var rightReference = table.getEntry("right_reference");
        var rightMeasurement = table.getEntry("right_measurement");

        SplitFFRamseteCommand autoCommand  =
          new SplitFFRamseteCommand(
              trajectory,
              m_drivetrain::getPose,
              ramseteController,
              new SimpleMotorFeedforward(
                0.163,
                3.1319,
                0.3),
            new SimpleMotorFeedforward(
                0.15892,
                3.1252,
                0.3818),
              Constants.kDriveKinematics,
              m_drivetrain::getWheelSpeeds,
            leftController,
            rightController,
              // RamseteCommand passes volts to the callback
              (leftVolts, rightVolts) -> {
                  m_drivetrain.tankDriveVolts(rightVolts, leftVolts);
                  leftMeasurement.setNumber(m_drivetrain.getWheelSpeeds().leftMetersPerSecond);
                  leftReference.setNumber(rightController.getSetpoint());

                  rightMeasurement.setNumber(m_drivetrain.getWheelSpeeds().rightMetersPerSecond);
                  rightReference.setNumber(leftController.getSetpoint());
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
