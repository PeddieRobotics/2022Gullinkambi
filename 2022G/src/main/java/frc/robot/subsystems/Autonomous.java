package frc.robot.subsystems;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Enumeration;
import java.util.Hashtable;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.AutoCommands.FiveBallRightDCMPBackAway;
import frc.robot.commands.AutoCommands.FiveBallRightSeneca;
import frc.robot.commands.AutoCommands.FiveBallRightDCMP;
import frc.robot.commands.AutoCommands.FiveBallRightDCMPNoPivot;
import frc.robot.commands.AutoCommands.FourBallLeft;
import frc.robot.commands.AutoCommands.FourBallRight;
import frc.robot.commands.AutoCommands.TwoBallLeftTrollLong;
import frc.robot.commands.AutoCommands.TwoBallLeftTrollShort;
import frc.robot.commands.AutoCommands.TwoBallLong;
import frc.robot.commands.AutoCommands.TwoBallOneTrollFender;
import frc.robot.commands.AutoCommands.TwoBallOneTrollHangar;
import frc.robot.commands.AutoCommands.TwoBallShort;
import frc.robot.utils.Constants;

public class Autonomous extends SubsystemBase {

    private final Drivetrain m_drivetrain;
    private static Autonomous autonomous;

    private SendableChooser<Command> autoRoutineSelector;
    private Hashtable<String,Command> autoRoutines;

    private Trajectory sanityCheck, pathVerification;
    private Trajectory twoBallLong, twoBallShort, twoBallOneTroll_1, twoBallOneTroll_2;
    private Trajectory fourBallRight_1, fourBallRight_2, fourBallLeft_1, fourBallLeft_2, fourBallLeft_3;
    private Trajectory fiveBallRightSeneca_1, fiveBallRightSeneca_2, fiveBallRightSeneca_3, fiveBallRightSeneca_4, fiveBallRightDCMP_1, fiveBallRightDCMP_2, fiveBallRightDCMP_3, fiveBallRightDCMP_4, fiveBallRightDCMPNoPivot_1, fiveBallRightDCMPNoPivot_2, fiveBallRightDCMPNoPivot_3, fiveBallRightDCMPNoPivot_4, fiveBallRightDCMPBackAway_1, fiveBallRightDCMPBackAway_2, fiveBallRightDCMPBackAway_3, fiveBallRightDCMPBackAway_4, fiveBallRightDCMPBackAway_5;

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

        autoRoutines.put("2 Ball (Long)", new TwoBallLong(twoBallLong.getInitialPose(), createCommandFromTrajectory(twoBallLong)));
        autoRoutines.put("2 Ball (Short)", new TwoBallShort(twoBallShort.getInitialPose(), createCommandFromTrajectory(twoBallShort)));

        autoRoutines.put("2 Ball 1 Troll Hangar", new TwoBallOneTrollHangar(twoBallOneTroll_1.getInitialPose(), createCommandFromTrajectory(twoBallOneTroll_1), createCommandFromTrajectory(twoBallOneTroll_2)));
        autoRoutines.put("2 Ball 1 Troll Fender", new TwoBallOneTrollFender(twoBallOneTroll_1.getInitialPose(), createCommandFromTrajectory(twoBallOneTroll_1), createCommandFromTrajectory(twoBallOneTroll_2)));

        autoRoutines.put("4 Ball (Right)", new FourBallRight(fourBallRight_1.getInitialPose(), createCommandFromTrajectory(fourBallRight_1), createCommandFromTrajectory(fourBallRight_2)));
        autoRoutines.put("4 Ball (Left)", new FourBallLeft(fourBallLeft_1.getInitialPose(), createCommandFromTrajectory(fourBallLeft_1), createCommandFromTrajectory(fourBallLeft_2), createCommandFromTrajectory(fourBallLeft_3)));

        autoRoutines.put("5 Ball (Right) Seneca", new FiveBallRightSeneca(fiveBallRightSeneca_1.getInitialPose(), createCommandFromTrajectory(fiveBallRightSeneca_1), createCommandFromTrajectory(fiveBallRightSeneca_2), createCommandFromTrajectory(fiveBallRightSeneca_3), createCommandFromTrajectory(fiveBallRightSeneca_4)));
        autoRoutines.put("5 Ball (Right) DCMP", new FiveBallRightDCMP(fiveBallRightDCMP_1.getInitialPose(), createCommandFromTrajectory(fiveBallRightDCMP_1), createCommandFromTrajectory(fiveBallRightDCMP_2), createCommandFromTrajectory(fiveBallRightDCMP_3), createCommandFromTrajectory(fiveBallRightDCMP_4)));
        autoRoutines.put("5 Ball (Right) DCMP NO PIVOT", new FiveBallRightDCMPNoPivot(fiveBallRightDCMPNoPivot_1.getInitialPose(), createCommandFromTrajectory(fiveBallRightDCMPNoPivot_1), createCommandFromTrajectory(fiveBallRightDCMPNoPivot_2), createCommandFromTrajectory(fiveBallRightDCMPNoPivot_3), createCommandFromTrajectory(fiveBallRightDCMPNoPivot_4)));
        autoRoutines.put("5 Ball (Right) DCMP BackAway", new FiveBallRightDCMPBackAway(fiveBallRightDCMPBackAway_1.getInitialPose(), createCommandFromTrajectory(fiveBallRightDCMPBackAway_1), createCommandFromTrajectory(fiveBallRightDCMPBackAway_2), createCommandFromTrajectory(fiveBallRightDCMPBackAway_3), createCommandFromTrajectory(fiveBallRightDCMPBackAway_4), createCommandFromTrajectory(fiveBallRightDCMPBackAway_5)));
   
    }

    public Command returnAutonomousCommand() {
        return autoRoutineSelector.getSelected();
    }

    private void defineAutoPaths(){
        // sanityCheck = PathPlanner.loadPath("SanityCheck", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared);
        // pathVerification = PathPlanner.loadPath("PathVerification", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared);

        twoBallLong = PathPlanner.loadPath("2BallLong", Constants.kMaxSpeedMetersPerSecond*0.8, Constants.kMaxAccelerationMetersPerSecondSquared*0.8);
        twoBallShort = PathPlanner.loadPath("2BallShort", Constants.kMaxSpeedMetersPerSecond*0.8, Constants.kMaxAccelerationMetersPerSecondSquared*0.8);

        twoBallOneTroll_1 = PathPlanner.loadPath("2B1T_1", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared);
        twoBallOneTroll_2 = PathPlanner.loadPath("2B1T_2", Constants.kMaxSpeedMetersPerSecond*0.8, Constants.kMaxAccelerationMetersPerSecondSquared*0.8);
      
        fourBallRight_1 = PathPlanner.loadPath("4BallRight_Part1", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared);
        fourBallRight_2 = PathPlanner.loadPath("4BallRight_Part2", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared);
        
        fourBallLeft_1 = PathPlanner.loadPath("4BallLeft_Part1", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared);
        fourBallLeft_2 = PathPlanner.loadPath("4BallLeft_Part2", Constants.kMaxSpeedMetersPerSecond*1.3, Constants.kMaxAccelerationMetersPerSecondSquared*1.3);
        fourBallLeft_3 = PathPlanner.loadPath("4BallLeft_Part3", Constants.kMaxSpeedMetersPerSecond*1.3, Constants.kMaxAccelerationMetersPerSecondSquared*1.3, true);

        fiveBallRightSeneca_1 = PathPlanner.loadPath("5BallRight_Seneca_Part1", Constants.kMaxSpeedMetersPerSecond*1.2, Constants.kMaxAccelerationMetersPerSecondSquared*1.2);
        fiveBallRightSeneca_2 = PathPlanner.loadPath("5BallRight_Seneca_Part2", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared);
        fiveBallRightSeneca_3 = PathPlanner.loadPath("5BallRight_Seneca_Part3", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared);
        fiveBallRightSeneca_4 = PathPlanner.loadPath("5BallRight_Seneca_Part4", Constants.kMaxSpeedMetersPerSecond*1.3, Constants.kMaxAccelerationMetersPerSecondSquared*1.3, true);
      
        fiveBallRightDCMP_1 = PathPlanner.loadPath("5BallRight_DCMP_Part1", Constants.kMaxSpeedMetersPerSecond*1.2, Constants.kMaxAccelerationMetersPerSecondSquared*1.2);
        fiveBallRightDCMP_2 = PathPlanner.loadPath("5BallRight_DCMP_Part2", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared);
        fiveBallRightDCMP_3 = PathPlanner.loadPath("5BallRight_DCMP_Part3", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared);
        fiveBallRightDCMP_4 = PathPlanner.loadPath("5BallRight_DCMP_Part4", Constants.kMaxSpeedMetersPerSecond*1.3, Constants.kMaxAccelerationMetersPerSecondSquared*1.3, true);
       
        fiveBallRightDCMPNoPivot_1 = PathPlanner.loadPath("5BallRight_DCMPNoPivot_Part1", Constants.kMaxSpeedMetersPerSecond*1.2, Constants.kMaxAccelerationMetersPerSecondSquared*1.2);
        fiveBallRightDCMPNoPivot_2 = PathPlanner.loadPath("5BallRight_DCMPNoPivot_Part2", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared);
        fiveBallRightDCMPNoPivot_3 = PathPlanner.loadPath("5BallRight_DCMPNoPivot_Part3", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared);
        fiveBallRightDCMPNoPivot_4 = PathPlanner.loadPath("5BallRight_DCMPNoPivot_Part4", Constants.kMaxSpeedMetersPerSecond*1.3, Constants.kMaxAccelerationMetersPerSecondSquared*1.3, true);

        fiveBallRightDCMPBackAway_1 = PathPlanner.loadPath("5BallRight_DCMPBackAway_Part1", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared);
        fiveBallRightDCMPBackAway_2 = PathPlanner.loadPath("5BallRight_DCMPBackAway_Part2", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared);
        fiveBallRightDCMPBackAway_3 = PathPlanner.loadPath("5BallRight_DCMPBackAway_Part3", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared);
        fiveBallRightDCMPBackAway_4 = PathPlanner.loadPath("5BallRight_DCMPBackAway_Part4", Constants.kMaxSpeedMetersPerSecond*0.2, Constants.kMaxAccelerationMetersPerSecondSquared, true);
        fiveBallRightDCMPBackAway_5 = PathPlanner.loadPath("5BallRight_DCMPBackAway_Part5", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared, true);

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
