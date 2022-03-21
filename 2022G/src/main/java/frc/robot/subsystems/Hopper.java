package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.*;
import frc.robot.utils.RobotMapGullinkambi;
import frc.robot.utils.Constants.OIConfig;

public class Hopper extends SubsystemBase {
    private static Hopper hopper;
    private LinearFilter bottomSensorFilter, topSensorFilter;

    private CANSparkMax hopperSystem;
    private RelativeEncoder hopperEncoder;
    private SparkMaxPIDController hopperPIDController;
    private double hopperVelSetpoint, hopperPosSetpoint;

    // Define sensors for the hopper to count cargo
    private DigitalInput bottomSensor, topSensor;

    private static UpdateLogs updateLogs = UpdateLogs.getInstance();

    public Hopper() {
        hopperSystem = new CANSparkMax(RobotMapGullinkambi.MOTOR_HOPPER, MotorType.kBrushless);
        hopperSystem.setIdleMode(IdleMode.kBrake);
        hopperSystem.setSmartCurrentLimit(Constants.HOPPER_MAX_CURRENT);

        hopperEncoder = hopperSystem.getEncoder();

        bottomSensorFilter = LinearFilter.singlePoleIIR(0.2, 0.02);
        topSensorFilter = LinearFilter.singlePoleIIR(0.2, 0.02);

        hopperPIDController = hopperSystem.getPIDController();
        hopperPIDController.setP(Constants.HOPPER_VEL_P, 0);
        hopperPIDController.setI(Constants.HOPPER_VEL_I, 0);
        hopperPIDController.setD(Constants.HOPPER_VEL_D, 0);
        hopperPIDController.setFF(Constants.HOPPER_VEL_FF, 0);

        hopperPIDController.setP(Constants.HOPPER_POS_P, 1);
        hopperPIDController.setI(Constants.HOPPER_POS_I, 1);
        hopperPIDController.setD(Constants.HOPPER_POS_D, 1);
        hopperPIDController.setFF(Constants.HOPPER_POS_FF, 1);

        bottomSensor = new DigitalInput(1);
        topSensor = new DigitalInput(0);
    }

    public void periodic() {
        if(Constants.USE_LOGGING){
            updateLogs.updateHopperLogData();
        }
    }

    public static Hopper getInstance() {
        if (hopper == null) {
            hopper = new Hopper();
        }

        return hopper;
    }
    
    public double getHopperVelocity(){
        return hopperEncoder.getVelocity();
    }

    public double getHopperPosition(){
        return hopperEncoder.getPosition();
    }

    public void setHopperVelocity(double rpm){
        hopperVelSetpoint = rpm;
        hopperPIDController.setReference(rpm, ControlType.kVelocity, 0);
    }

    public void setHopperPosition(double pos){
        hopperPosSetpoint = pos;
        hopperPIDController.setReference(pos, ControlType.kPosition, 1);
    }


    public void runHopper(double speed){
        hopperSystem.set(-speed);
    }

    public void stopHopper() {
        runHopper(0);
    }

    public void reverseHopper(double speed) {
        runHopper(-speed);
    }

    public double getHopperCurrent(){
        return hopperSystem.getOutputCurrent();
    }

    public double getHopperMotorTemperature(){
        return hopperSystem.getMotorTemperature();
    }

    public boolean atRPM(){
        return Math.abs(getHopperVelocity()) > Math.abs(hopperVelSetpoint);
    }

    public boolean sensesBallBottom() {
        return !bottomSensor.get();
    }

    public boolean sensesBallTop() {
        return !topSensor.get();
    }

    public boolean sensesBallBottomFiltered() {
        boolean filteredInput = false;
        if(sensesBallBottom()){
            double x = bottomSensorFilter.calculate(1);
            filteredInput = x > Constants.LOWER_SENSOR_INPUT_THRESHOLD;
        } else {
            double x = bottomSensorFilter.calculate(0);
            filteredInput = x > Constants.LOWER_SENSOR_INPUT_THRESHOLD;
        }
        return filteredInput;
    }

    public boolean sensesBallTopFiltered() {
        boolean filteredInput = false;
        if(sensesBallTop()){
            double x = topSensorFilter.calculate(1);
            filteredInput = x > Constants.UPPER_SENSOR_INPUT_THRESHOLD;
        } else {
            double x = topSensorFilter.calculate(0);
            filteredInput = x > Constants.UPPER_SENSOR_INPUT_THRESHOLD;
        }
        return filteredInput;
    }

    public void putSmartDashboardOverrides() {
        SmartDashboard.putNumber("OR: Hopper power", 0.0);
        SmartDashboard.putNumber("OR: Hopper velocity", 0.0);
        SmartDashboard.putNumber("OR: Hopper position", 0.0);

        SmartDashboard.putNumber("Teleop: Hopper shoot speed", Constants.HOPPER_SHOOT_SPEED);
        SmartDashboard.putNumber("OR: Hop Vel P", Constants.HOPPER_VEL_P);
        SmartDashboard.putNumber("OR: Hop Vel I", Constants.HOPPER_VEL_I);
        SmartDashboard.putNumber("OR: Hop Vel D", Constants.HOPPER_VEL_D);
        SmartDashboard.putNumber("OR: Hop Vel FF", Constants.HOPPER_VEL_FF);
        SmartDashboard.putNumber("OR: Hop Pos P", Constants.HOPPER_POS_P);
        SmartDashboard.putNumber("OR: Hop Pos I", Constants.HOPPER_POS_I);
        SmartDashboard.putNumber("OR: Hop Pos D", Constants.HOPPER_POS_D);
        SmartDashboard.putNumber("OR: Hop Pos FF", Constants.HOPPER_POS_FF);

    }

    public void updateHopperInfoOnDashboard(){
        SmartDashboard.putBoolean("Lower sensor", sensesBallBottom());
        SmartDashboard.putBoolean("Upper sensor", sensesBallTop());
        SmartDashboard.putNumber("Hopper position", getHopperPosition());

        if(Constants.OI_CONFIG != OIConfig.COMPETITION){
            SmartDashboard.putNumber("Hopper velocity", getHopperVelocity());
        }
    }

    public void updateHopperFromDashboard() {
        if(SmartDashboard.getNumber("OR: Hopper power", 0) > 0){
            runHopper(SmartDashboard.getNumber("OR: Hopper power", 0.0));
        }
        else if(SmartDashboard.getNumber("OR: Hopper velocity", 0) > 0){
            setHopperVelocity(SmartDashboard.getNumber("OR: Hopper velocity", 0.0));
        }
        else if(SmartDashboard.getNumber("OR: Hopper position", 0.0) != 0){
            setHopperPosition(SmartDashboard.getNumber("OR: Hopper position", 0.0));
        }
        hopperPIDController.setP(SmartDashboard.getNumber("OR: Hop Vel P", Constants.HOPPER_VEL_P), 0);
        hopperPIDController.setI(SmartDashboard.getNumber("OR: Hop Vel I", Constants.HOPPER_VEL_I), 0);
        hopperPIDController.setD(SmartDashboard.getNumber("OR: Hop Vel D", Constants.HOPPER_VEL_D), 0);
        hopperPIDController.setFF(SmartDashboard.getNumber("OR: Hop Vel FF", Constants.HOPPER_VEL_FF), 0);

        hopperPIDController.setP(SmartDashboard.getNumber("OR: Hop Pos P", Constants.HOPPER_POS_P), 1);
        hopperPIDController.setI(SmartDashboard.getNumber("OR: Hop Pos I", Constants.HOPPER_POS_I), 1);
        hopperPIDController.setD(SmartDashboard.getNumber("OR: Hop Pos D", Constants.HOPPER_POS_D), 1);
        hopperPIDController.setFF(SmartDashboard.getNumber("OR: Hop Pos FF", Constants.HOPPER_POS_FF), 1);
    }
}
