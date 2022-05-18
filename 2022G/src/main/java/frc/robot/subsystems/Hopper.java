package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.OIConfig;
import frc.robot.utils.RobotMapGullinkambi;

public class Hopper extends SubsystemBase {
    private static Hopper hopper;
    private LinearFilter bottomSensorFilter, topSensorFilter;

    private CANSparkMax hopperSystem;
    private RelativeEncoder hopperEncoder;
    private SparkMaxPIDController hopperPIDController;
    private double hopperVelSetpoint;
    private double initialRevUpTime, elapsedRevUpTime;
    private boolean revUpStarted, revUpEnded;

    // Define sensors for the hopper to count cargo
    private DigitalInput bottomSensor, topSensor;

    public Hopper() {
        hopperSystem = new CANSparkMax(RobotMapGullinkambi.MOTOR_HOPPER, MotorType.kBrushless);
        hopperSystem.setIdleMode(IdleMode.kBrake);
        hopperSystem.setSmartCurrentLimit(Constants.HOPPER_MAX_CURRENT);

        hopperEncoder = hopperSystem.getEncoder();

        hopperVelSetpoint = 0.0;
        initialRevUpTime = 0.0;
        revUpStarted = false;
        revUpEnded = false;

        bottomSensorFilter = LinearFilter.singlePoleIIR(0.2, 0.02);
        topSensorFilter = LinearFilter.singlePoleIIR(0.2, 0.02);

        hopperPIDController = hopperSystem.getPIDController();
        hopperPIDController.setP(Constants.HOPPER_VEL_P);
        hopperPIDController.setI(Constants.HOPPER_VEL_I);
        hopperPIDController.setD(Constants.HOPPER_VEL_D);
        hopperPIDController.setFF(Constants.HOPPER_VEL_FF);

        bottomSensor = new DigitalInput(1);
        topSensor = new DigitalInput(0);
    }

    public void periodic() {

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
        hopperPIDController.setReference(rpm, ControlType.kVelocity);
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

    public boolean isHopperFull(){
        return sensesBallTop() && sensesBallBottom();
    }

    public void putSmartDashboardOverrides() {
        SmartDashboard.putNumber("OR: Hopper power", 0.0);
        SmartDashboard.putNumber("OR: Hopper velocity", 0.0);
        SmartDashboard.putNumber("Teleop: Hopper shoot layup speed", Constants.HOPPER_SHOOT_LAYUP_SPEED);
        SmartDashboard.putNumber("Teleop: Hopper shoot LL speed", Constants.HOPPER_SHOOT_LL_SPEED);
        SmartDashboard.putNumber("OR: Hop Vel P", Constants.HOPPER_VEL_P);
        SmartDashboard.putNumber("OR: Hop Vel I", Constants.HOPPER_VEL_I);
        SmartDashboard.putNumber("OR: Hop Vel D", Constants.HOPPER_VEL_D);
        SmartDashboard.putNumber("OR: Hop Vel FF", Constants.HOPPER_VEL_FF);

    }

    public void updateHopperInfoOnDashboard(){
        SmartDashboard.putBoolean("Lower sensor", sensesBallBottom());
        SmartDashboard.putBoolean("Upper sensor", sensesBallTop());
        SmartDashboard.putBoolean("Upper Sensor Filtered", sensesBallTopFiltered());
        SmartDashboard.putBoolean("Hopper full", isHopperFull());
        SmartDashboard.putNumber("Hopper velocity", getHopperVelocity());

        if(Constants.OI_CONFIG != OIConfig.COMPETITION){
            SmartDashboard.putNumber("Hopper position", getHopperPosition());
        }
    }

    public void updateHopperFromDashboard() {
        if(SmartDashboard.getNumber("OR: Hopper power", 0) != 0){
            runHopper(SmartDashboard.getNumber("OR: Hopper power", 0.0));
        }
        else{
            setHopperVelocity(SmartDashboard.getNumber("OR: Hopper velocity", 0.0));
        }
        hopperPIDController.setP(SmartDashboard.getNumber("OR: Hop Vel P", Constants.HOPPER_VEL_P));
        hopperPIDController.setI(SmartDashboard.getNumber("OR: Hop Vel I", Constants.HOPPER_VEL_I));
        hopperPIDController.setD(SmartDashboard.getNumber("OR: Hop Vel D", Constants.HOPPER_VEL_D));
        hopperPIDController.setFF(SmartDashboard.getNumber("OR: Hop Vel FF", Constants.HOPPER_VEL_FF));
    }
}
