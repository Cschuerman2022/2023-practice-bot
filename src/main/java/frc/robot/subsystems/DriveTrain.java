// RobotBuilder Version: 5.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.*;
import frc.robot.util.OIReporters;
//import frc.robot.OI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.util.RobotMath;

/**
 * 
 * @author 23bbrenner
 */
public class DriveTrain extends SubsystemBase {

    private DifferentialDrive m_drive;
    private MotorControllerGroup m_driveLeft;
    private MotorControllerGroup m_driveRight;
    private RobotContainer m_robotContainer;

    // Constants
    private double m_turnGain = DriveConstants.kTurnGain;
    private double m_deadband = DriveConstants.kDeadband;
    private double m_driveGain = DriveConstants.kDriveGain;
    private boolean brakesEnabled = true;

    // Hardware
    private WPI_TalonSRX driveLeftLeader;
    private WPI_VictorSPX driveLeftFollower;
    private WPI_TalonSRX driveRightLeader;
    private WPI_VictorSPX driveRightFollower;

    // Controllers
    protected XboxController driveController;

    // Global Variables
    private DifferentialDriveOdometry odometry;
    private AHRS gyro;
    private double inversionMultiplier = 1;
    private boolean firstRun = true;
    private double initialHeading = 0;


    public DriveTrain() {

        driveLeftLeader = new WPI_TalonSRX(DriveConstants.kLeftLeaderPort);
        driveLeftLeader.configFactoryDefault();
        driveLeftFollower = new WPI_VictorSPX(DriveConstants.kLeftFollowerPort);
        driveLeftFollower.configFactoryDefault();

        driveRightLeader = new WPI_TalonSRX(DriveConstants.kRightLeaderPort);
        driveRightLeader.configFactoryDefault();
        driveRightFollower = new WPI_VictorSPX(DriveConstants.kRightFollowerPort);
        driveRightFollower.configFactoryDefault();

        /* Set the peak and nominal outputs */
        driveLeftLeader.configNominalOutputForward(0, 30);
        driveLeftLeader.configNominalOutputReverse(0, 30);
        driveLeftLeader.configPeakOutputForward(1, 30);
        driveLeftLeader.configPeakOutputReverse(-1, 30);

        driveRightLeader.configNominalOutputForward(0, 30);
        driveRightLeader.configNominalOutputReverse(0, 30);
        driveRightLeader.configPeakOutputForward(1, 30);
        driveRightLeader.configPeakOutputReverse(-1, 30);

        // Current Limit - prevents breakers from tripping in PDP
        // !!arbitary values, need update!!
        driveLeftLeader.configSupplyCurrentLimit(new
        SupplyCurrentLimitConfiguration(true, 35, 34, 10));
        driveRightLeader.configSupplyCurrentLimit(new
        SupplyCurrentLimitConfiguration(true, 35, 34, 10));

        // Invert and set Break Mode
        driveLeftLeader.setInverted(false);
        driveLeftFollower.setInverted(false);
        driveLeftLeader.setNeutralMode(NeutralMode.Coast);
        driveLeftFollower.setNeutralMode(NeutralMode.Coast);

        driveRightLeader.setInverted(true);
        driveRightFollower.setInverted(true);
        driveRightLeader.setNeutralMode(NeutralMode.Coast);
        driveRightFollower.setNeutralMode(NeutralMode.Coast);

        // Config Left Motors
        driveLeftLeader.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
        driveLeftLeader.setSensorPhase(false);
        driveLeftLeader.configNeutralDeadband(m_deadband);
        driveLeftFollower.configNeutralDeadband(m_deadband);

        // Config Right Motors
        driveRightLeader.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
        driveRightLeader.setSensorPhase(true);
        driveRightLeader.configNeutralDeadband(m_deadband);
        driveRightLeader.configNeutralDeadband(m_deadband);

        // Config differential drive
        driveLeftFollower.follow(driveLeftLeader);
        driveRightFollower.follow(driveRightLeader);

        m_driveLeft = new MotorControllerGroup(driveLeftLeader, driveLeftFollower);
        m_driveRight = new MotorControllerGroup(driveRightLeader, driveRightFollower);
        
        driveLeftLeader.configOpenloopRamp(DriveConstants.kRampRate);
        driveRightLeader.configOpenloopRamp(DriveConstants.kRampRate);

        m_drive = new DifferentialDrive(m_driveLeft, m_driveRight);
        m_drive.setSafetyEnabled(true);
        m_drive.setExpiration(0.1);

        // Config sensors

        gyro = new AHRS(Port.kMXP);
        gyro.reset();
        gyro.calibrate();

        odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), m_driveGain, m_deadband); // ask Kelly about the purpose of drivegain & deadband here
        
    }


    @Override
    public void periodic() {
        if (RobotContainer.getInstance().getOI().shouldInvertMotors()) {
            invertMotors();
        }
        if (RobotContainer.getInstance().getOI().shouldEnableBrakes()) {
            brakesEnabled = true;
        }
        enableBrakes(brakesEnabled);

        if(firstRun){
			initialHeading = gyro.getAngle();
			firstRun = false;
		}
        odometry.update(gyro.getRotation2d(), leftMeters(), rightMeters());
        
        updateSmartDashboard();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    /* DRIVE */

    public void stopMotors() {
        driveLeftLeader.stopMotor();
        driveRightLeader.stopMotor();
    }
    
    public DifferentialDriveWheelSpeeds getWheelSpeeds() { // kinematics

        return new DifferentialDriveWheelSpeeds(
            RobotMath.magEncoderConvertMetersPerSec(driveLeftLeader.getSelectedSensorVelocity()), 
            RobotMath.magEncoderConvertMetersPerSec(driveRightLeader.getSelectedSensorVelocity()));
                
    }

    public void setWheelSpeedsAuto (double leftSpeed, double rightSpeed){
        
        driveLeftLeader.set(leftSpeed);
        driveRightLeader.set(rightSpeed);
    }

    public void setWheelSpeeds(double leftSpeed, double rightSpeed) { // from DriveCommand

        // driveLeftLeader.set(leftSpeed*inversionMultiplier);
        // driveRightLeader.set(rightSpeed*inversionMultiplier);

        OIReporters.DriveReporters.leftSpeedFromCommand = (leftSpeed * inversionMultiplier);
        OIReporters.DriveReporters.rightSpeedFromCommand = (rightSpeed * inversionMultiplier);
    }

    public void curvatureDrive(double speed, double rotation, boolean semiCurvature) {
       // SlewRateLimiter speedFilter = new SlewRateLimiter(Constants.DriveConstants.kSlewRateLimiter);
        m_drive.curvatureDrive (-speed * inversionMultiplier, -rotation * inversionMultiplier, semiCurvature);
    }

    public void arcadeDrive(double speed, double rotation) {
       // SlewRateLimiter filter = new SlewRateLimiter(Constants.DriveConstants.kSlewRateLimiter);
        m_drive.arcadeDrive(speed * inversionMultiplier, rotation * inversionMultiplier);
    }

    // for bus voltage pid control
    public void tankDriveVolts(double m_driveLeftVolts, double m_driveRightVolts) {
        driveLeftLeader.setVoltage(m_driveLeftVolts);
        driveRightLeader.setVoltage(m_driveRightVolts);
        m_drive.feed();
    }

    public void enableBrakes(boolean enabled) {
        
        // SmartDashboard.putBoolean("Brakes Enabled", brakesEnabled);
        if (enabled) {
        driveLeftLeader.setNeutralMode(NeutralMode.Brake);
        driveRightLeader.setNeutralMode(NeutralMode.Brake);
        
        OIReporters.DriveReporters.brakesEnabled = true;
        }
        else {
        driveRightLeader.setNeutralMode(NeutralMode.Coast);
        driveRightFollower.setNeutralMode(NeutralMode.Coast);

        OIReporters.DriveReporters.brakesEnabled = false;
        }
        brakesEnabled = true;
    }

    public void invertMotors() {
        inversionMultiplier *= -1;
        OIReporters.DriveReporters.inversionMult = inversionMultiplier;
    }
    

    /* SENSORS */

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetGyro(){
		initialHeading = gyro.getAngle();
	}
    
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        driveLeftLeader.setSelectedSensorPosition(0);
        driveRightLeader.setSelectedSensorPosition(0);
    }

    public double leftMeters() {
        return RobotMath.magEncoderConvertMeters(driveLeftLeader.getSelectedSensorPosition()) * inversionMultiplier;
    }    

   public double rightMeters() {
       return RobotMath.magEncoderConvertMeters(driveRightLeader.getSelectedSensorPosition()) * inversionMultiplier;
     }

    public double getAverageEncoderDistance() {
        return Math.abs(((leftMeters() + rightMeters()) / 2.0));
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading(){
		//initialHeading = gyro.getAngle();
		return gyro.getAngle() - initialHeading;
	}
    // public Rotation2d getHeading() {
    //     return Rotation2d.fromDegrees(-gyro.getAngle());
    // }

    public double getTurnRate() {
        return -gyro.getRate();
    }

    public void updateSmartDashboard() {
        OIReporters.DriveReporters.gyroYaw = gyro.getYaw();
        OIReporters.DriveReporters.gyroPitch = gyro.getPitch();
        OIReporters.DriveReporters.gyroRoll = gyro.getRoll();
        OIReporters.DriveReporters.gyroAngle = gyro.getAngle();

        OIReporters.DriveReporters.gyroMoving = gyro.isMoving();
        OIReporters.DriveReporters.gyroConnected = gyro.isConnected();
        OIReporters.DriveReporters.gyroCalibrating = gyro.isCalibrating();

       OIReporters.DriveReporters.leftEncoderSpeed= driveLeftLeader.getSelectedSensorVelocity();
       OIReporters.DriveReporters.rightEncoderSpeed = driveRightLeader.getSelectedSensorVelocity();
       OIReporters.DriveReporters.leftEncoderDistance = driveLeftLeader.getSelectedSensorPosition();
       OIReporters.DriveReporters.rightEncoderDistance = driveRightLeader.getSelectedSensorPosition();
    }

   
}
