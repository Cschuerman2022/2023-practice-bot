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
import frc.robot.OIReporters;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.*;
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

/**
 * 
 * @author 23bbrenner
 */
public class DriveTrain extends SubsystemBase {

    private DifferentialDrive m_drive;
    private MotorControllerGroup m_driveLeft;
    private MotorControllerGroup m_driveRight;

    // Constants
    private double m_turnGain = DriveConstants.kTurnGain;
    private double m_deadband = DriveConstants.kDeadband;
    private double m_driveGain = DriveConstants.kDriveGain;
    private boolean brakesEnabled = false;

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
        // driveLeftLeader.setNeutralMode(NeutralMode.Coast);
        // driveLeftFollower.setNeutralMode(NeutralMode.Coast);

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
        if (OI.getInstance().shouldInvertMotors()) {
            invertMotors();
        }
        if (OI.getInstance().shouldEnableBrakes()) {
            brakesEnabled = true;
        }
        enableBrakes(brakesEnabled);

        odometry.update(gyro.getRotation2d(), leftMeters(), rightMeters());
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
    
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        double leftEncoderSpeedRaw = driveLeftLeader.getSelectedSensorVelocity();
        double rightEncoderSpeedRaw = driveRightFollower.getSelectedSensorPosition();

        double leftEncoderSpeedConverted = ((driveLeftLeader.getSelectedSensorVelocity() / Constants.kUnitsPerRev) / Constants.kGearRatio) / 1000;
        double rightEncoderSpeedConverted = ((driveRightLeader.getSelectedSensorVelocity() / Constants.kUnitsPerRev) / Constants.kGearRatio) / 1000;

        
        SmartDashboard.putNumber("Raw Left Speed from Encoders", leftEncoderSpeedRaw);
        SmartDashboard.putNumber("Raw Right Speed from Encoders", rightEncoderSpeedRaw);

        
        SmartDashboard.putNumber("Converted Left Speed from Encoders", leftEncoderSpeedConverted);
        SmartDashboard.putNumber("Converted Speed from Encoders", rightEncoderSpeedConverted);

        return new DifferentialDriveWheelSpeeds(
                ((driveLeftLeader.getSelectedSensorVelocity() / Constants.kUnitsPerRev) / Constants.kGearRatio) / 1000,
                ((driveRightLeader.getSelectedSensorVelocity() / Constants.kUnitsPerRev) / Constants.kGearRatio) / 1000); // Sensor Units per 100ms -> Meters per sec
        }

    public void setWheelSpeeds(double leftSpeed, double rightSpeed) {
        SmartDashboard.putNumber("Left Speed from DriveCommand", (leftSpeed * inversionMultiplier));
        SmartDashboard.putNumber("Right Speed from DriveCommand", (rightSpeed * inversionMultiplier));

        // driveLeftLeader.set(leftSpeed*inversionMultiplier);
        // driveRightLeader.set(rightSpeed*inversionMultiplier);
    }

     // already configured in DriveCommand if using setWheelSpeeds
     public void setMaxOutput(double maxOutput) {
        m_drive.setMaxOutput(maxOutput);
     }

    public void curvatureDrive(double speed, double rotation, boolean semiCurvature) {
        SlewRateLimiter filter = new SlewRateLimiter(Constants.DriveConstants.kSlewRateLimiter);
        m_drive.curvatureDrive(filter.calculate(speed * inversionMultiplier), -rotation * inversionMultiplier, semiCurvature);
    }

    public void arcadeDrive(double speed, double rotation) {
        SlewRateLimiter filter = new SlewRateLimiter(Constants.DriveConstants.kSlewRateLimiter);
        m_drive.arcadeDrive(filter.calculate(speed * inversionMultiplier), rotation * inversionMultiplier);
    }

    // ask Kelly if still relevent 
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
        
        OIReporters.brakesEnabled = true;
        }
        else {
        driveRightLeader.setNeutralMode(NeutralMode.Coast);
        driveRightFollower.setNeutralMode(NeutralMode.Coast);

        OIReporters.brakesEnabled = false;
        }
        brakesEnabled = false;
    }

    public void invertMotors() {
        inversionMultiplier *= -1;
        OIReporters.inversionMult = inversionMultiplier;
    }
    

    /* SENSORS */

    public Pose2d getPose() {
        return odometry.getPoseMeters();
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
        return (((Math.PI * Units.inchesToMeters(Constants.kWheelDiameterInches)) * (driveLeftLeader.getSelectedSensorPosition() 
        / (Constants.kUnitsPerRev)) / Constants.kGearRatio)) * inversionMultiplier;
    }    

   public double rightMeters() {
       return (((Math.PI * Units.inchesToMeters(Constants.kWheelDiameterInches)) * (driveRightLeader.getSelectedSensorPosition() 
       / (Constants.kUnitsPerRev)) / Constants.kGearRatio)) * inversionMultiplier;
     }

    public double getAverageEncoderDistance() {
        return ((leftMeters() + rightMeters()) / 2.0);

        // return (((driveLeftLeader.getSelectedSensorPosition() / Constants.kUnitsPerRev) / Constants.kGearRatio) + 
        // ((driveRightLeader.getSelectedSensorPosition() / Constants.kUnitsPerRev) / Constants.kGearRatio) / 2.0);

        //return (driveLeftLeader.getSelectedSensorPosition() + driveRightLeader.getSelectedSensorPosition() / 2.0);
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        gyro.reset();
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(-gyro.getAngle());
    }

    public double getTurnRate() {
        return -gyro.getRate();
    }

    public void updateSmartDashboard() {
        SmartDashboard.putNumber("imu-yaw", gyro.getYaw());
        SmartDashboard.putNumber("imu-pitch", gyro.getPitch());
        SmartDashboard.putNumber("imu-roll", gyro.getRoll());
        SmartDashboard.putNumber("imu-angle", gyro.getAngle());

        SmartDashboard.putBoolean("imu-moving", gyro.isMoving());
        SmartDashboard.putBoolean("imu-connected", gyro.isConnected());
        SmartDashboard.putBoolean("imu-calibrating", gyro.isCalibrating());

        SmartDashboard.putNumber("Left Velocity", driveLeftLeader.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Right Velocity", driveRightLeader.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Left Distance", driveLeftLeader.getSelectedSensorPosition());
        SmartDashboard.putNumber("Right Distance", driveRightLeader.getSelectedSensorPosition());

        

    }

   
}
