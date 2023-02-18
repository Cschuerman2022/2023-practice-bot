// RobotBuilder Version: 5.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: RobotContainer.

package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior; 
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  
    // private static RobotContainer m_robotContainer = new RobotContainer();
    // The robot's subsystems
    // public final DriveTrain m_driveTrain = new DriveTrain();
    
    private static RobotContainer instanceRobotContainer;

    private DriveTrain driveTrain;
    // private Vision vision;
    // private Claw claw;
    // private Wrist wrist;
    // private Arm arm;
    // private Turret turret;

    private DriveCommand driveCommand;

    private XboxController driveController = OI.getInstance().getDriveController();

    // Creates new gamepads connected to Xbox Controllers
    // private static XboxController driveController = new XboxController(0);
    // private static XboxController armController = new XboxController(1);

    // // Command choosers
    // SendableChooser<Command> m_autoChooser = new SendableChooser<>();

    // // Object choosers
    // public final SendableChooser<Integer> driverControlsChooser = new SendableChooser<>();
    // public final SendableChooser<Integer> controllerScalingChooser = new SendableChooser<>();
    // public final SendableChooser<Integer> driveModeChooser = new SendableChooser<>();

  
  // public static RobotContainer getInstance() {
  //   if (instanceRobotContainer == null) {
  //     instanceRobotContainer = new RobotContainer();
  //   }
  //   return instanceRobotContainer;
  // }

  
  public RobotContainer() {
    OI.getInstance().configureSmartDashboard();
    //OI.getInstance().configureCommandButtons();
    OI.getInstance().configureButtonBindings();
    initSubsystems();
    
  }      

  private void initSubsystems() {
    
    driveTrain = new DriveTrain();
    // vision = new Vision();
    // claw = new Claw();
    // wrist = new Wrist();
    // arm = new Arm();
    // turret = new Turret();
  }

//   public void configureCommandButtons() {
//     m_commandDriveController.leftStick().onTrue(new SafeReset());
// }

  // private void configureButtonBindings() {
  //   invertMotors();
  //   enableBrakes(); 
  // }

  // public boolean invertMotors() {
  //   return driveController.getAButtonPressed();
  // }

  // public boolean enableBrakes() {
  //   return driveController.getBButton();
  // }
      
//   private void configureSmartDashboard() {

//     // Choosers
//     m_autoChooser.setDefaultOption("Simple Autonomous", getAutonomousCommand());

//     driverControlsChooser.setDefaultOption("Left Stick", 0);
//     driverControlsChooser.addOption("Trigger Acceleration", 1);

//     controllerScalingChooser.setDefaultOption("Cubic", 0);
//     controllerScalingChooser.addOption("Linear", 1);
//     controllerScalingChooser.addOption("Squared", 2);
//     controllerScalingChooser.addOption("Limited Polynomic", 3);

//     driveModeChooser.setDefaultOption("Semi Curvature", 0);
//     driveModeChooser.addOption("Reg Curvature", 1);
//     driveModeChooser.addOption("Arcade", 2);
    
//     SmartDashboard.putData("Autonomous Mode", m_autoChooser);
//     SmartDashboard.putData("Driver Controls", driverControlsChooser);
//     SmartDashboard.putData("Drive Controller Scaling", controllerScalingChooser);
//     SmartDashboard.putData("Drive Mode", driveModeChooser);

//     // Constants
//     SmartDashboard.putNumber("Ramp Rate",Constants.DriveConstants.kRampRate);
//     SmartDashboard.putNumber("Ramp Rate",Constants.DriveConstants.kDeadband);
//     SmartDashboard.putNumber("Ramp Rate",Constants.DriveConstants.kSlewRateLimiter);
//   }

//   public XboxController getDriveController() {
//   return driveController;
// }

  public void manualDrive() {
    driveCommand = new DriveCommand(driveTrain, driveController);
    driveTrain.setDefaultCommand(driveCommand);
}

  public void safeReset() {
  driveTrain.stopMotors();
  driveTrain.resetEncoders();
}

 
//   /**
//    * Use this to pass the autonomous command to the main {@link Robot} class.
//    *
//    * @return the command to run in autonomous
//   */

  public DriveTrain getDriveTrain() {
    return driveTrain;
  }

//   public int getDriverControlsChooser() {
//     return driverControlsChooser.getSelected();
//   }

//   public int getDriveModeChooser() {
//     return driveModeChooser.getSelected();
//   }

//   public int getControllerScalingChooser() {
//     return controllerScalingChooser.getSelected();
//   }

//   public Command getAutonomousCommand() {
//     // The selected command will be run in autonomous
//     return OI.getInstance().getAutonomousCommand();
// }

 
}
  