// // RobotBuilder Version: 5.0
// //
// // This file was generated by RobotBuilder. It contains sections of
// // code that are automatically generated and assigned by robotbuilder.
// // These sections will be updated in the future when you export to
// // Java from RobotBuilder. Do not put any code or make any change in
// // the blocks indicating autogenerated code or it will be lost on an
// // update. Deleting the comments indicating the section will prevent
// // it from being updated in the future.

// // ROBOTBUILDER TYPE: PIDSubsystem.

// package frc.robot.subsystems;

// import frc.robot.commands.*;
// import edu.wpi.first.wpilibj2.command.PIDSubsystem;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.livewindow.LiveWindow;
// import edu.wpi.first.math.MathUtil;

 
// import com.revrobotics.CANSparkMax.IdleMode;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import edu.wpi.first.wpilibj.CounterBase.EncodingType;
// import edu.wpi.first.wpilibj.Encoder;

     

// /**
//  *
//  */
// public class Turret extends PIDSubsystem {

//     // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

//     // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

//     // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
// private CANSparkMax turretMotor;private Encoder turretEncoder;
//     //P I D Variables
//     private static final double kP = 1.0;
//     private static final double kI = 0.0;
//     private static final double kD = 0.0;
//     private static final double kF = 0.0;
//     // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

//     // Initialize your subsystem here
//     public Turret() {
         
//         super(new PIDController(kP, kI, kD));
//         getController().setTolerance(0.2);

    

         
// turretMotor = new CANSparkMax(11, MotorType.kBrushless);
 
//  turretMotor.restoreFactoryDefaults();  
// turretMotor.setInverted(false);
// turretMotor.setIdleMode(IdleMode.kCoast);
// turretMotor.burnFlash();
  

// turretEncoder = new Encoder(0, 1, false, EncodingType.k4X);
//  addChild("turretEncoder",turretEncoder);
//  turretEncoder.setDistancePerPulse(1.0);


     

//         // Use these to get going:
//         // setSetpoint() -  Sets where the PID controller should move the system
//         //                  to
//         // enable() - Enables the PID controller.
//     }

//     @Override
//     public void periodic() {
//         // This method will be called once per scheduler run
//         super.periodic();

//     }

//     @Override
//     public void simulationPeriodic() {
//         // This method will be called once per scheduler run when in simulation

//     }

//     @Override
//     public double getMeasurement() {
//         // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SOURCE
//         return  turretEncoder.getRate();

//     // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SOURCE
//     }

//     @Override
//     public void useOutput(double output, double setpoint) {
//         output += setpoint*kF;
//         // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=OUTPUT
// turretMotor.set(output);

//     // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=OUTPUT
//     }

//     // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

//     //!!PLACEHOLDER METHODS!!//
//         public double getAngle() {
//             return 0;
//           }

//         public Object rotateTurret(double output) {
//             return null;
//         }
//     // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

// }
