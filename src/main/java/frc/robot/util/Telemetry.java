// package frc.robot.util;

// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.util.sendable.Sendable;
// import edu.wpi.first.util.sendable.SendableBuilder;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.RobotMap;
// import frc.robot.subsystems.Drivetrain;

// public final class Telemetry {

//     NetworkTableInstance inst = NetworkTableInstance.getDefault();
//     NetworkTable datatable = inst.getTable("SmartDashboard");

//     public Telemetry() {
//         inst.startDSClient();
//     }
    
//     private static Telemetry instance; 


//     public void DrivetrainSendableBuilder(SendableBuilder builder){
//         builder.addDoubleProperty("Translation kP", D, null);
//     }

//     // public static Telemetry getInstance() {
//     //     if (instance == null) {
//     //         instance = new Telemetry();
//     //     }
//     //     return instance;
//     // }
// }