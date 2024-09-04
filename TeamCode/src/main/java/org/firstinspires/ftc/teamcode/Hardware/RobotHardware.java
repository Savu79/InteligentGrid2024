package org.firstinspires.ftc.teamcode.Hardware;



import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
public class RobotHardware {

    public Servo MicroServo1;
    public Servo MicroServo2;
    public Servo AngleControlServo;

    public DcMotorEx ExtentionMotor;

    private HardwareMap hardwareMap;

    private static RobotHardware instance = null;

    public boolean enabled;

    //! VALORI CONSTANTE

    public static int ExtentionMAX=900;
    public static int ExtentionINT=600;
    public static int ExtentionMID=0;
    public static int ExtentionMIN=25;

    public static double ServoControlMAX=0.53; //0.55, 0.58
    public static double ServoControlMID=0.35; //0.2
    public static double ServoControlMIN=0.02; //0.315

    public static double AvionParcat=1;
    public static double AvionDecolare=0.75;

    public enum State {
        TRAJECTORY_1,
        TRAJECTORY_2,
        TRAJECTORY_3,
        TRAJECTORY_4// First, follow a splineTo() trajectory
    }


    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        instance.enabled = true;
        return instance;
    }
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        //TODO declaram motoare
        //MicroServo1= hardwareMap.get(Servo.class, "MicroServo1");//albastru
//        MicroServo2= hardwareMap.get(Servo.class, "MicroServo2");//negru
//        MicroServo1.setPosition(RobotHardware.MicroServoINCHIS1);
//        MicroServo2.setPosition(RobotHardware.MicroServoINCHIS2);


        ExtentionMotor= hardwareMap.get(DcMotorEx.class, "ExtensionMotor");
        ExtentionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        backCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
//        pipeline = new SleeveDetection.SkystoneDeterminationPipeline();
//        backCamera.setPipeline(pipeline);

    }

    public void loop() {

    }

    public void read() {
//        try {
//            intake.read();
//        } catch (Exception ignored) {
    }

    public void write() {
//            try {
//                intake.write();
//            } catch (Exception ignored){}
    }
}
