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

    public Servo PickUpServo;
    public Servo AirlockServo;
    public Servo PivotServo;
    public Servo CutieServo;
    public Servo MicroServo;

    public DcMotorEx ExtensionMotor;
    public DcMotorEx IntakeMotor;

    private HardwareMap hardwareMap;

    private static RobotHardware instance = null;

    public boolean enabled;

    //! VALORI CONSTANTE

    public static int ExtentionMAX=1500;
    public static int ExtentionINT=600;
    public static int ExtentionMID=0;
    public static int ExtentionMIN=0;

    public static double MicroServoReleased = 0.69; //*done
    public static double MicroServoClosed = 0; //

    public static double AirlockServoMIN =1;
    public static double AirlockServoMAX =0;

    public static double PivotServoMINMIN =0.80; //*done
    public static double PivotServoMIN =0.75; //*done
    public static double PivotServoMAX=0;

    public static double CutieServoJOS =0.286;
    public static double CutieServoSUS=0.557;
    public static double CutieServoMIN=0;


    public static double PickUpServoMIN =0.37; //*done
    public static double PickUpServoMAX =1;

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
        AirlockServo = hardwareMap.get(Servo.class, "AirlockServo");//negru
        AirlockServo.setPosition(RobotHardware.AirlockServoMIN);

        PickUpServo= hardwareMap.get(Servo.class, "PickUpServo");//negru
        PickUpServo.setPosition(RobotHardware.PickUpServoMAX);

        PivotServo= hardwareMap.get(Servo.class, "PivotServo");//negru
        PivotServo.setPosition(RobotHardware.PivotServoMIN);

        CutieServo= hardwareMap.get(Servo.class, "CutieServo");//negru
        CutieServo.setPosition(RobotHardware.CutieServoJOS);

        MicroServo= hardwareMap.get(Servo.class, "MicroServo");//niggeR
        MicroServo.setPosition((RobotHardware.MicroServoReleased));

        ExtensionMotor= hardwareMap.get(DcMotorEx.class, "ExtensionMotor");
        ExtensionMotor.setTargetPosition(0);
        ExtensionMotor.setPower(1);
        ExtensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ExtensionMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        IntakeMotor= hardwareMap.get(DcMotorEx.class, "IntakeMotor");

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
