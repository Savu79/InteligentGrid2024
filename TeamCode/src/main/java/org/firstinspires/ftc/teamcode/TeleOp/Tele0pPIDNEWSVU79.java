package org.firstinspires.ftc.teamcode.TeleOp;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Subsystems.ExtentionSubsystem;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@TeleOp(group ="tele0p")
public class Tele0pPIDNEWSVU79 extends LinearOpMode {
    private RobotHardware robot= RobotHardware.getInstance();
    private SampleMecanumDrive drive;
    private ExtentionSubsystem extMotor;
    int extTarget=0;
    double vit =1;
    boolean isPickedUp = false;
    boolean intakeON = false;


    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap,telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        extMotor= new ExtentionSubsystem(robot);
        extMotor.setExtentionTarget(0);
        waitForStart();

        while (opModeIsActive()) {

            //* MICRO SERVO
            if (gamepad2.right_bumper)
            {
                robot.MicroServo.setPosition(RobotHardware.MicroServoClosed);
            }

            if(gamepad2.left_bumper)
            {
                robot.MicroServo.setPosition(RobotHardware.MicroServoReleased);
            }

            //* AIRLOCK SERVO
            if(gamepad1.a)
            {
                robot.AirlockServo.setPosition(RobotHardware.AirlockServoMAX);
            }
            else{
                robot.AirlockServo.setPosition(RobotHardware.AirlockServoMIN);
            }

            //* PICKUP SERVO
            if(gamepad1.b)
            {
                robot.PickUpServo.setPosition(RobotHardware.PickUpServoMAX);
            }

            if(gamepad1.y)
            {
                robot.PickUpServo.setPosition(RobotHardware.PickUpServoMIN);
            }

            //* INTAKE
            if(gamepad2.dpad_up)
            {
                robot.IntakeMotor.setPower(-1);
            }
            if(gamepad2.dpad_right)
            {
                robot.IntakeMotor.setPower(0);
            }
            if(gamepad2.dpad_down)
            {
                robot.IntakeMotor.setPower(1);
            }

            //*VITEZA
            if(gamepad1.dpad_up)
            {
                vit=1;
            }
            if(gamepad1.dpad_right)
            {
                vit=0.75;
            }
            if(gamepad1.dpad_down)
            {
                vit=0.5;
            }
            if(gamepad1.dpad_left)
            {
                vit=0.25;
            }

            //* COMANDA DE STOP
            if (isStopRequested())
            {
                //extMotor.setExtentionTarget(RobotHardware.ExtentionMIN);
                robot.ExtensionMotor.setTargetPosition(RobotHardware.ExtentionMIN);
                sleep(500);
                robot.PivotServo.setPosition(RobotHardware.PivotServoMIN);
                robot.CutieServo.setPosition(RobotHardware.CutieServoJOS);
            }

            //* MUTARE EXTENTION
            //extMotor.update();
            if(gamepad2.right_stick_y!=0){
                extTarget+=(int)(-gamepad2.right_stick_y*20);
                extTarget= Range.clip(extTarget, RobotHardware.ExtentionMIN, RobotHardware.ExtentionMAX);
            }
            //if(gamepad2.right_bumper) extTarget= RobotHardware.ExtentionMIN;
            //if(gamepad2.left_bumper) extTarget= RobotHardware.ExtentionMAX;

            robot.ExtensionMotor.setTargetPosition(extTarget);
            //extMotor.setExtentionTarget(extTarget);

            //* MUTARE AUTOMATA
            if(robot.ExtensionMotor.getCurrentPosition() > 1000)
            {
                robot.PivotServo.setPosition(RobotHardware.PivotServoMAX);
                robot.CutieServo.setPosition(RobotHardware.CutieServoSUS);
            }
            else if(robot.ExtensionMotor.getCurrentPosition() > 150)
            {
                robot.PivotServo.setPosition(RobotHardware.PivotServoMIN);
                robot.CutieServo.setPosition(RobotHardware.CutieServoJOS);
            }

            if(robot.ExtensionMotor.getCurrentPosition() < 150)
            {
                robot.PivotServo.setPosition(RobotHardware.PivotServoMINMIN);
            }

            //*DRIVE
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * vit,
                            -gamepad1.left_stick_x * vit,
                            gamepad1.right_stick_x * vit));

            drive.updatePoseEstimate();

            telemetry.addData("x", drive.getPoseEstimate().getX());
            telemetry.addData("y", drive.getPoseEstimate().getY());
            telemetry.addData("heading (deg)", Math.toDegrees(drive.getPoseEstimate().getHeading()));
            telemetry.addData("ExtentionMotor", robot.ExtensionMotor.getCurrentPosition());
            telemetry.addData("extTarget", extTarget);
            telemetry.update();
        }
    }
}




