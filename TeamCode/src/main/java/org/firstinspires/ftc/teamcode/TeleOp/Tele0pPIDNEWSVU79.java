package org.firstinspires.ftc.teamcode.TeleOp;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
    boolean isPickedUp = false;


    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap,telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        extMotor= new ExtentionSubsystem(robot);
        extMotor.setExtentionTarget(0);
        waitForStart();

        while (opModeIsActive()) {


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
                if (!isPickedUp) {;
                    robot.PickUpServo.setPosition(RobotHardware.PickUpServoMIN);
                    isPickedUp = true;
                }
                else {
                    robot.PickUpServo.setPosition(RobotHardware.PickUpServoMAX);
                    isPickedUp = false;
                }
            }

            //* COMANDA DE STOP
            if (isStopRequested())
            {
                extMotor.setExtentionTarget(RobotHardware.ExtentionMIN);
            }

            //* MUTARE EXTENTION
            extMotor.update();
            if(gamepad2.right_stick_y!=0){
                extTarget+=(int)(-gamepad2.right_stick_y*40);
                extTarget= Range.clip(extTarget, RobotHardware.ExtentionMIN, RobotHardware.ExtentionMAX);
            }
            if(gamepad2.right_bumper) extTarget= RobotHardware.ExtentionMIN;
            if(gamepad2.left_bumper) extTarget= RobotHardware.ExtentionMAX;
            extMotor.setExtentionTarget(extTarget);

            //*DRIVE
            drive.setWeightedDrivePower(
                    new Pose2d(
                            Math.pow(-gamepad1.left_stick_y, 3),
                            Math.pow(-gamepad1.left_stick_x, 3),
                            Math.pow(-gamepad1.right_stick_x, 3)));

            drive.updatePoseEstimate();

            telemetry.addData("x", drive.getPoseEstimate().getX());
            telemetry.addData("y", drive.getPoseEstimate().getY());
            telemetry.addData("heading (deg)", Math.toDegrees(drive.getPoseEstimate().getHeading()));
            telemetry.addData("ExtentionMotor", robot.ExtentionMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}




