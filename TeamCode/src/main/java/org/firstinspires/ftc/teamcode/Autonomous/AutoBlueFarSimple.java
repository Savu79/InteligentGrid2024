package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OpenCVCode.OpenPipelineMedkit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class AutoBlueFarSimple extends LinearOpMode {
    //*1P de la un pixel
    private SampleMecanumDrive drive;
    private OpenPipelineMedkit.OpenIPosition pi;
    int extTarget=0;
    Pose2d myPose;
    boolean isClosed=false;
    boolean afost=false;

    DcMotorEx leftFront;
    DcMotorEx rightRear;
    DcMotorEx rightFront;
    DcMotorEx leftRear;

    OpenCvCamera backCamera;
    OpenPipelineMedkit pipeline;
    ElapsedTime waitTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


    public void runOpMode(){
        //drive = new SampleMecanumDrive(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        backCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        pipeline = new OpenPipelineMedkit();
        backCamera.setPipeline(pipeline);
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        backCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                backCamera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(backCamera, 10);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        //*Dupa declaratii ne asiguram ca robotul se afla in dimensiuni:

        //drive.setPoseEstimate(new Pose2d(-36, 61, Math.toRadians(270)));

        //* While pt ca pivMotor sa ajunga la pozitie + detectie
        while(opModeInInit()){
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.update();
            pi=pipeline.getAnalysis();
        }
        if(opModeIsActive()){
            //drive.update();
            leftFront.setPower(0.5);
            leftRear.setPower(0.5);
            rightFront.setPower(0.5);
            rightRear.setPower(0.5);
            sleep(2200);
            leftFront.setPower(0.5);
            leftRear.setPower(-0.5);
            rightFront.setPower(-0.5);
            rightRear.setPower(0.5);
            sleep(3300);
            leftFront.setPower(0);
            leftRear.setPower(0);
            rightFront.setPower(0);
            rightRear.setPower(0);

            }
            myPose=drive.getPoseEstimate();
            telemetry.addData("X", myPose.getX());
            telemetry.addData("Y", myPose.getY());
            telemetry.addData("Heading", myPose.getHeading());
            telemetry.update();
        }
    }


