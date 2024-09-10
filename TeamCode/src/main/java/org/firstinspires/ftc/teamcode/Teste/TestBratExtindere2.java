package org.firstinspires.ftc.teamcode.Teste;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(group ="test")
public class TestBratExtindere2 extends LinearOpMode {

    DcMotor extindatoarea;

    int x;
    //Gamepad gamepad1=new Gamepad();
    @Override
    public void runOpMode() {


        extindatoarea = hardwareMap.get(DcMotor.class, "ExtensionMotor");
        extindatoarea.setTargetPosition(0);
        extindatoarea.setPower(1);
        extindatoarea.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extindatoarea.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();
        while(opModeIsActive()) {
            if(gamepad1.right_stick_button) extindatoarea.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            telemetry.addData("pozitie: ",extindatoarea.getCurrentPosition());
            telemetry.update();
            x=x+10*(int)gamepad1.right_stick_y;
            extindatoarea.setTargetPosition(x);
        }
    }
}
