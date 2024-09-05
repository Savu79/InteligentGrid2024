package org.firstinspires.ftc.teamcode.Teste;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;


@TeleOp(group ="test")
public class TestCutieServo extends LinearOpMode
{
    Servo CutieServo; //servo pentru yellow blocks
    double pozitie = RobotHardware.CutieServoMIN;
    //Gamepad gamepad1 = new Gamepad();

    public void runOpMode(){
        CutieServo = hardwareMap.get(Servo.class, "CutieServo");//albastru

        waitForStart();
        while(opModeIsActive()) {
            pozitie += 0.0005 * gamepad1.right_stick_y;

            pozitie= Range.clip(pozitie, 0, 1);

            CutieServo.setPosition(pozitie);
            telemetry.addData("pozitie_pivot: ", CutieServo.getPosition());
            telemetry.update();

        }
    }
}

