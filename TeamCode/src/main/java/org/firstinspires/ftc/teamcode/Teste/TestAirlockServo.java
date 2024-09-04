package org.firstinspires.ftc.teamcode.Teste;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;


@TeleOp(group ="test")
public class TestAirlockServo extends LinearOpMode
{
    Servo AirlockServo; //servo pentru yellow blocks
    double pozitie = RobotHardware.AirlockServoMIN;
    //Gamepad gamepad1 = new Gamepad();

    public void runOpMode(){
        AirlockServo= hardwareMap.get(Servo.class, "AirlockServo");//albastru

        waitForStart();
        while(opModeIsActive()) {
            pozitie += 0.0005 * gamepad1.right_stick_y;

            pozitie= Range.clip(pozitie, -1, 1);

            AirlockServo.setPosition(pozitie);
            telemetry.addData("pozitie_airlock: ", AirlockServo.getPosition());
            telemetry.update();

        }
    }
}

