package org.firstinspires.ftc.teamcode.Teste;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;


@TeleOp(group ="test")
public class TestPickUpServo extends LinearOpMode
{
    Servo PickUpServo; //servo pentru yellow blocks
    double pozitie = RobotHardware.PickUpServoMAX;
    //Gamepad gamepad1 = new Gamepad();

    public void runOpMode(){
        PickUpServo= hardwareMap.get(Servo.class, "PickUpServo");//albastru

        waitForStart();
        while(opModeIsActive()) {
            pozitie += 0.0005 * gamepad1.right_stick_y;

            pozitie= Range.clip(pozitie, 0, 1);

            PickUpServo.setPosition(pozitie);
            telemetry.addData("pozitie_pickup: ", PickUpServo.getPosition());
            telemetry.update();

        }
    }
}

