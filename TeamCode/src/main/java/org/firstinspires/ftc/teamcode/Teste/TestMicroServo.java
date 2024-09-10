package org.firstinspires.ftc.teamcode.Teste;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;


@TeleOp(group ="test")
public class TestMicroServo extends LinearOpMode
{
    Servo MicroServo; //servo pentru yellow blocks
    double pozitie = RobotHardware.MicroServoClosed;
    //Gamepad gamepad1 = new Gamepad();

    public void runOpMode(){
        MicroServo = hardwareMap.get(Servo.class, "MicroServo");//albastru

        waitForStart();
        while(opModeIsActive()) {
            pozitie += 0.0005 * gamepad1.right_stick_y;

            pozitie= Range.clip(pozitie, 0, 1);

            MicroServo.setPosition(pozitie);
            telemetry.addData("pozitie_pivot: ", MicroServo.getPosition());
            telemetry.update();

        }
    }
}

