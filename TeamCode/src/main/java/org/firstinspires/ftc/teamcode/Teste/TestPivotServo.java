package org.firstinspires.ftc.teamcode.Teste;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;


@TeleOp(group ="test")
public class TestPivotServo extends LinearOpMode
{
    Servo PivotServo; //servo pentru yellow blocks
    double pozitie = RobotHardware.PivotServoMAX;
    //Gamepad gamepad1 = new Gamepad();

    public void runOpMode(){
        PivotServo = hardwareMap.get(Servo.class, "PivotServo");//albastru

        waitForStart();
        while(opModeIsActive()) {
            pozitie += 0.0005 * gamepad1.right_stick_y;
            if(gamepad1.a)
            {
                pozitie=RobotHardware.PivotServoMIN;
            }

            if(gamepad1.y)
            {
                pozitie=RobotHardware.PivotServoMAX;
            }
            pozitie= Range.clip(pozitie, 0, 1);

            PivotServo.setPosition(pozitie);
            telemetry.addData("pozitie_pivot: ", PivotServo.getPosition());
            telemetry.update();

        }
    }
}

