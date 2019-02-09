package org.firstinspires.ftc.teamcode.test_programs;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.subsystems.DriveFunctions;

import static org.firstinspires.ftc.teamcode.subsystems.DriveFunctions.oneMotorEncoder;

@Disabled
@TeleOp(name="wires test ") //Name the class
public class wiresTest extends LinearOpMode
{
    //Define drive motors
    DcMotor leftMotorFront;
    DcMotor rightMotorFront;
    DcMotor leftMotorBack;
    DcMotor rightMotorBack;

    //Define glyph motors
    DcMotor mineralSpool;
    DcMotor spinner;
    DcMotor hanger;

    Servo mineralFlipper;
    Servo dunker;
    Servo markerDropper;

    //Define drive powers to avoid magic numbers
    float power = (float) 0.5;

//***************************************************************************************************************************
    //MAIN BELOW
    @Override
    public void runOpMode() throws InterruptedException
    {
        leftMotorFront = hardwareMap.dcMotor.get("leftMotorFront");
        rightMotorFront = hardwareMap.dcMotor.get("rightMotorFront");
        leftMotorBack = hardwareMap.dcMotor.get("leftMotorBack");
        rightMotorBack = hardwareMap.dcMotor.get("rightMotorBack");

        mineralSpool = hardwareMap.dcMotor.get("mineralSpool");
        spinner = hardwareMap.dcMotor.get("spinner");
        hanger = hardwareMap.dcMotor.get("hanger");

        mineralFlipper = hardwareMap.servo.get("mineralFlipper");
        dunker = hardwareMap.servo.get("dunker");
        markerDropper = hardwareMap.servo.get("markerDropper");

        //Wait for start button to be clicked
        waitForStart();

//***************************************************************************************************************************
        while (opModeIsActive())
        {
            if (gamepad1.y)
            {
                oneMotorEncoder(leftMotorFront, power, 300);
                telemetry.addData("motor forwards", leftMotorFront);
            }
            if (gamepad1.b)
            {
                oneMotorEncoder(rightMotorFront, power, 300);
                telemetry.addData("motor forwards", rightMotorFront);
            }
            if (gamepad1.a)
            {
                oneMotorEncoder(leftMotorBack, power, 300);
                telemetry.addData("motor forwards", leftMotorBack);
            }
            if (gamepad1.x)
            {
                oneMotorEncoder(rightMotorBack, power, 300);
                telemetry.addData("motor forwards", rightMotorBack);
            }

            if (gamepad1.dpad_up)
            {
                oneMotorEncoder(leftMotorFront, -power, -300);
                telemetry.addData("motor backwards", leftMotorFront);
            }
            if (gamepad1.dpad_right)
            {
                oneMotorEncoder(rightMotorFront, -power, -300);
                telemetry.addData("motor backwards", rightMotorFront);
            }
            if (gamepad1.dpad_down)
            {
                oneMotorEncoder(leftMotorBack, -power, -300);
                telemetry.addData("motor backwards", leftMotorBack);
            }
            if (gamepad1.dpad_left)
            {
                oneMotorEncoder(rightMotorBack, -power, -300);
                telemetry.addData("motor backwards", rightMotorBack);
            }

            telemetry.update();

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();
        }//Close while opModeIsActive loop
    } //Close "run Opmode" loop
} //Close class and end program
