package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.DriveFunctions;

import org.firstinspires.ftc.teamcode.subsystems.intake.*;
import org.firstinspires.ftc.teamcode.subsystems.mineral_flip.*;
import org.firstinspires.ftc.teamcode.subsystems.dunk.*;
import org.firstinspires.ftc.teamcode.subsystems.hang.*;

@TeleOp(name="Tele Op") //Name the class
public class teleMaster extends LinearOpMode {
    //Define drive motors
    DcMotor leftMotorFront;
    DcMotor rightMotorFront;
    DcMotor leftMotorBack;
    DcMotor rightMotorBack;

    //Define  motors
    Servo dunker;
    DcMotor spinner;
    Servo mineralFlipper;
    DcMotor mineralSpool;

    DcMotor hanger;
    Servo markerDropper;

    enum flipPositions { DOWN, UP }
    flipPositions currFlipPos;

    //Subsystems
    Flip flip;
    Dunk dunk;
    Intake intake;
    Hang hang;

    int upDownToggle = 0;
    //***************************************************************************************************************************
    //MAIN BELOW
    @Override
    public void runOpMode() throws InterruptedException {
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

        //Set up the DriveFunctions class and give it all the necessary components (motors, sensors)
        DriveFunctions functions = new DriveFunctions(leftMotorFront, rightMotorFront, leftMotorBack, rightMotorBack, hanger);

        //Set the sensor to active mode
        //Set the directions and modes of the motors.
        functions.initializeMotorsAndSensors();

        //Intialize Subsystems
        flip = new mineralFlip(mineralFlipper);
        dunk = new servoArmDunk(hanger, dunker);
        intake = new intakeMinerals(spinner, mineralSpool);
        hang = new linearActuator(hanger);

        dunker.setPosition(0.85);
        mineralSpool.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mineralSpool.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        currFlipPos = flipPositions.UP;
        flip.up();

        //Wait for start button to be clicked
        waitForStart();

//***************************************************************************************************************************
        while (opModeIsActive()) {
            float drivePower = (float) ((gamepad1.left_stick_y + gamepad2.left_stick_y) * 0.5);
            float shiftPower = (float) ((gamepad1.left_stick_x + gamepad2.left_stick_x) * 0.5);
            float leftTurnPower = (float) ((gamepad1.left_trigger + gamepad2.left_trigger) * 0.3);
            float rightTurnPower = (float) ((gamepad1.right_trigger + gamepad2.right_trigger) * 0.3);
            float spoolPower = gamepad1.right_stick_y;

            //Drive if joystick pushed more Y than X on gamepad1 (fast)
            if (Math.abs(drivePower) > Math.abs(shiftPower))
            {
                functions.driveTeleop(drivePower);
            }

            //Shift if pushed more on X than Y on gamepad1 (fast)
            if (Math.abs(shiftPower) > Math.abs(drivePower))
            {
                functions.shiftTeleop(shiftPower);
            }

            //If the left trigger is pushed on gamepad1, turn left at that power (fast)
            if (leftTurnPower > 0)
            {
                functions.leftTurnTeleop(leftTurnPower);
            }

            //If the right trigger is pushed on gamepad1, turn right at that power (fast)
            if (rightTurnPower > 0)
                functions.rightTurnTeleop(rightTurnPower);

            //If the joysticks are not pushed significantly shut off the wheels
            if (Math.abs(drivePower) + Math.abs(shiftPower) + Math.abs(leftTurnPower) + Math.abs(rightTurnPower) < 0.15)
                functions.setDriveMotorPowers((float) 0.0, (float) 0.0, (float) 0.0, (float) 0.0);

            if (gamepad1.dpad_down) { hang.down(); }

            if (gamepad1.dpad_up) { hang.up(); }

            if (gamepad1.right_bumper || gamepad2.right_bumper) {
                intake.reverse();
            }

            if (gamepad1.left_bumper || gamepad2.left_bumper)
                intake.start();

            if (Math.abs(spoolPower) > 0.1)
                mineralSpool.setPower(-spoolPower);

            if (Math.abs(spoolPower) <=0.1)
                mineralSpool.setPower(0.0);

            if (gamepad1.b)
            {
                intake.stop();
            }

            if (gamepad1.x) {
                mineralSpool.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                mineralSpool.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            if (gamepad2.a) {
                if (currFlipPos == flipPositions.DOWN) {
                    flip.up();
                    functions.oneMotorEncoder(mineralSpool, (float) -0.5, -1000);
                    currFlipPos = flipPositions.UP;
                }
                else if (currFlipPos == flipPositions.UP) {
                    functions.oneMotorEncoder(mineralSpool, (float) 0.5, 1000);
                    flip.down();
                    currFlipPos=flipPositions.DOWN;
                }
            }

            if (gamepad2.y) {
                intake.stop();
                functions.oneMotorEncoder(mineralSpool, (float) -0.5, -Math.abs(mineralSpool.getCurrentPosition()));
                flip.flip();
                Thread.sleep(500);
                functions.oneMotorEncoder(mineralSpool, (float) 0.5, 300);
                flip.up();
            }

            if (gamepad2.b)
            {
                upDownToggle++;
                if (upDownToggle % 2 == 1)
                {
                    flip.up();
                }
                if (upDownToggle % 2 == 0)
                {
                    flip.down();
                }
            }

            if (gamepad2.x)
            {
                flip.flip();
                Thread.sleep(600);
                flip.up();
            }

            //Dunk
            if (gamepad2.dpad_up) {
                dunk.dunk();

            }

            if (gamepad2.dpad_down)
            {
                dunk.down();
            }


            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();
        }//Close while opModeIsActive loop
    } //Close "run Opmode" loop
} //Close class and end program
