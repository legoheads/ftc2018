package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.DriveFunctions;

import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.intake.intakeThings;

import org.firstinspires.ftc.teamcode.subsystems.dunk.Dunk;
import org.firstinspires.ftc.teamcode.subsystems.dunk.servoArmDunk;

@TeleOp(name="Tele Op") //Name the class
public class teleMaster extends LinearOpMode
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
    float drivePower;
    float shiftPower;
    float leftTurnPower;
    float rightTurnPower;
    float spoolPower;

    int liftCount;
    int flipPos = -1;

    enum flipPositions {
        DOWN, UP, FLIP
    }

//    flipPositions flipPos;


    Intake intake;
    Dunk dunk;

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

        //Set up the DriveFunctions class and give it all the necessary components (motors, sensors)
        DriveFunctions functions = new DriveFunctions(leftMotorFront, rightMotorFront, leftMotorBack, rightMotorBack, mineralSpool, spinner, hanger, mineralFlipper, dunker, markerDropper);

        //Set the sensor to active mode
        //Set the directions and modes of the motors.
        functions.initializeMotorsAndSensors();

        intake = new intakeThings(mineralFlipper);
        dunk = new servoArmDunk(hanger, dunker);

        leftMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //Wait for start button to be clicked
        waitForStart();

        dunk.down();

//***************************************************************************************************************************
        while (opModeIsActive()) {
            drivePower = (float) ((gamepad1.left_stick_y + gamepad2.left_stick_y) * 0.85);
            shiftPower = (float) ((gamepad1.left_stick_x + gamepad2.left_stick_x) * 0.85);
            leftTurnPower = (float) ((gamepad1.left_trigger + gamepad2.left_trigger) * 0.75);
            rightTurnPower = (float) ((gamepad1.right_trigger + gamepad2.right_trigger) * 0.75);
            spoolPower = gamepad1.right_stick_y;

            telemetry.addData("flipPos", flipPos);
            telemetry.update();

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
            {
                functions.rightTurnTeleop(rightTurnPower);
            }

            //If the joysticks are not pushed significantly shut off the wheels
            if (Math.abs(drivePower) + Math.abs(shiftPower) + Math.abs(leftTurnPower) + Math.abs(rightTurnPower) < 0.15)
            {
                functions.setDriveMotorPowers((float) 0.0, (float) 0.0, (float) 0.0, (float) 0.0);
            }

            if (gamepad2.dpad_down)
            {
                functions.hang((float)1.0, 2925);

            }
            if (gamepad2.dpad_up)
            {
                functions.hang((float)-1.0, -2925);
            }


            if (gamepad1.right_bumper || gamepad2.right_bumper)
            {
                spinner.setPower(0.5);
            }

            if (gamepad1.left_bumper || gamepad2.left_bumper)
            {
                spinner.setPower(-0.5);
            }

            if (gamepad2.b)
            {
                spinner.setPower(0.0);
            }

//            if (gamepad2.y)
//            {
//
//                //Use the encoder
//                hanger.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//                //Reset the encoder
//                hanger.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//                //Use the encoder
//                hanger.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//                //Set up the motor to run to the given position
//                hanger.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//                //Set the target position as the value entered
//                hanger.setTargetPosition(-3100);
//
//                //Turn the motor on at the corresponding power
//                hanger.setPower(-1.0);
//
//                //Empty while loop while the motor is moving
//                while ((hanger.isBusy()))
//                {
//                    drivePower = (float) ((gamepad1.left_stick_y + gamepad2.left_stick_y) * 0.85);
//                    shiftPower = (float) ((gamepad1.left_stick_x + gamepad2.left_stick_x) * 0.85);
//                    leftTurnPower = (float) ((gamepad1.left_trigger + gamepad2.left_trigger) * 0.75);
//                    rightTurnPower = (float) ((gamepad1.right_trigger + gamepad2.right_trigger) * 0.75);
//
//
//
//                    //Drive if joystick pushed more Y than X on gamepad1 (fast)
//                    if (Math.abs(drivePower) > Math.abs(shiftPower))
//                    {
//                        functions.driveTeleop(drivePower);
//                    }
//
//                    //Shift if pushed more on X than Y on gamepad1 (fast)
//                    if (Math.abs(shiftPower) > Math.abs(drivePower))
//                    {
//                        functions.shiftTeleop(shiftPower);
//                    }
//
//                    //If the left trigger is pushed on gamepad1, turn left at that power (fast)
//                    if (leftTurnPower > 0)
//                    {
//                        functions.leftTurnTeleop(leftTurnPower);
//                    }
//
//                    //If the right trigger is pushed on gamepad1, turn right at that power (fast)
//                    if (rightTurnPower > 0)
//                    {
//                        functions.rightTurnTeleop(rightTurnPower);
//                    }
//
//                    //If the joysticks are not pushed significantly shut off the wheels
//                    if (Math.abs(drivePower) + Math.abs(shiftPower) + Math.abs(leftTurnPower) + Math.abs(rightTurnPower) < 0.15)
//                    {
//                        functions.setDriveMotorPowers((float) 0.0, (float) 0.0, (float) 0.0, (float) 0.0);
//                    }
//                }
//
//                //Stop the motor
//                hanger.setPower(0.0);
//
//                //Use the encoder in the future
//                hanger.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            }

            if (Math.abs(spoolPower) > 0.2)
            {
                mineralSpool.setPower(-spoolPower);
            }

            if (Math.abs(spoolPower) <=0.2)
            {
                mineralSpool.setPower(0.0);
            }
            if (gamepad1.dpad_left)
            {
                intake.flip();
            }

            //
            if (gamepad1.dpad_right)
            {
                intake.down();
            }

            //Intake Flip
//            if ((gamepad2.y || gamepad2.a) && flipPos<0){ flipPos=1;}
//            if (gamepad2.y && flipPos>=0){flipPos++; }
            if (gamepad2.a && flipPos>=0){ flipPos--; }
            if (gamepad2.a){ intake.down(); }
            if (gamepad2.x){ intake.up(); }
            if (gamepad2.y){ intake.flip(); }


//            if (gamepad2.dpad_left || gamepad2.dpad_right){
//                intake.flip();
//            }

            //Dunk
            if (gamepad1.dpad_up){
                dunk.dunk();
            }

            //Dunk return
            if (gamepad1.dpad_down){
                dunk.down();
            }
            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();
        }//Close while opModeIsActive loop
    } //Close "run Opmode" loop
} //Close class and end program
