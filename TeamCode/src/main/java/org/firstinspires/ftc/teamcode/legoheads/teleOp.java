package org.firstinspires.ftc.teamcode.legoheads;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Tele Op") //Name the class
public class teleOp extends LinearOpMode
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
    Servo mineralDoor;
    Servo dunker;
    CRServo pin;
    Servo markerDropper;

    //Define drive powers to avoid magic numbers
    float drivePower;
    float shiftPower;
    float leftTurnPower;
    float rightTurnPower;
    float spoolPower;

    int pinCount = -1;
    int liftCount;
    int flipCount;

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
        mineralDoor = hardwareMap.servo.get("mineralDoor");
        dunker = hardwareMap.servo.get("dunker");
        pin = hardwareMap.crservo.get("pin");
        markerDropper = hardwareMap.servo.get("markerDropper");

        //Set up the DriveFunctions class and give it all the necessary components (motors, sensors)
        DriveFunctions functions = new DriveFunctions(leftMotorFront, rightMotorFront, leftMotorBack, rightMotorBack, mineralSpool, spinner, hanger, mineralFlipper, mineralDoor, dunker, pin, markerDropper);

        //Set the sensor to active mode
        //Set the directions and modes of the motors.
        functions.initializeMotorsAndSensors();

        leftMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //Wait for start button to be clicked
        waitForStart();

//***************************************************************************************************************************
        while (opModeIsActive())
        {
            drivePower = (float) ((gamepad1.left_stick_y + gamepad2.left_stick_y) * 0.85);
            shiftPower = (float) ((gamepad1.left_stick_x + gamepad2.left_stick_x) * 0.85);
            leftTurnPower = (float) ((gamepad1.left_trigger + gamepad2.left_trigger) * 0.75);
            rightTurnPower = (float) ((gamepad1.right_trigger + gamepad2.right_trigger) * 0.75);
            spoolPower = -gamepad1.right_stick_y / 2;

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

            if (gamepad1.a)
            {
                functions.hang((float)1.0, 2925);

            }
            if (gamepad1.y)
            {
                functions.hang((float)-1.0, -2925);
            }


            if(gamepad1.b)
            {
                pin.setPower(0.75);
                Thread.sleep(7200);
                pin.setPower(0.0);
            }

            if(gamepad1.x)
            {
                pin.setPower(-0.75);
                Thread.sleep(7200);
                pin.setPower(0.0);
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

            if (gamepad2.y)
            {
                //Use the encoder
                hanger.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                //Reset the encoder
                hanger.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                //Use the encoder
                hanger.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                //Set up the motor to run to the given position
                hanger.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //Set the target position as the value entered
                hanger.setTargetPosition(-3100);

                //Turn the motor on at the corresponding power
                hanger.setPower(-1.0);

                //Empty while loop while the motor is moving
                while ((hanger.isBusy()))
                {
                    drivePower = (float) ((gamepad1.left_stick_y + gamepad2.left_stick_y) * 0.85);
                    shiftPower = (float) ((gamepad1.left_stick_x + gamepad2.left_stick_x) * 0.85);
                    leftTurnPower = (float) ((gamepad1.left_trigger + gamepad2.left_trigger) * 0.75);
                    rightTurnPower = (float) ((gamepad1.right_trigger + gamepad2.right_trigger) * 0.75);



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
                }

                //Stop the motor
                hanger.setPower(0.0);

                //Use the encoder in the future
                hanger.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            if (Math.abs(spoolPower) > 0.2)
            {
                mineralSpool.setPower(spoolPower);
            }

            if (Math.abs(spoolPower) <=0.2)
            {
                mineralSpool.setPower(0.0);
            }
            if (gamepad1.dpad_left)
            {
                spinner.setPower(-0.5);
                mineralFlipper.setPosition(mineralFlipper.getPosition() + 0.3);
            }

            if (gamepad1.dpad_right)
            {
                spinner.setPower(0.5);
                mineralFlipper.setPosition(mineralFlipper.getPosition() - 0.3);
            }

            if (gamepad1.dpad_up)
            {
                functions.driveAutonomous((float) 0.4, 80);
                //Use the encoder
                hanger.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                //Reset the encoder
                hanger.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                //Use the encoder
                hanger.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                //Set up the motor to run to the given position
                hanger.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //Set the target position as the value entered
                hanger.setTargetPosition(-2600);

                //Turn the motor on at the corresponding power
                hanger.setPower(-1.0);

                //Empty while loop while the motor is moving
                while ((hanger.isBusy()))
                {
                    drivePower = (float) ((gamepad1.left_stick_y + gamepad2.left_stick_y) * 0.85);
                    shiftPower = (float) ((gamepad1.left_stick_x + gamepad2.left_stick_x) * 0.85);
                    leftTurnPower = (float) ((gamepad1.left_trigger + gamepad2.left_trigger) * 0.75);
                    rightTurnPower = (float) ((gamepad1.right_trigger + gamepad2.right_trigger) * 0.75);



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

                    dunker.setPosition(0.2);
                }

                //Stop the motor
                hanger.setPower(0.0);

                //Use the encoder in the future
                hanger.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                dunker.setPosition(0.2);
            }

            if (gamepad1.dpad_down)
            {
                //Use the encoder
                hanger.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                //Reset the encoder
                hanger.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                //Use the encoder
                hanger.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                //Set up the motor to run to the given position
                hanger.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //Set the target position as the value entered
                hanger.setTargetPosition(2600);

                //Turn the motor on at the corresponding power
                hanger.setPower(1.0);

                //Empty while loop while the motor is moving
                while ((hanger.isBusy()))
                {
                    drivePower = (float) ((gamepad1.left_stick_y + gamepad2.left_stick_y) * 0.85);
                    shiftPower = (float) ((gamepad1.left_stick_x + gamepad2.left_stick_x) * 0.85);
                    leftTurnPower = (float) ((gamepad1.left_trigger + gamepad2.left_trigger) * 0.75);
                    rightTurnPower = (float) ((gamepad1.right_trigger + gamepad2.right_trigger) * 0.75);



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

                    dunker.setPosition(0.85);
                }

                //Stop the motor
                hanger.setPower(0.0);

                //Use the encoder in the future
                hanger.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                dunker.setPosition(0.85);
            }
            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();
        }//Close while opModeIsActive loop
    } //Close "run Opmode" loop
} //Close class and end program
