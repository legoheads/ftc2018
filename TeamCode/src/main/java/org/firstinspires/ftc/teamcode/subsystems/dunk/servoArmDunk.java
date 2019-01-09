package org.firstinspires.ftc.teamcode.subsystems.dunk;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.subsystems.hang.linearActuator;


public class servoArmDunk implements Dunk {

    DcMotor hanger;
    Servo dunker;

    //DC Hanger Variables
    final int DUNK_DISTANCE = 10000;
    final double DUNK_POWER = 1.0;

    //Dunk Servo Variables
    final double DUNK_POSITION = 0.23;
    final double DOWN_POSTITION = 0.85;
    final double WIGGLE_POSITION1 = 0.23;
    final double WIGGLE_POSITION2 = 0.2;

    public servoArmDunk(DcMotor verticalMotion, Servo dunkArm)
    {
        hanger = verticalMotion;
        dunker = dunkArm;
    }

    //Dunk
    @Override
    public void dunkWithPause() throws InterruptedException
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
        hanger.setTargetPosition(-DUNK_DISTANCE);

        //Turn the motor on at the corresponding power
        hanger.setPower(-DUNK_POWER);

        //Empty while loop while the motor is moving
        while ((hanger.isBusy())) { }

        //Stop the motor
        hanger.setPower(0.0);

        //Use the encoder in the future
        hanger.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hanger.setPower(-DUNK_POWER);
        Thread.sleep(500);
        hanger.setPower(0.0);

        dunker.setPosition(DUNK_POSITION+0.22);

        Thread.sleep(1300);

        dunker.setPosition(DUNK_POSITION);
    }

    //Dunk return
    @Override
    public void down() throws InterruptedException
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
        hanger.setTargetPosition(DUNK_DISTANCE);

        //Turn the motor on at the corresponding power
        hanger.setPower(DUNK_POWER);

        //Empty while loop while the motor is moving
        while ((hanger.isBusy())){
            dunker.setPosition(DOWN_POSTITION);
        }

        //Stop the motor
        hanger.setPower(0.0);

        //Use the encoder in the future
        hanger.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hanger.setPower(DUNK_POWER);
        Thread.sleep(2500);
        hanger.setPower(0.0);

        dunker.setPosition(DOWN_POSTITION);
    }

    //Dunk
    @Override
    public void dunkNoPause() throws InterruptedException {


        //Use the encoder
        hanger.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Reset the encoder
        hanger.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Use the encoder
        hanger.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Set up the motor to run to the given position
        hanger.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Set the target position as the value entered
        hanger.setTargetPosition(-DUNK_DISTANCE);

        //Turn the motor on at the corresponding power
        hanger.setPower(-DUNK_POWER);

        //Empty while loop while the motor is moving
        while ((hanger.isBusy())) {
//            dunker.setPosition(DUNK_POSITION+0.25);
//            Thread.sleep(1000);
//            dunker.setPosition(DUNK_POSITION);
        }

        //Stop the motor
        hanger.setPower(0.0);

        //Use the encoder in the future
        hanger.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hanger.setPower(-DUNK_POWER);
        Thread.sleep(2500);
        hanger.setPower(0.0);

//        dunker.setPosition(DUNK_POSITION+0.22);

//        Thread.sleep(1500);

        dunker.setPosition(DUNK_POSITION);
    }

    @Override
    public void wiggle()
    {
        for (int n = 0; n <= 3; n++)
        {
            dunker.setPosition(WIGGLE_POSITION1);
            dunker.setPosition(WIGGLE_POSITION2);
        }
    }
}




