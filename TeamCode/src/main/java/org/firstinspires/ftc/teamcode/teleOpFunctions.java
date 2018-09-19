package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

public class teleOpFunctions
{
    //Define drive motors
    DcMotor leftMotorFront;
    DcMotor rightMotorFront;
    DcMotor leftMotorBack;
    DcMotor rightMotorBack;

    //Define the color sensor
    ColorSensor colorSensor;
    public teleOpFunctions(DcMotor leftMotorFront, DcMotor rightMotorFront, DcMotor leftMotorBack, DcMotor rightMotorBack)
    {
            //These lines enable us to store the motors, sensors and CDI without having to write them over and over again
            //Initialize DC and Servo motors
            this.leftMotorFront = leftMotorFront;
            this.leftMotorBack = leftMotorBack;
            this.rightMotorFront = rightMotorFront;
            this.rightMotorBack = rightMotorBack;
    }

    commonFunctions commonFunctions = new commonFunctions(leftMotorFront, rightMotorFront, leftMotorBack, rightMotorBack);

    /**
     * If this function is called, turn on the drive motors at the given powers to make it drive forward or backwards
     */
    public void straightDrive(float power)
    {
        //Send all the motors in the same direction
        commonFunctions.setDriveMotorPowers(power, power, power, power);
    }

    /**
     * If this function is called, turn on the drive motors at the given powers, to make it tank turn left
     */
    public void leftTurn(float power)
    {
        power = -power;
        //Turn the left motors backwards and the right motors forward so that it turns left
        commonFunctions.setDriveMotorPowers(-power, -power, power, power);
    }

    /**
     * If this function is called, turn on the drive motors at the given powers, to make it tank turn right
     */
    public void rightTurn(float power)
    {
        //Turn the right motors backwards and the left motors forward so that it turns right
        commonFunctions.setDriveMotorPowers(power, power, -power, -power);
    }

    /**
     * If this function is called, turn on the drive motors at the
     * given powers, to make it shift in the desired direction
     */
    public void shift(float power)
    {
        //This sequence of backwards, forwards, forwards, backwards makes the robot shift
        commonFunctions.setDriveMotorPowers(-power, power, power, -power);
    }

}
