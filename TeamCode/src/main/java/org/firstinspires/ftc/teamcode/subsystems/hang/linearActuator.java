package org.firstinspires.ftc.teamcode.subsystems.hang;

import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.DriveFunctions.oneMotorEncoder;

public class linearActuator implements Hang{

    private final float HANG_POWER = (float) 1.0;
    private final int HANG_DISTANCE = 10000;
    private final int DUNK_DISTANCE = 5385;

    private DcMotor hanger;

    public linearActuator(DcMotor linearActuator)
    {
        hanger = linearActuator;
    }

    public void up() throws InterruptedException
    {
        oneMotorEncoder(hanger, HANG_POWER, HANG_DISTANCE);
        hanger.setPower(HANG_POWER);
        Thread.sleep(2500);
        hanger.setPower(0.0);
    }

    public void down() throws InterruptedException
    {
        oneMotorEncoder(hanger, -HANG_POWER, -HANG_DISTANCE);
        hanger.setPower(-HANG_POWER);
        Thread.sleep(2500);
        hanger.setPower(0.0);
    }

    public void setDunk() throws InterruptedException {
        oneMotorEncoder(hanger, HANG_POWER, DUNK_DISTANCE);
        hanger.setPower(0.0);
    }
}
