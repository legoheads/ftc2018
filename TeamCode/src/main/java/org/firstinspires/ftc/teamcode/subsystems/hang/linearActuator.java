package org.firstinspires.ftc.teamcode.subsystems.hang;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.DriveFunctions;

public class linearActuator implements Hang{

    private final float HANG_POWER = (float) 0.7;
    private final int HANG_DISTANCE = 10000;

    private DcMotor hanger;

    public linearActuator(DcMotor linearActuator)
    {
        hanger = linearActuator;
    }

    public void up() throws InterruptedException
    {
        DriveFunctions.oneMotorEncoder(hanger, HANG_POWER, HANG_DISTANCE);
        hanger.setPower(HANG_POWER);
        Thread.sleep(2500);
        hanger.setPower(0.0);
    }

    public void down() throws InterruptedException
    {
        DriveFunctions.oneMotorEncoder(hanger, -HANG_POWER, -HANG_DISTANCE);
        hanger.setPower(-HANG_POWER);
        Thread.sleep(2500);
        hanger.setPower(0.0);
    }
}
