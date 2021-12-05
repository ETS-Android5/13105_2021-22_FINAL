package org.firstinspires.ftc.teamcode.RobotClasses.Subsytems;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * A wrapper class for the BNO055IMU
 */
public class Gyro {
    private BNO055IMU imu;
    private BNO055IMU.Parameters parameters;

    private double resetAngle;
    private double minusAngle;

    Orientation angle;

    public Gyro(BNO055IMU imu) {
        this.imu = imu;

        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    public double getAngle() {
        imu.getPosition();
        angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angle.firstAngle;
    }

    public double resetGyro(){
        resetAngle = getAngle();
        minusAngle = resetAngle*0.9;
        resetAngle = resetAngle - minusAngle;
        return resetAngle;
    }

}