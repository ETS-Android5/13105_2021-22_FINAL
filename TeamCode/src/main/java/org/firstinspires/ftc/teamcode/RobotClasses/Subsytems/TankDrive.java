package org.firstinspires.ftc.teamcode.RobotClasses.Subsytems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class TankDrive {
    static Standard_Bot robot = new Standard_Bot();
    Gyro gyro = new Gyro();

    private static DcMotorImplEx frontRight = robot.StdFrontRight;
    private static DcMotorImplEx frontLeft = robot.StdFrontLeft;
    private static DcMotorImplEx backRight = robot.StdBackRight;
    private static DcMotorImplEx backLeft = robot.StdBackLeft;
    DistanceSensor sensorRange;

    public void drive(double right, double left, double anglrt) {

        int rightTarget;
        int leftTarget;

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftTarget = (int) (left * (33) + frontLeft.getCurrentPosition());
        rightTarget = (int) (right * (33) + frontRight.getCurrentPosition());

        frontLeft.setTargetPosition(leftTarget);
        backLeft.setTargetPosition(leftTarget);
        frontRight.setTargetPosition(rightTarget);
        backRight.setTargetPosition(rightTarget);

        frontLeft.setVelocity(90, AngleUnit.DEGREES);
        backLeft.setVelocity(90, AngleUnit.DEGREES);
        frontRight.setVelocity(90, AngleUnit.DEGREES);
        backRight.setVelocity(90, AngleUnit.DEGREES);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setVelocity(anglrt, AngleUnit.DEGREES);
        backLeft.setVelocity(anglrt, AngleUnit.DEGREES);
        frontRight.setVelocity(anglrt, AngleUnit.DEGREES);
        backRight.setVelocity(anglrt, AngleUnit.DEGREES);

        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
            ;
        }

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setVelocity(0, AngleUnit.DEGREES);
        frontRight.setVelocity(0, AngleUnit.DEGREES);
        backLeft.setVelocity(0, AngleUnit.DEGREES);
        backRight.setVelocity(0, AngleUnit.DEGREES);

    }
    public void rotate(double degrees, double velocity) {
        double temp = rotate(degrees, velocity, 0);
        return;
    }

    private double rotate(double degrees, double velocity, int dummy) {
        double leftPower, rightPower;
        double currentAngle = 0, currentDistance = 0, minAngle = 0, minDistance = 100;

        // restart imu movement tracking.

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0) {   // turn right.
            leftPower = velocity;
            rightPower = -velocity;
        } else if (degrees > 0) {
            // turn left.
            leftPower = -velocity;
            rightPower = velocity;
        } else return 0;

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // set power to rotate.
        frontLeft.setVelocity(leftPower, AngleUnit.DEGREES);
        backLeft.setVelocity(leftPower, AngleUnit.DEGREES);
        frontRight.setVelocity(rightPower, AngleUnit.DEGREES);
        backRight.setVelocity(rightPower, AngleUnit.DEGREES);

        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (gyro.getAngle() == 0) {
            }

            while ((currentAngle = gyro.getAngle()) > degrees) {
                currentDistance = sensorRange.getDistance(DistanceUnit.INCH);
                if (currentDistance < minDistance) {
                    minDistance = currentDistance;
                    minAngle = currentAngle;
                }

            }
        } else {    // left turn.
            while (gyro.getAngle() == 0) {
            }
            while ((currentAngle = gyro.getAngle()) < degrees) {
            }
        }
        // turn the motors off.
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // reset angle tracking on new heading.
        return minAngle;
    }

}