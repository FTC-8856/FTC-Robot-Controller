package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.android.AndroidOrientation;
import org.firstinspires.ftc.robotcore.external.android.AndroidTextToSpeech;
import java.lang.reflect.Array;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import java.util.Map;
import java.util.HashMap;

@TeleOp(name="Brave New World", group="Pushbot")

public class BraveNewWorld extends OpMode{

    /* Declare OpMode members. */
    //AndroidOrientation compass(); // Android Orientation Object Declaration
    //AndroidTextToSpeech speek(); // Text to Speech Object
    RobotHardware robot = new RobotHardware(); // use the class created to define a Pushbot's hardware
    //                      FR BR FL BL
    double[] leftvalue =    {1,-1,-1,1};// Motor Values to strafe Left
    double[] forwardvalue = {1,1,1,1}; // Motor Values to go Forwards
    double[] clockwisevalue={1,1,-1,-1}; // Motor Values to turn clockwise
    Map<DcMotor, Double[]> valueMap = new HashMap<>();
    double wobble_pos = 0;
    double finger_pos = 0;
    

    BNO055IMU imu;
    BNO055IMU.Parameters imuParameters;
        Orientation angles;
        Acceleration gravity;
        Acceleration accel;
        Position pos;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
    
        imu = hardwareMap.get(BNO055IMU.class, "imu");
    
        // Create new IMU Parameters object.
        imuParameters = new BNO055IMU.Parameters();
        imuParameters.mode = BNO055IMU.SensorMode.IMU;
        // Use degrees as angle unit.
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        // Express acceleration as m/s^2.
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // Disable logging.
        imuParameters.loggingEnabled = false;
        // Initialize IMU.
        imu.initialize(imuParameters);

        valueMap.insert(robot.frontright, new Double[]{1, 1, 1});
        valueMap.insert(robot.backright, new Double[]{-1, 1, 1});
        valueMap.insert(robot.frontleft, new Double[]{-1, 1, -1});
        valueMap.insert(robot.backleft, new Double[]{1, 1, -1});
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        telemetry.addData("Brave New World", "Groovy");
        telemetry.addData("Version", "0.20");
        telemetry.addData("Accelerometer", imu.isAccelerometerCalibrated());
        telemetry.addData("Gyro", imu.isGyroCalibrated());
        telemetry.addData("Magnetometer", imu.isMagnetometerCalibrated());
        telemetry.addData("Calib. Status", imu.getCalibrationStatus().toString());
        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }
    
    private void setPower(DcMotor motor) {
        Double[] values = valueMap.get(motor);
        double forward_backward = values[0] * gamepad1.right_stick_y; 
        double strafe = values[1] * gamepad1.right_stick_x;
        double turn = values[2] * gamepad1.left_stick_x;
        let power = forward_backward + strafe + turn;
        motor.setPower(Math.max(-1, Math.min(power, 1)));
    }
    
    @Override 
    public void loop() {         
        setPower(robot.frontright);
        setPower(robot.backright);
        setPower(robot.frontleft);
        setPower(robot.backleft);
        
        wobble_pos += 0.01 * gamepad2.left_stick_y;
        robot.wobble.setPosition(wobble_pos);
        finger_pos += 0.02 * gamepad2.left_stick_x;
        robot.wobbleFinger.setPosition(finger_pos);
        
        telemetry.addData("fwd/bkwd",  "%.2f", gamepad1.right_stick_y);
        telemetry.addData("strafe", "%.2f", gamepad1.right_stick_x);
        telemetry.addData("turn", "%.2f\n------------", gamepad1.left_stick_x);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Rot", "(%.2f, %.2f, %.2f)", angles.thirdAngle, angles.secondAngle, angles.firstAngle);
        telemetry.addData("RGBA", "(%.2f, %.2f, %.2f, %.2f)", robot.greg.red(), robot.greg.blue(), robot.greg.green(), robot.greg.alpha());
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        telemetry.addData("Exit","Goodest Good Job!");
        telemetry.update();
    }
}
