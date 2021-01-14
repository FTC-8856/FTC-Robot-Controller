package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.tensorflow.lite.Interpreter;

import java.io.File;

@SuppressWarnings("unused")
@Autonomous(name = "Greg Brain")
        // Yeeeees we are getting the brain
class GregBrain extends OpMode {
    @NonNull
    private final
    ElapsedTime gametime = new ElapsedTime();
    @NonNull
    private final
    RobotHardware robot = new RobotHardware();
    private Interpreter brain;

    @NonNull
    private static Action decodeFloat(final float f) {
        final int f11 = (int) ((double) f / 11.0);
        Action action = Action.NONE;
        if (1 > f11) {
            action = Action.OPEN_CLAW;
        } else if (2 > f11) {
            action = Action.CLOSE_CLAW;
        } else if (3 > f11) {
            action = Action.RETRACT_ARM;
        } else if (4 > f11) {
            action = Action.EXTEND_ARM;
        } else if (5 > f11) {
            action = Action.START_INTAKE;
        } else if (6 > f11) {
            action = Action.STOP_INTAKE;
        } else if (7 > f11) {
            action = Action.REVERSE_INTAKE;
        } else if (8 > f11) {
            action = Action.FIRE_LOW;
        } else if (9 > f11) {
            action = Action.FIRE_MID;
        } else if (10 > f11) {
            action = Action.FIRE_HIGH;
        }
        return action;
    }

    @Override
    public final void init() {
        final File tfliteFile = new File("/FIRST/8856tensorflow/greg.tflite");
        this.brain = new Interpreter(tfliteFile);
        this.robot.init(this.hardwareMap, "imu");
    }

    @Override
    public final void start() {
        this.robot.startIMU(new Position(
                DistanceUnit.METER,
                0.0, 0.0, 0.0 // <--- starting position
                , 0L
        ));
        this.gametime.reset();
    }

    @Override
    public final void loop() {
        this.robot.hardwareLoop();
        final float[] inputLayer = {
                (float) this.gametime.seconds() / 30.0F // 30 second autonomous period
                , (float) this.robot.getPos().x
                , (float) this.robot.getPos().y
                , this.robot.getRot().thirdAngle // Heading
        };
        final float[] outputLayer = new float[4]; // Basic Joystick controls for now
        this.brain.run(inputLayer, outputLayer); // activate the  t h i n k i n g
        this.robot.chassis(new double[]{(double) outputLayer[0], (double) outputLayer[1], (double) outputLayer[2]});
        final Action action = GregBrain.decodeFloat(outputLayer[4]);
        this.robot.performAction(action);
    }

    @Override
    public final void stop() {
        this.brain.close();
        this.robot.hardwareStop();
    }


}
