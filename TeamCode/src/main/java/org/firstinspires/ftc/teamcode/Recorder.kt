package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import java.io.FileOutputStream
import java.io.ObjectOutputStream

@TeleOp(name = "ריקורדר", group = "autotrain")
class Recorder : BraveNewWorld() {
    private val actionsList = mutableListOf<ByteArray>()
    private var startTimestamp : Long = 0
    override fun extendInit(){
        telemetry.speak("Prepare thyself, recording is to commence on start")
        telemetry.update()
    }

    override fun start() {
        actionsList.add(gamepad1.toByteArray()) // To make sure things start out at 0 (may cause issues with timestamp)
        startTimestamp = System.currentTimeMillis()
        Gamepad.GamepadCallback { // Add new callback to when any event is fired on the gamepads
            if(it.id == gamepad1.id){ // Just record gamepad1 for now
                val timestamp = System.currentTimeMillis()-startTimestamp
                var tempGamepad = Gamepad()
                tempGamepad.copy(gamepad1) // Copy gamepad state
                tempGamepad.timestamp = timestamp // Replace timestamp with our absolute stamp
                actionsList.add(tempGamepad.toByteArray()) // Add it to the list
            }
        }
    }

    override fun init_loop() {
        var funcounter = 0.0f
        telemetry.addData("Brave New World", "Recorded")
        telemetry.addData("Version", funcounter)
        funcounter += 0.1f
        telemetry.update()
    }

    override fun extendStop() {
        telemetry.clear()
        telemetry.addLine("Do you want to save this recording?")
        telemetry.addLine("Press A to SAVE & EXIT")
        telemetry.addLine("Press B to EXIT w/o SAVING")
        telemetry.speak("Make your choice")
        telemetry.update()
        while(!gamepad1.a && !gamepad1.b){
            Thread.sleep(5)
        }
        if(gamepad1.a){
            val ostream = ObjectOutputStream(FileOutputStream("autonomous.recording")) // no unique file name for now, we'll do that later
            ostream.writeObject(actionsList)
            ostream.close()
        }
        telemetry.addLine("Stopping in 3")
        telemetry.update()
        Thread.sleep(1000)
        telemetry.addLine("Stopping in 2")
        telemetry.update()
        Thread.sleep(1000)
        telemetry.addLine("Stopping in 1")
        telemetry.update()
        Thread.sleep(1000)
    }
}