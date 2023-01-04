package org.firstinspires.ftc.teamcode.robot

import org.firstinspires.ftc.robotcore.external.Telemetry
import kotlin.reflect.KProperty

typealias Callback = () -> Unit
typealias Predicate = () -> Boolean

class EventLoop(private val opModeIsActive: Predicate, private val telemetry: Telemetry? = null) {
    /**
     * Routines that need to be called on every loop.
     */
    val updates = mutableListOf<Callback>()

    /**
     * Routines that happen once over an interval of time. Each [Callback] is executed while its [Predicate]
     * returns true, and the pair is removed otherwise.
     */
    val singleEvents = mutableListOf<Pair<Predicate, Callback>>()

    private val keyEvents = mutableListOf<Pair<Predicate, Callback>>()

    fun onPressed(key: KProperty<*>, func: Callback) {
        keyEvents.add({ pressed(key) } to func)
    }

    fun onAnyPressed(vararg keys: KProperty<*>, func: Callback) {
        keyEvents.add({ keys.any { pressed(it) } } to func)
    }

    fun onReleased(key: KProperty<*>, func: Callback) {
        keyEvents.add({ released(key) } to func)
    }

    fun onAnyReleased(vararg keys: KProperty<*>, func: Callback) {
        keyEvents.add({ keys.any { pressed(it) } } to func)
    }

    /**
     * Start the event loop, which will run blocking until the method passed in the constructor returns false.
     */
    fun run() {
        while (opModeIsActive()) {
            telemetry?.addData("size", updates.size)
            telemetry?.addData("keys", keyEvents.size)
            telemetry?.addData("single", singleEvents.size)

            keyEvents.filter { it.first() }
                .forEach { it.second() }
            if (singleEvents.size >= 1) singleEvents.apply { retainAll { it.first() } }
                .forEach { println("run second"); it.second() }
            updates.forEach { it() }
        }
    }
}
