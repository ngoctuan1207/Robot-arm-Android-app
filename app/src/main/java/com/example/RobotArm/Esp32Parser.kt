package com.example.RobotArm

import com.google.gson.Gson
import com.google.gson.JsonSyntaxException

object Esp32Parser {
    fun parse(json: String): Esp32Feedback? {
        return try {
            Gson().fromJson(json, Esp32Feedback::class.java)
        }
        catch (e: JsonSyntaxException) {
            null
        }
    }
}
