package com.example.RobotArm

import android.content.Context
import android.net.wifi.WifiManager
import android.util.Log
import android.widget.Toast
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.withContext
import java.io.BufferedReader
import java.io.InputStreamReader
import java.io.OutputStreamWriter
import java.io.PrintWriter
import java.net.InetSocketAddress
import java.net.Socket

class ESP32_Server (private val address: String,
                    private val port: Int,
                    private val context: Context,
                    private val verbose: Boolean = false)
{
    private var socket: Socket? = null
    private lateinit var writer: PrintWriter
    private lateinit var reader: BufferedReader
    private var receiveJob: Job? = null
    private val Debug_Tag = "ESP32_Server"

    object CommandFlags {
        const val MoveJoystick: Byte    = 0x09
        const val Move: Byte            = 0x10
        const val Calib: Byte           = 0x11
        const val EmerStop: Byte        = 0x12
        const val Get_angle: Byte       = 0x13
        const val Get_speed: Byte       = 0x14
        const val Get_all: Byte         = 0x15
        const val Jog: Byte             = 0x16
        const val Move_noFB: Byte       = 0x17
        const val MoveHome: Byte        = 0x26
        const val ClearError: Byte      = 0x27
//        const val Rob_ena: Byte         = 0x29
    }

    object StatusFlags {
        const val Ready: Byte           = 0x01
        const val Busy: Byte            = 0x02
        const val Error: Byte           = 0x03
        const val Complete: Byte        = 0x28
        const val Home: Byte            = 0x25      // Out of Range
        const val OOR: Byte             = 0x04      // Out of Range
        const val OOS: Byte             = 0x05      // Out of Speed
        const val LIM: Byte             = 0x06      // Lost I2C Memory
        const val LIE: Byte             = 0x07      // Lost I2C Encoder
        const val LI: Byte              = 0x08      // Lost all I2C Peripheral
        const val TCP_error: Byte       = 0x0A
        const val TCP_norm: Byte        = 0x0B
    }

    object JogMode {
        const val Linear: Byte          = 0x18
        const val J123: Byte            = 0x19
        const val J456: Byte            = 0x20
        const val Reori: Byte           = 0x21
        const val Angle: Byte           = 0x22
        const val Posi: Byte            = 0x23
        const val Calib: Byte           = 0x24
    }

    suspend fun connect(): Boolean {
        try {
            if (socket == null || socket!!.isClosed) {
                val wifiManager = context.getSystemService(Context.WIFI_SERVICE) as WifiManager
                val ssid = wifiManager.connectionInfo.ssid.replace("\"", "")

                if (!ssid.contains("ESP32-Robot-Mode", ignoreCase = true)) {
                    withContext(Dispatchers.Main) {
                        Toast.makeText(context, "Please connect to the Robot!", Toast.LENGTH_LONG).show()
                    }
                    Log.e(Debug_Tag, "WiFi not connected to ESP32")
                    return false
                }
                Log.i(Debug_Tag, "Connected to ESP32 Wifi")
                val sock = Socket()
                sock.connect(InetSocketAddress(address, port), 1500)
                socket = sock
                writer = PrintWriter(OutputStreamWriter(socket!!.getOutputStream()), true)
                reader = BufferedReader(InputStreamReader(socket!!.getInputStream()))
                Log.i(Debug_Tag, "Connected to ESP32 at $address:$port")

            }

        } catch (e: Exception) {
            Log.e(Debug_Tag, "Connection failed: ${e.message}")
        }
        return true
    }

    suspend fun send(jsonData: String) {
        if (jsonData.isBlank()) {
            if (verbose) Log.i(Debug_Tag, "Attempted to send blank message. Ignored.")
            return
        }
        withContext(Dispatchers.IO) {
            try {
                if (socket == null || socket!!.isClosed || !::reader.isInitialized) {
                    connect()
                }
                if (::writer.isInitialized) {
                    writer.println(jsonData)
                    writer.flush()
                    if (verbose) Log.i(Debug_Tag, "Sent: $jsonData") else {

                    }
                } else {

                }
            } catch (e: Exception) {
                Log.e(Debug_Tag, "Send failed: ${e.message}")
            }
        }
    }

    suspend fun receive(): String? {
        return withContext(Dispatchers.IO) {
            try {
                if (socket == null || socket!!.isClosed || !::reader.isInitialized) {
                    connect()
                }
                val response = reader.readLine()
                if (verbose && (response!=null) ) Log.i(Debug_Tag, "Received: $response")
                response
            } catch (e: Exception) {
                Log.e(Debug_Tag, "Receive failed: ${e.message}")
                null
            }
        }
    }
}


