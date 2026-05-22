package com.example.RobotArm

import android.content.Context
import android.view.View
import android.widget.AdapterView
import android.widget.ArrayAdapter
import android.widget.Spinner

fun setupSpinner(
    context: Context,
    spinner: Spinner,
    arrayRes: Int,
    layoutRes: Int,
    onSelected: (String) -> Unit
) {
    val items = context.resources.getStringArray(arrayRes)
    val adapter = ArrayAdapter(
        context,
        layoutRes,
        items
    )
    adapter.setDropDownViewResource(layoutRes)
    spinner.adapter = adapter
    spinner.onItemSelectedListener = object : AdapterView.OnItemSelectedListener {
        override fun onItemSelected(
            parent: AdapterView<*>?,
            view: View?,
            position: Int,
            id: Long
        ) {
            val selected = parent?.getItemAtPosition(position).toString()
            onSelected(selected)
        }
        override fun onNothingSelected(parent: AdapterView<*>?) {

        }
    }
}
