package com.example.minhkhang1795.androidapp;

import android.app.ProgressDialog;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.Intent;
import android.os.AsyncTask;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.Toast;

import java.io.IOException;
import java.util.UUID;

public class MainActivity extends AppCompatActivity {

    Button btnMode1, btnMode2, btnMode3, btnMode4, btnDis;
    String address = null;
    ProgressDialog progress;
    BluetoothAdapter myBluetooth = null;
    BluetoothSocket btSocket = null;
    boolean isBtConnected = false;
    // SPP UUID. Look for it
    static final UUID myUUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        // Receive the address of the bluetooth device
        Intent newint = getIntent();
        address = newint.getStringExtra(DeviceList.EXTRA_ADDRESS);

        // Connect the layouts
        setContentView(R.layout.activity_main);
        btnMode1 = findViewById(R.id.btnMode1);
        btnMode2 = findViewById(R.id.btnMode2);
        btnMode3 = findViewById(R.id.btnMode3);
        btnMode4 = findViewById(R.id.btnMode4);
        btnDis = findViewById(R.id.btnDis);

        // Commands to be sent to bluetooth
        btnMode1.setOnClickListener(new ModeButtonClick("1"));
        btnMode2.setOnClickListener(new ModeButtonClick("2"));
        btnMode3.setOnClickListener(new ModeButtonClick("3"));
        btnMode4.setOnClickListener(new ModeButtonClick("4"));
        btnDis.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                disconnect();
            }
        });

        // Call the class to connect Bluetooth
        new ConnectBT().execute();
    }

    class ModeButtonClick implements View.OnClickListener {
        String mode;

        ModeButtonClick(String mode) {
            this.mode = mode;
        }

        @Override
        public void onClick(View v) {
            if (mode != null)
                setMode(mode);
        }
    }

    private void disconnect() {
        if (btSocket != null) { // If the btSocket is busy
            try {
                btSocket.close(); // Close connection
            } catch (IOException e) {
                msg("Error");
            }
        }
        finish(); // Return to the first layout

    }

    private void setMode(String mode) {
        if (btSocket != null) {
            try {
                btSocket.getOutputStream().write(mode.getBytes());
                msg("Successfully switched to mode " + mode);
            } catch (IOException e) {
                msg("Error");
            }
        }
    }

    // Fast way to call Toast
    private void msg(String s) {
        Toast.makeText(getApplicationContext(), s, Toast.LENGTH_SHORT).show();
    }

    private class ConnectBT extends AsyncTask<Void, Void, Void>  { // UI thread
        private boolean connectSuccess = true; // If it's here, it's almost connected

        @Override
        protected void onPreExecute() {
            progress = ProgressDialog.show(MainActivity.this, "Connecting...", "Please wait!!!");
        }

        @Override
        protected Void doInBackground(Void... devices) { // While the progress dialog is shown, the connection is done in background
            try {
                if (btSocket == null || !isBtConnected) {
                    myBluetooth = BluetoothAdapter.getDefaultAdapter();// Get the mobile bluetooth device
                    BluetoothDevice dispositivo = myBluetooth.getRemoteDevice(address);// Connects to the device's address and checks if it's available
                    btSocket = dispositivo.createInsecureRfcommSocketToServiceRecord(myUUID);// Create a RFCOMM (SPP) connection
                    BluetoothAdapter.getDefaultAdapter().cancelDiscovery();
                    btSocket.connect();//start connection
                }
            } catch (IOException e) {
                connectSuccess = false;// If the try failed, you can check the exception here
            }
            return null;
        }

        @Override
        protected void onPostExecute(Void result) { // After the doInBackground, checks if everything went fine
            super.onPostExecute(result);

            if (!connectSuccess) {
                msg("Connection Failed. Is it a SPP Bluetooth? Try again.");
                finish();
            } else {
                msg("Connected.");
                isBtConnected = true;
            }
            progress.dismiss();
        }
    }
}
