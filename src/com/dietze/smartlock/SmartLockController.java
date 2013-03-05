package com.dietze.smartlock;


import java.util.ArrayList;
import java.util.Set;

import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.Context;
import android.content.Intent;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.os.Message;
import android.util.Log;
import android.view.View;
import android.view.View.OnClickListener;
import android.view.Window;
import android.widget.AdapterView;
import android.widget.AdapterView.OnItemSelectedListener;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.EditText;
import android.widget.Spinner;
import android.widget.TextView;

public class SmartLockController extends Activity implements Runnable, Handler.Callback, OnItemSelectedListener{
	
	String ipaddress = "10.10.1.100";
	Context context;
	
	//App State
	private BlueSmirfSPP      mSPP;
	private boolean           mIsThreadRunning;
	private String            mBluetoothAddress;
	private ArrayList<String> mArrayListBluetoothAddress;

	// UI
	private TextView     mTextViewStatus;
	private Spinner      mSpinnerDevices;
	private ArrayAdapter mArrayAdapterDevices;
	private Handler      mHandler;

	// Arduino state
	private int mStateLED;
	private int mStatePOT;
	private int LEDStatus;

	public SmartLockController()
	{
		mIsThreadRunning           = false;
		mBluetoothAddress          = null;
		mSPP                       = new BlueSmirfSPP();
		mStateLED                  = 0;
		mStatePOT                  = 0;
		mArrayListBluetoothAddress = new ArrayList<String>();
		
	}

	@Override
	public void onCreate(Bundle savedInstanceState)
	{
		super.onCreate(savedInstanceState);
		
		// initialize UI
		setContentView(R.layout.main);
		//requestWindowFeature(Window.FEATURE_NO_TITLE);
		
		mTextViewStatus         = (TextView) findViewById(R.id.ID_STATUS);
		ArrayList<String> items = new ArrayList<String>();
		mSpinnerDevices         = (Spinner) findViewById(R.id.ID_PAIRED_DEVICES);
		mArrayAdapterDevices    = new ArrayAdapter<String>(this, android.R.layout.simple_spinner_item, items);
		mHandler                = new Handler(this);
		mSpinnerDevices.setOnItemSelectedListener(this);
		mArrayAdapterDevices.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
		mSpinnerDevices.setAdapter(mArrayAdapterDevices);
				
		
		
		
		context = getApplicationContext();
		
		findViewById(R.id.button_open).setOnClickListener(Listener);
		findViewById(R.id.button_lock).setOnClickListener(Listener);
		
		Log.d("SmartLock Controller", "OnCreate finished");
		
		
	}

	@Override
	protected void onResume()
	{
		super.onResume();

		// update the paired device(s)
		BluetoothAdapter adapter = BluetoothAdapter.getDefaultAdapter();
		Set<BluetoothDevice> devices = adapter.getBondedDevices();
		mArrayAdapterDevices.clear();
		mArrayListBluetoothAddress.clear();
		Log.d("SmartLock Controller", "Adapters Cleared");
		if(devices.size() > 0)
		{
			for(BluetoothDevice device : devices)
			{
				mArrayAdapterDevices.add(device.getName());
				mArrayListBluetoothAddress.add(device.getAddress());
			}
			
			Log.d("SmartLock Controller", "Adapters Filled");
			// request that the user selects a device
			if(mBluetoothAddress == null)
			{
				//mSpinnerDevices.performClick();
			}
		}
		else
		{
			mBluetoothAddress = null;
		}

		UpdateUI();
	}

	@Override
	protected void onPause()
	{
		mSPP.disconnect();
		super.onPause();
	}

	@Override
	protected void onDestroy()
	{
		super.onDestroy();
	}

	/*
	 * Spinner callback
	 */

	public void onItemSelected(AdapterView<?> parent, View view, int pos, long id)
	{
		mBluetoothAddress = mArrayListBluetoothAddress.get(pos);
	}

	public void onNothingSelected(AdapterView<?> parent)
	{
		mBluetoothAddress = null;
	}

	/*
	 * buttons
	 */

	public void onBluetoothSettings(View view)
	{
		Intent i = new Intent(android.provider.Settings.ACTION_BLUETOOTH_SETTINGS);
		startActivity(i);
	}

	public void onBluetoothLock(View view)
	{
		if(mSPP.isConnected())
		{
			mStateLED = 1;
			
		}
		
		
		
	}
	
	public void onBluetoothOpen(View view)
	{
		
		if(mSPP.isConnected())
		{
			
			mStateLED = 2;
		}
		
		
		
	}
	
	
	

	public void onConnectLink(View view)
	{
		if(mIsThreadRunning == false)
		{
			mIsThreadRunning = true;
			Thread t = new Thread(this);
			t.start();
			UpdateUI();
		}
		
		
		
	}

	public void onDisconnectLink(View view)
	{
		mSPP.disconnect();
		//startThread();
	}

	/*
	 * main loop
	 */

	public void run()
	{
		Looper.prepare();
		mSPP.connect(mBluetoothAddress);
		int tempStatus = 0;
		while(mSPP.isConnected())
		{
			
			
			mSPP.flush();
			if(mStateLED != 0){
			mSPP.writeByte(mStateLED);
			}
			mSPP.flush();
			
			mStateLED = 0;
			
			tempStatus = mSPP.readByte();
			
			if(tempStatus != 0){
				LEDStatus = tempStatus;
				Log.d("Bluetooth",Integer.toString(LEDStatus));
			}
			
			
			if(mSPP.isError())
			{
				mSPP.disconnect();
			}
			
			mHandler.sendEmptyMessage(0);

			// wait briefly before sending the next packet
			try { Thread.sleep(700); }
			catch(InterruptedException e) { Log.e("Bluetooth", e.getMessage());}
			//mIsThreadRunning = false;
		}
		
		mStateLED        = 0;
		mIsThreadRunning = false;
		mHandler.sendEmptyMessage(0);
		
	}

	/*
	 * update UI
	 */

	public boolean handleMessage (Message msg)
	{
		// received update request from Bluetooth IO thread
		UpdateUI();
		return true;
	}

	private void UpdateUI()
	{
		if(mSPP.isConnected())
		{
			String LEDstate = "Unknown";
			
			if(LEDStatus == 1){
				LEDstate = "on";
				mTextViewStatus.setText("connected to " + mSPP.getBluetoothAddress() + "\n" +
                        "LED is " + LEDstate + "\n");
			}
			else if(LEDStatus == 2){
				LEDstate = "off";
				mTextViewStatus.setText("connected to " + mSPP.getBluetoothAddress() + "\n" +
                        "LED is " + LEDstate + "\n");
			}
			else{ 
				LEDstate = "Unknown";
				mTextViewStatus.setText("connected to " + mSPP.getBluetoothAddress() + "\n" +
                    "LED is " + LEDstate + "\n");
			}
			
			
			                       
		}
		else if(mIsThreadRunning)
		{
			mTextViewStatus.setText("connecting to " + mBluetoothAddress);
		}
		else
		{
			mTextViewStatus.setText("disconnected");
		}
	}

	

	
	private OnClickListener Listener = new OnClickListener() { 
		
	    public void onClick(View v) {
	    	
	    	if (v instanceof Button){
	           String buttonText = ((Button) v).getText().toString();
	           //Log.d("Parse App", "Button Text:" + buttonText);
	           
	           String ipaddress = ((EditText) findViewById(R.id.editText1)).getText().toString();
	           
	           if (buttonText.equals("Open")){
	        	   //Do something
	        	   Log.d("SmartLock App", "Sending Open Command");
	        	   new RequestTask(context).execute("http://"+ipaddress+"/?OPEN");
	        	   
	        	   
	           }
	           
	           else if (buttonText.equals("Lock")){
	        	   //Do something
	        	   Log.d("SmartLock App", "Sending Lock Command");
	        	   new RequestTask(context).execute("http://"+ipaddress+"/?LOCK");
	        	   
	        	   
	           }
	           
	           else {
	        	   //Do nothing
	        	   Log.d("SmartLock App", "Click Matches No Known Actions");
	           }
	    	}
	    	else {
		    	Log.d("SmartLock App", "Not a Button");
		    }
	    	
	    }
	    
	}; 
}

