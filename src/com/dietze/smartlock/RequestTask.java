package com.dietze.smartlock;

import java.io.ByteArrayOutputStream;
import java.io.IOException;

import org.apache.http.HttpResponse;
import org.apache.http.HttpStatus;
import org.apache.http.StatusLine;
import org.apache.http.client.ClientProtocolException;
import org.apache.http.client.HttpClient;
import org.apache.http.client.methods.HttpGet;
import org.apache.http.impl.client.DefaultHttpClient;

import android.content.Context;
import android.os.AsyncTask;
import android.util.Log;
import android.widget.Toast;

/*
 * AsyncTask class responsible for all the networking threads for the SmartLock Application.
 * 
 * Must be constructed with a context in order to post errors to the main thread.
 * 
 * Includes minor error handling.
 * 
 */
class RequestTask extends AsyncTask<String, String, String>{
	
	Context context = null;
	int duration = Toast.LENGTH_SHORT;
	boolean isError = false;
	SmartLockController slc;

	public RequestTask(Context context){
		setContext(context);
		this.slc = slc;
	}
	
	public void setContext(Context context){
		this.context = context;
	}
	
    @Override
    protected String doInBackground(String... uri) {
    	
        HttpClient httpclient = new DefaultHttpClient();
        HttpResponse response;
        String responseString = null;
        try {
            response = httpclient.execute(new HttpGet(uri[0]));
            StatusLine statusLine = response.getStatusLine();
            if(statusLine.getStatusCode() == HttpStatus.SC_OK){
                ByteArrayOutputStream out = new ByteArrayOutputStream();
                response.getEntity().writeTo(out);
                out.close();
                responseString = out.toString();
                Log.d("Request Task", responseString);
                
            } else{
                //Closes the connection.
                response.getEntity().getContent().close();
                throw new IOException(statusLine.getReasonPhrase());
            }
        } catch (ClientProtocolException e) {
            responseString = showError(e);
        	
        	
        } catch (IOException e) {
            //TODO Handle problems..
        	responseString = showError(e);
        }
        
        return responseString;
    }
    
    //Prints the Error to the Android Log and sets the isError boolean to True
    String showError(Exception e){
    	isError = true;
    	String error = e.getMessage();
    	
    	Log.d("Request Task", error);
    	return error;
    	
    }
    
    

    @Override
    protected void onPostExecute(String result) {
        super.onPostExecute(result);
        
        //If there was an error post it as a toast to the main thread
        if(isError){
        	Toast.makeText(context, result, duration).show();
        }
    }
}