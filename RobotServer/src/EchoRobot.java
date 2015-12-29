import java.io.*;
import java.net.*;
import java.sql.*;
public class EchoRobot extends Thread{
	Thread t;
	public static int pendingEcho=2;
	public void run(){


while (true){

	try{

	      DatagramSocket clientSocket = new DatagramSocket();
	      InetAddress IPAddress = InetAddress.getByName("192.168.1.138");
	      byte[] sendData = new byte[3];
	      String startCmde="c4e";
	      sendData = startCmde.getBytes();
	      DatagramPacket sendPacket = new DatagramPacket(sendData, sendData.length, IPAddress, 8888);
	      clientSocket.send(sendPacket);

	      System.out.println("send echo:"+pendingEcho);
	     clientSocket.close();
	//	  System.out.println("statut du thread "   +this.getState());
	      if (pendingEcho<=1)
	      {

		   Thread.sleep(30000);
	      }
	      else
	      {
			Thread.sleep(5000);

	      }
		      pendingEcho=pendingEcho+1;
	//		  System.out.println("statut du thread "   +this.getState());
	   	   
		}
	   catch(Exception e)
	   {}
	   
	   finally{}

	   /*
	      Thread.sleep(10000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			}
			*/
	}
	
		}

	
	}

