import java.io.*;
import java.net.*;


public class SendUDP {
	String ipRobot="192.168.1.133";  // 138 ou 133
	public void SendUDP() {
		try{
	//	  System.out.println("argument: " + args[0]);
			  System.out.println("sendUDP" );
	      DatagramSocket clientSocket = new DatagramSocket();
	      InetAddress IPAddress = InetAddress.getByName(ipRobot);
	      byte[] sendData = new byte[3];
	      String startCmde="c4x";
	      sendData = startCmde.getBytes();
	      DatagramPacket sendPacket = new DatagramPacket(sendData, sendData.length, IPAddress, 8888);
	      clientSocket.send(sendPacket);
	      clientSocket.close();
	//   System.exit(0);
	   
	  	   
	   
	   
		}
	   catch(Exception e)
	   {}
	   
	   finally{}
		}
	public void SendUDPStop() {
		try{
	  System.out.println("stop" );
	      DatagramSocket clientSocket = new DatagramSocket();
	      InetAddress IPAddress = InetAddress.getByName(ipRobot);
	      byte[] sendData = new byte[3];
	      String startCmde="c4s";
	      sendData = startCmde.getBytes();
	      DatagramPacket sendPacket = new DatagramPacket(sendData, sendData.length, IPAddress, 8888);
	      clientSocket.send(sendPacket);
	      clientSocket.close();
	//   System.exit(0);
	  
	  	 
	   
	   
		}
	   catch(Exception e)
	   {}
	   
	   finally{}
		}
	public void SendUDPStart() {
		// TODO Auto-generated method stub
		try{
			  System.out.println("start" );
			      DatagramSocket clientSocket = new DatagramSocket();
			      InetAddress IPAddress = InetAddress.getByName(ipRobot);
			      byte[] sendData = new byte[3];
			      String startCmde="c4x";
			      sendData = startCmde.getBytes();
			      DatagramPacket sendPacket = new DatagramPacket(sendData, sendData.length, IPAddress, 8888);
			      clientSocket.send(sendPacket);
			      clientSocket.close();
			//   System.exit(0);
			  
			  	 
			   
			   
				}
			   catch(Exception e)
			   {}
			   
			   finally{}
				}
	public void SendUDPScan() {
		// TODO Auto-generated method stub
		try{
			  System.out.println("scan" );
			      DatagramSocket clientSocket = new DatagramSocket();
			      InetAddress IPAddress = InetAddress.getByName(ipRobot);
			      byte[] sendData = new byte[3];
			      String startCmde="c4+";
			      sendData = startCmde.getBytes();
			      DatagramPacket sendPacket = new DatagramPacket(sendData, sendData.length, IPAddress, 8888);
			      clientSocket.send(sendPacket);
			      clientSocket.close();
			  	 
			   
			   
				}
			   catch(Exception e)
			   {}
			   
			   finally{}
				}
	public void SendUDPInit() {
		// TODO Auto-generated method stub
		try{
			  System.out.println("init" );
			      DatagramSocket clientSocket = new DatagramSocket();
			      InetAddress IPAddress = InetAddress.getByName(ipRobot);
			      byte[] sendData = new byte[3];
			      String startCmde="c4I";
			      sendData = startCmde.getBytes();
			      DatagramPacket sendPacket = new DatagramPacket(sendData, sendData.length, IPAddress, 8888);
			      clientSocket.send(sendPacket);
			      clientSocket.close();
			  	 
			   
			   
				}
			   catch(Exception e)
			   {}
			   
			   finally{}
				}
	public void SendEcho() {
		try{
	  System.out.println("echo" );
	      DatagramSocket clientSocket = new DatagramSocket();
	      InetAddress IPAddress = InetAddress.getByName(ipRobot);
	      byte[] sendData = new byte[3];
	      String startCmde="c4e";
	      sendData = startCmde.getBytes();
	      DatagramPacket sendPacket = new DatagramPacket(sendData, sendData.length, IPAddress, 8888);
	      clientSocket.send(sendPacket);
	      clientSocket.close();
  	 	   
	   
		}
	   catch(Exception e)
	   {}
	   
	   finally{}
		}
	public void SendUDPMove(long angle,long move) {
		// TODO Auto-generated method stub
		try{
			
			  System.out.println("angle:"+angle+ " move:"+move );
			      DatagramSocket clientSocket = new DatagramSocket();
			      InetAddress IPAddress = InetAddress.getByName(ipRobot);
			      byte[] cmde = new byte[15];
			      byte[] sendData = new byte[15];
			      cmde[0]=0x63;
			      cmde[1]=0x34;
			      cmde[2]=0x6d;
			      if (angle>=0){
				      cmde[3]=0x2b;
			      }
			      else {
				      cmde[3]=0x2d;
				      angle=-angle;
			      }
			      cmde[4]=(byte)(angle/256);
			      cmde[5]=(byte)(angle);
			      if (move>=0){
				      cmde[6]=0x2b;
			      }
			      else {
				      cmde[6]=0x2d;
				      move=-move;
			      }
			      cmde[7]=(byte)(move/256);
			      cmde[8]=(byte)(move);
	//		      int i;
//			      for (i=9;i<20;i++)
//			      	{
//			    	  cmde[i]=0x00;
//			      	}
//			      String startCmde="c4m";
		      sendData = cmde;
		      System.out.println(byteArrayToHex(sendData));
			      DatagramPacket sendPacket = new DatagramPacket(sendData, sendData.length, IPAddress, 8888);
			      clientSocket.send(sendPacket);
			      clientSocket.close();
			//   System.exit(0);
			  		  	 
			   
			   
				}
			   catch(Exception e)
			   {}
			   
			   finally{}
				}
	public static String byteArrayToHex(byte[] a) {
		   StringBuilder sb = new StringBuilder(a.length * 2);
		   for(byte b: a)
		      sb.append(String.format("%02x", b & 0xff));
		   return sb.toString();
		}

	}
	


