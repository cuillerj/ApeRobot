import java.net.DatagramPacket;

//import javax.swing.*;

//import javax.swing.JPanel;
//import java.awt.*;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.io.*; 
import java.net.*;
import java.sql.*;
import java.util.Date;
import java.util.Calendar;
public class TraceEvents extends Thread{
	
	public static int posXG=0;
	public static int posYG=0;
	public static int orientG=0;
	public static String idscanG="0";
	public static byte actStat=0x00;
	public static String stationStatus="";
	static char[] TAB_BYTE_HEX = { '0', '1', '2', '3', '4', '5', '6','7',
            '8', '9', 'A', 'B', 'C', 'D', 'E','F' };
public  void run()
			{ 

	//	FenetreGraphique graph = new FenetreGraphique();
	//	PanneauGraphique.point(posXG,posYG);
	//	graph.repaint();
	//	graph.SetInitialPosition();
	try{
			DatagramSocket serverSocket = new DatagramSocket(1831); 
			//byte[] receiveData = new byte[1024];
			//byte[] sendData = new byte[1024];
			String FNamD="logdata";
			int ID=0; // station ID
			byte Type=0; // station type
			int myID=0; // station ID
			int InpLen=0; // input datalength - lue dans la trame UDP
			int FreeCycle=0;
			int posXP=0;
			int posYP=0;
			int orientP=0;
			int trameNumber=0;
			byte lastTrameNumber=0;

		    //System.out.println(" - time:"+new java.sql.Timestamp(today.getTime()));
		    System.out.println(" Start");
		//	EchoRobot echo=new EchoRobot();
//		echo.EchoRobot();
			//echo.start();
	//	      SendUDP snd = new SendUDP();
	//	      snd.SendEcho();
			while(true)
			{




//				ResultSet rs = null;

				byte[] sentence2=new byte[1024];
		//		byte[] sendData = new byte[1024];
				DatagramPacket receivePacket = new DatagramPacket(sentence2, sentence2.length);
				serverSocket.receive(receivePacket);


				 sentence2 =  receivePacket.getData();
				 InpLen=(byte)sentence2[4];

				int oct0=(byte)(sentence2[0]&0x7F)-(byte)(sentence2[0]&0x80); // manip car byte consideré signé
				int oct1=(byte)(sentence2[1]&0x7F)-(byte)(sentence2[1]&0x80);
				ID= 256*oct0+oct1;
//				ID=sentence2[0];
				Type=sentence2[2];

				trameNumber=sentence2[15];
//			    java.util.Date today = new java.util.Date();
			    java.util.Date today = new java.util.Date();

			    System.out.print(today+ ": ");
			    System.out.print("Trace ID " + ID+" Type "+Type+ ": ");


				String sentence = new String( sentence2);
				System.out.println(sentence.substring(5));



		//			System.out.println("RECEIVED length: " + sentence2.length);
//				System.out.println(" RECEIVED: " +sentence2[6]);

				int idx=Type;



			   }
			
			}
	   catch(Exception e)
	   {}
	   
	   finally{}
			}
		//   System.exit(0);
		 

			


/*public void RefreshAff() {
	// TODO Auto-generated method stub
    ihm.RefreshStat();
	}
	}
//   System.exit(0);
  */

static void hexaPrint(byte y)
{

		System.out.print("-" + String.format("%x", y));
	}
}
