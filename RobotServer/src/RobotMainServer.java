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
public class RobotMainServer 
	{
	public static int posXG=0;
	public static int posYG=0;
	public static int orientG=0;
	public static String idscanG="0";
	public static byte actStat=0x00;
	public static String stationStatus="";
	static char[] TAB_BYTE_HEX = { '0', '1', '2', '3', '4', '5', '6','7',
            '8', '9', 'A', 'B', 'C', 'D', 'E','F' };
public static void main(String args[]) throws Exception
			{ 
		GetLastScanID();
		System.out.println("lastid:"+idscanG+" "+posXG);

		Fenetre ihm = new Fenetre();
		Fenetre2 ihm2 = new Fenetre2();
		FenetreGraphiqueSonar ihm3 = new FenetreGraphiqueSonar();
		ihm2.SetInitialPosition();
		ihm3.SetInitialPosition();
	//	FenetreGraphique graph = new FenetreGraphique();
	//	PanneauGraphique.point(posXG,posYG);
	//	graph.repaint();
	//	graph.SetInitialPosition();
			DatagramSocket serverSocket = new DatagramSocket(1830); 
			//byte[] receiveData = new byte[1024];
			//byte[] sendData = new byte[1024];
			int ID=0; // station ID
			byte Type=0; // station type
			int InpLen=0; // input datalength - lue dans la trame UDP
			int trameNumber=0;
			byte lastTrameNumber=0;
		//	EchoRobot echo=new EchoRobot();
//		echo.EchoRobot();
			//echo.start();
			//	EchoRobot echo=new EchoRobot();
//			echo.EchoRobot();
				//echo.start();
	//	      SendUDP snd = new SendUDP();
	//	      snd.SendEcho();
			TraceEvents trace=new TraceEvents();
//			trace.TraceEvents();
			trace.start();
			while(true)
			{
			    ihm.RefreshStat();
				Connection conn = null;
				Statement stmtR = null;
				Statement stmtI = null;

//				ResultSet rs = null;
				byte[] receiveData = new byte[1024];
				byte[] sentence2=new byte[1024];
		//		byte[] sendData = new byte[1024];
				DatagramPacket receivePacket = new DatagramPacket(sentence2, sentence2.length);
				serverSocket.receive(receivePacket);
//				String sentence = new String( receivePacket.getData());

				 sentence2 =  receivePacket.getData();
				 InpLen=(byte)sentence2[4];
				

				ID=sentence2[0];
	//			Type=sentence2[2];

			 trameNumber=sentence2[7];
//			 for (int i=0;i<50;i++)
//			 {
//				 hexaPrint(sentence2[i]);

//			 }
//			 System.out.println();
				System.out.print(" ID " + ID+ " len "+InpLen+" ");
		//			System.out.println("RECEIVED length: " + sentence2.length);
//				System.out.println(" RECEIVED: " +sentence2[7]);
				EchoRobot.pendingEcho=0;
				int idx=Type;
				if (sentence2[6]==0x01){ // scan -test InpLen contournement bug && ( InpLen==10)

					System.out.print( " n°:"+trameNumber);
					
	//			String FNam=FNamD+idx+".txt";  // modifie le nom du fichier log en ajoutant le type de la station emettrice
			    java.util.Date today = new java.util.Date();
			    //System.out.println(" - time:"+new java.sql.Timestamp(today.getTime()));
			    System.out.println(" - time: "+today);
				 Thread.sleep(300);
				InetAddress IPAddress = receivePacket.getAddress();
			      DatagramSocket clientSocket = new DatagramSocket();
//			      InetAddress IPAddress = InetAddress.getByName("192.168.1.99");
			      byte[] sendData = new byte[4];
			      sendData[0]=0x63;
			      sendData[1]=0x34;
			      sendData[2]=0x61;
			      sendData[3]=sentence2[7];
//				      String startCmde="c4a";
//			      sendData = startCmde.getBytes();
			      DatagramPacket sendPacket = new DatagramPacket(sendData, sendData.length, IPAddress, 8888);
			      clientSocket.send(sendPacket);
			     clientSocket.close();
				int port = receivePacket.getPort();
				int i2=9;
				int oct0=(byte)(sentence2[i2]&0x7F)-(byte)(sentence2[i2]&0x80); // manip car byte consideré signé
				int oct1=(byte)(sentence2[i2+1]&0x7F)-(byte)(sentence2[i2+1]&0x80);
				int distFront=256*oct0+oct1;
				i2=12;
				 oct0=(byte)(sentence2[i2]&0x7F)-(byte)(sentence2[i2]&0x80); // manip car byte consideré signé
				 oct1=(byte)(sentence2[i2+1]&0x7F)-(byte)(sentence2[i2+1]&0x80);
				int distBack=256*oct0+oct1;
				i2=15;
				 oct0=(byte)(sentence2[i2]&0x7F)-(byte)(sentence2[i2]&0x80); // manip car byte consideré signé
				 oct1=(byte)(sentence2[i2+1]&0x7F)-(byte)(sentence2[i2+1]&0x80);
				int angle=256*oct0+oct1;

	//			PanneauGraphique.point(22,22);
	//			ihm2.setAlwaysOnTop(true);
//				ihm2.setLocation(20,20);
		//		ihm.setTitle("robot v0 - time: "+today);
	//			ihm2.setSize("pour voir");
if (lastTrameNumber!=trameNumber)
{
				try {
					Class.forName("com.mysql.jdbc.Driver").newInstance();
					String connectionUrl = "jdbc:mysql://jserver:3306/robot";
					String connectionUser = "jean";
					String connectionPassword = "manu7890";
					conn = DriverManager.getConnection(connectionUrl, connectionUser, connectionPassword);
//					conn.setAutoCommit(false);
					stmtR = conn.createStatement();
					stmtI = conn.createStatement();

//				String sql="INSERT INTO IndValue VALUES ("+ID+","+IndValue+",now(),"+IndId+")";
				
					
//				String sql="INSERT INTO IndValue VALUES (2,7,now(),1)";
				//	int angl=ihm.ang();
				//	int mov=ihm.mov();
					int idscan=ihm.ids();
					idscanG=ihm.idsString();
					String idsG=idscanG;
					GetCurrentPosition(idsG);


				/*	double degrees=angl;
					double radians=Math.toRadians(degrees);
					int posX=(int) (Math.cos(radians)*mov);					
					int posY=(int) (Math.sin(radians)*mov);
					*/
					int posX=posXG;
					int posY=posYG;
					int angl=orientG;
					//					System.out.println("angle:"+angl+" move:"+mov+" id:"+idscan);
			//		angl=angl+orientP;
			//		posX=posX+posXP;
			//		posY=posY+posYP;
			//		int posY=0;
					String sql="INSERT INTO scanResult VALUES ("+idscan+",now(),"+posX+","+posY+","+angle+","+distFront+","+distBack+","+angl+")";
					//System.out.println("ind id "+IndIdS+", pos " + IndPos + ", len: " + IndLen+" value"+IndValue);
					System.out.println(sql);
					stmtI.executeUpdate(sql);

						
				} catch (Exception e) {
					e.printStackTrace();
				} finally {
					try { if (stmtR != null) stmtR.close(); } catch (SQLException e) { e.printStackTrace(); }
					try { if (stmtI != null) stmtI.close(); } catch (SQLException e) { e.printStackTrace(); }
					try { if (conn != null) conn.close(); } catch (SQLException e) { e.printStackTrace(); }
				}
				lastTrameNumber=sentence2[15];
}
		  		PanneaugraphiqueSonar.point(posXG,posYG,orientG,distFront,distBack,angle);
	  		ihm3.repaint();
				}
				if (sentence2[6]==0x65){                    // e echo
					EchoRobot.pendingEcho=0;
					System.out.print("echo response: ");
					ihm.MajRobotStat("connected");
					if (sentence2[8]==0x66){
						System.out.print("scan running ");
						ihm.MajRobotStat("scan running");
					}
					if (sentence2[8]==0x67){
						System.out.print("scan ended ");
						ihm.MajRobotStat("scan ended");
					}
					if (sentence2[8]==0x68){
						System.out.print("moving");
						ihm.MajRobotStat("moving");
						int ang=ihm.ang();
						int mov=ihm.mov();
						actStat=0x02;  
						ihm2.PosActualise(ang,mov);
			//			PanneauGraphique.point(150+posXG,150+posYG);
			//			graph.repaint();
					}
					if (sentence2[8]==0x69){
						if (actStat==0x01){   // on a rate le statut moving
							long ang=ihm.ang();
							long mov=ihm.mov();
							ihm2.PosActualise(ang,mov);
							actStat=0x02; 
						}
						if (actStat==0x02){ 
							ihm2.ValidePosition();
		//					PanneauGraphique.point(200+posXG/10,200+posYG/10);
		//					graph.repaint();
						}
						ihm.MajRobotStat("move ended");
						System.out.print("move ended");
						actStat=0x03;
					}
					String sb ;

					int sbn=sentence2[9];
					byte sbnb=(byte)sbn;
					StringBuffer sbb1 = new StringBuffer(2);
					sbb1.append( TAB_BYTE_HEX[(sbnb>>4) & 0xf] );
					sbb1.append( TAB_BYTE_HEX[(sbnb) & 0x0f] );
					sbn=sentence2[11];
					sbnb=(byte)sbn;
					StringBuffer sbb2 = new StringBuffer(2);
					sbb2.append( TAB_BYTE_HEX[(sbnb>>4) & 0xf] );
					sbb2.append( TAB_BYTE_HEX[(sbnb) & 0x0f] );
					sbn=sentence2[12];
					sbnb=(byte)sbn;
					StringBuffer sbb3 = new StringBuffer(2);
					sbb3.append( TAB_BYTE_HEX[(sbnb>>4) & 0xf] );
					sbb3.append( TAB_BYTE_HEX[(sbnb) & 0x0f] );
					StringBuffer sbb = new StringBuffer(15);
					sbn=sentence2[13];
					sbnb=(byte)sbn;
					StringBuffer sbb4 = new StringBuffer(2);
					sbb4.append( TAB_BYTE_HEX[(sbnb>>4) & 0xf] );
					sbb4.append( TAB_BYTE_HEX[(sbnb) & 0x0f] );
					ihm.MajRobotDiag("0x"+sbb1+" 0x"+sbb2+" 0x"+sbb3+" 0x"+sbb4);
					System.out.println(" diagPower:0x"+sbb1+"  Motors:0x"+sbb2+"  Connections:0x"+sbb3+"  Robot:0x"+sbb4);
					
					//
					int oct0=(byte)(sentence2[15]&0x7F)-(byte)(sentence2[15]&0x80); // manip car byte consideré signé
					int oct1=(byte)(sentence2[16]&0x7F)-(byte)(sentence2[16]&0x80);
					int posX=256*oct0+oct1;
					if (sentence2[14]==0x2d)
					{
						posX=-posX;
					}
					oct0=(byte)(sentence2[18]&0x7F)-(byte)(sentence2[18]&0x80); // manip car byte consideré signé
					oct1=(byte)(sentence2[19]&0x7F)-(byte)(sentence2[19]&0x80);
					int posY=256*oct0+oct1;
					if (sentence2[17]==0x2d)
					{
						posY=-posY;
					}
					oct0=(byte)(sentence2[21]&0x7F)-(byte)(sentence2[21]&0x80); // manip car byte consideré signé
					oct1=(byte)(sentence2[22]&0x7F)-(byte)(sentence2[22]&0x80);
					int alpha=256*oct0+oct1;
					if (sentence2[20]==0x2d)
					{
						alpha=-alpha;
					}
					System.out.println("posX:"+posX+ " posY:"+posY+" angle:"+ alpha);
				}
				if (sentence2[6]==0x66){                    // scan en cours
					System.out.println("scan running");
					ihm.MajRobotStat("scan running");

				}
				if (sentence2[6]==0x67){                    // scan en cours
					System.out.println("scan ended");
					ihm.MajRobotStat("scan ended");
				}
				if (sentence2[6]==0x68){                    // scan en cours
					System.out.println("moving");
					ihm.MajRobotStat("moving");
					int ang=ihm.ang();
					int mov=ihm.mov();
					ihm2.PosActualise(ang,mov);
					actStat=0x02;   // move en cours
				}
				if (sentence2[6]==0x69){                    // scan en cours
					if (actStat==0x01){   // on a rate le statut moving
						int ang=ihm.ang();
						int mov=ihm.mov();
						ihm2.PosActualise(ang,mov);
						actStat=0x02; 
					}
					System.out.println("move ended");
					ihm.MajRobotStat("move ended");
					if (actStat==0x02){ 
						ihm2.ValidePosition();
		//				PanneauGraphique.point(200+posXG/10,200+posYG/10);
		//				graph.repaint();
					}
					actStat=0x03;
				}
				if (sentence2[6]==0x70){  // power info
				int power1=sentence2[7]*10;
				if (power1<0)
				{
					power1=256-power1;
				}
				int power2=sentence2[8]*10;
				if (power2<0)
				{
					power2=256-power2;
				}
				int power3=sentence2[9]*10;
				if (power3<0)
				{
					power3=256-power3;
				}
//				System.out.println("power1: "+power1+"cV power2: "+power2+"cV power3: "+power3+"cV");
				ihm.MajRobotPower(Integer.toString(power1)+ "cV "+Integer.toString(power2)+ "cV "+Integer.toString(power3)+ "cV ");
				}
			   }
			
			}
			
		//   System.exit(0);
		 

			

public static void GetCurrentPosition(String ids) {
	Connection conn = null;
	Statement stmtR1 = null;
	Statement stmtI1 = null;
//		Fenetre ihm = new Fenetre();
//		JFormattedTextField ids=indScan;
//	String ids =indScan.getText();
	int indscan=Integer.parseInt(ids);

	  System.out.println("actualise position:"+indscan);
try {

Class.forName("com.mysql.jdbc.Driver").newInstance();
String connectionUrl = "jdbc:mysql://jserver:3306/robot";
String connectionUser = "jean";
String connectionPassword = "manu7890";
conn = DriverManager.getConnection(connectionUrl, connectionUser, connectionPassword);
//conn.setAutoCommit(false);
stmtR1 = conn.createStatement();
stmtI1 = conn.createStatement();
ResultSet rs = null;

rs = stmtR1.executeQuery("SELECT * FROM scanResult WHERE idscan = "+indscan+"  ORDER by time desc limit 1"); 
while (rs.next()) {
  posXG = rs.getInt("posX");
  posYG = rs.getInt("posY");
  orientG = rs.getInt("orientation");
  System.out.println("x:"+posXG+" Y:"+posYG+" orient:"+orientG);
}
rs.close();

} catch (Exception e) {
e.printStackTrace();
} 
finally {
try { if (stmtR1 != null) stmtR1.close(); } catch (SQLException e) { e.printStackTrace(); }
try { if (stmtI1 != null) stmtI1.close(); } catch (SQLException e) { e.printStackTrace(); }
try { if (conn != null) conn.close(); } catch (SQLException e) { e.printStackTrace(); }}
}
public static void  GetLastScanID() {
	Connection conn = null;
	Statement stmtR1 = null;
	Statement stmtI1 = null;
//		Fenetre ihm = new Fenetre();
//		JFormattedTextField ids=indScan;
//	String ids =indScan.getText();
//	int indscan=Integer.parseInt(ids);

//	  System.out.println("actualise position:"+indscan);
try {

Class.forName("com.mysql.jdbc.Driver").newInstance();
String connectionUrl = "jdbc:mysql://jserver:3306/robot";
String connectionUser = "jean";
String connectionPassword = "manu7890";
conn = DriverManager.getConnection(connectionUrl, connectionUser, connectionPassword);
//conn.setAutoCommit(false);
stmtR1 = conn.createStatement();
stmtI1 = conn.createStatement();
ResultSet rs = null;

rs = stmtR1.executeQuery("SELECT * FROM scanResult  ORDER by time desc limit 1"); 
while (rs.next()) {
  idscanG = rs.getString("idscan");
  posXG = rs.getInt("posX");
  posYG = rs.getInt("posY");
  orientG = rs.getInt("orientation");
  System.out.println("x:"+posXG+" Y:"+posYG+" orient:"+orientG);
}
rs.close();

} catch (Exception e) {
e.printStackTrace();
} 
finally {
try { if (stmtR1 != null) stmtR1.close(); } catch (SQLException e) { e.printStackTrace(); }
try { if (stmtI1 != null) stmtI1.close(); } catch (SQLException e) { e.printStackTrace(); }
try { if (conn != null) conn.close(); } catch (SQLException e) { e.printStackTrace(); }}

}
/*public void RefreshAff() {
	// TODO Auto-generated method stub
    ihm.RefreshStat();
	}
	}
//   System.exit(0);
  */
public static void  UpdateGraphRobotLocation() {
	}

static void hexaPrint(byte y)
{

		System.out.print("-" + String.format("%x", y));
	}
}
