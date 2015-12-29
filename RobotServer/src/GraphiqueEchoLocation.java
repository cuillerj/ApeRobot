
import java.awt.Color;
import java.awt.Font;
import java.awt.Graphics;
import java.sql.Connection;
import java.sql.DriverManager;
import java.sql.ResultSet;
import java.sql.SQLException;
import java.sql.Statement;

import javax.swing.JFrame;
import javax.swing.JPanel;
 
public class GraphiqueEchoLocation extends JPanel { 
	public static int ptXFront=0;
	public static int ptYFront=0;
	public static int ptXBack=0;
	public static int ptYBack=0;
	public static int refAngleSonar=(int) Math.toRadians(65); // le sonar est aligne avec le robot a 75
	public static int posX;
	public static int posY;
	public static int angle;
	public static int distFront;
	public static int distBack;
	public static int orientation;
  public void paintComponent(Graphics g){
   
		Connection conn = null;
		Statement stmtR1 = null;
		Statement stmtI1 = null;

		int indscan= Integer.parseInt(RobotMainServer.idscanG);

//		  System.out.println("graphique echo location execute");
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
    g.setColor(Color.black);
    g.drawLine(this.getWidth()/2,0,this.getWidth()/2,this.getHeight());
    g.drawLine(this.getWidth()/4,0,this.getWidth()/4,this.getHeight());
    g.drawLine(this.getWidth()*3/4,0,this.getWidth()*3/4,this.getHeight());
        g.drawLine(0,this.getHeight()/2 ,this.getWidth(), this.getHeight()/2);
        g.drawLine(0,this.getHeight()/4 ,this.getWidth(), this.getHeight()/4);
        g.drawLine(0,this.getHeight()*3/4 ,this.getWidth(), this.getHeight()*3/4);
        Font font = new Font("Courier", Font.BOLD, 20);
        g.setFont(font);
        g.drawString("X",this.getHeight()-10,this.getHeight()/2 );
        g.drawString("Y", this.getWidth()/2,this.getWidth()-30);
	rs = stmtR1.executeQuery("SELECT * FROM scanResult WHERE idscan = "+indscan+"  ORDER by time desc "); 
	int nbPt=0;
	while (rs.next()) {
	  posX = rs.getInt("posX");
	  posY = rs.getInt("posY");
	  angle = rs.getInt("angle");
	  distFront = rs.getInt("distFront");
	  distBack = rs.getInt("distBack");
	  orientation = rs.getInt("orientation");
//	  System.out.println("idscan:"+indscan+" x:"+posX+" Y:"+posY+" angle:"+angle+" distfront:"+distFront+" disback:"+distBack+" orient:"+orientation);
	    ptXFront=(int) (posX/10+distFront*Math.cos(Math.toRadians(orientation)+Math.toRadians(angle)-refAngleSonar));
	    ptYFront=(int) (posY/10+distFront*Math.sin(Math.toRadians(orientation)+Math.toRadians(angle)-refAngleSonar));
	    ptXBack=(int) (posX/10+distBack*Math.cos(Math.toRadians(orientation)+Math.toRadians(angle)+Math.PI-refAngleSonar));
	    ptYBack=(int) (posY/10+distBack*Math.sin(Math.toRadians(orientation)+Math.toRadians(angle)+Math.PI-refAngleSonar));
	    int center[]={this.getWidth()/2,this.getHeight()/2};
	    System.out.println("center:"+center[0]+" "+center[1]);
	    int ptXrefFront=ptXFront+this.getWidth()/2;
	    int ptYrefFront=ptYFront+this.getHeight()/2;
	    int ptXrefBack=ptXBack+this.getWidth()/2;
	    int ptYrefBack=ptYBack+this.getHeight()/2;
	    int dimpt=5;  // taille di point a dessiner
//	    System.out.println("graph echo Je suis exécutéex: front X"+ptXrefFront+" Y;"+ptYrefFront+" back X"+ptXrefBack+" Y;"+ptYrefBack); 
//	    double deltaX=15+5*Math.cos(radian);
//	    double deltaY=10+5*Math.sin(radian);
//	    System.out.println(deltaX+"<->"+deltaY);

//	    alpha=0;
//	    int x0[]={ 0,20,30,20,0};
//	    int y0[]={ 20,20,10,0,0};
//	    int x0[]={ (int) (posXnRef+h*Math.cos(radian)),(int) (posXnRef+l*Math.sin(radian)),(int) (posXnRef+diag*Math.cos(gamma)),(int) (posXnRef+diag*Math.cos(beta)),(int) (posXnRef-l*Math.sin(radian))};
//	    int y0[]={ (int) (posYnRef+h*Math.sin(radian)),(int) (posYnRef-l*Math.cos(radian)),(int) (posYnRef+diag*Math.sin(gamma)),(int) (posYnRef+diag*Math.sin(beta)),(int) (posYnRef+l*Math.cos(radian))};

	    if (distFront!=0){
	    g.setColor(Color.orange);
	    g.fillOval(ptXrefFront-dimpt/2,ptYrefFront-dimpt/2,dimpt,dimpt);
	    nbPt=nbPt+1;
	    }

	
	    if (distBack!=0 && distBack!=7){
	    g.setColor(Color.red);
	    g.fillOval(ptXrefBack-dimpt/2,ptYrefBack-dimpt/2,dimpt,dimpt);
	    nbPt=nbPt+1;
	    }
	

	    System.out.println("nb point:"+nbPt);
//	    int x[]={(int) (center[0]+posX-deltaX),(int) (center[0]+posX),(int) (center[0]+posX+deltaPx),(int) (center[0]+posX),(int) (center[0]+posX-deltaX)};
//	    int y[]={(int) (center[1]+posY+deltaY),(int) (center[1]+posY+deltaY),(int) (center[1]+posY+deltaPy),(int) (center[1]+posY-deltaY),(int) (center[1]+posY-deltaY)};
//	    for (int i=0;i<=4;i++){
//	    	System.out.println((x[i]-center[0])+","+(y[i]-center[1]));
//	    }
//	    g.fillPolygon(x, y,5);
	   // g.fillRect(500+, 500+posY, 30, 20);

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
    

  
}
