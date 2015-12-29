import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.text.NumberFormat;
import java.sql.*;

import javax.swing.JButton;
import javax.swing.JFormattedTextField;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextField;


public class Fenetre2 extends JFrame{
//	private Panneau pan = new Panneau();
//	  private JButton boutonActualise = new JButton("Calcul");
//	  private JButton boutonValide = new JButton("Valide");
//	  private JButton boutonInit = new JButton("Initialise position");
	 FenetreGraphique graph = new FenetreGraphique();
	  private JPanel container = new JPanel();
	//  private JPanel container2 = new JPanel();
	  private JLabel label = new JLabel("Position du robot ");
	  
	  private JFormattedTextField  posX= new JFormattedTextField(NumberFormat.getIntegerInstance());
	  private JFormattedTextField  posY = new JFormattedTextField(NumberFormat.getIntegerInstance());
	  private JFormattedTextField  orient= new JFormattedTextField(NumberFormat.getIntegerInstance());
	  private JFormattedTextField  indScan= new JFormattedTextField(NumberFormat.getIntegerInstance());
	  private JFormattedTextField  posXP= new JFormattedTextField(NumberFormat.getIntegerInstance());
	  private JFormattedTextField  posYP= new JFormattedTextField(NumberFormat.getIntegerInstance());
	  private JFormattedTextField  orientP= new JFormattedTextField(NumberFormat.getIntegerInstance());
	 // private JTextField move = new JTextField("0");
	  private int compteur = 0;
	//public int ang;
	//public int mov;
	//public int ids;
	  public int mov;
	  public int ang;
	  public Fenetre2(){
	    this.setTitle("Statut du robot");
	    this.setSize(250, 150);
	    this.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
	    this.setLocationRelativeTo(null);
	    this.setLocation(200,50);
	    container.setBackground(Color.white);
	    container.setLayout(new BorderLayout());
	  //  container.add(pan, BorderLayout.CENTER);
	        
	    //Ce sont maintenant nos classes internes qui écoutent nos boutons 
//	    boutonActualise.addActionListener(new BoutonActualise());
//	    boutonValide.addActionListener(new BoutonValide());
	    JPanel south = new JPanel();
	    JPanel top = new JPanel();
	    JPanel center = new JPanel();
	    Font police = new Font("Tahoma", Font.BOLD, 16);
	    posX.setFont(police);
	    posX.setPreferredSize(new Dimension(50, 30));
	    posX.setForeground(Color.BLUE);
	    posY.setFont(police);
	    posY.setPreferredSize(new Dimension(50, 30));
	    posY.setForeground(Color.BLUE);
	    orient.setFont(police);
	    orient.setPreferredSize(new Dimension(50, 30));
	    orient.setForeground(Color.BLUE);
	    indScan.setFont(police);
	    indScan.setPreferredSize(new Dimension(50, 30));
	    indScan.setForeground(Color.RED);
//	    indScan.setBackground(Color.black);
	    indScan.setText(RobotMainServer.idscanG);
	//    indScan.setLocation(300, 300);
	    posXP.setFont(police);
	    posXP.setPreferredSize(new Dimension(50, 30));
	    posXP.setForeground(Color.BLUE);
	    String posXPS=""+RobotMainServer.posXG;
	    posXP.setText(posXPS);
	    posYP.setFont(police);
	    posYP.setPreferredSize(new Dimension(50, 30));
	    posYP.setForeground(Color.BLUE);
	    String posYPS=""+RobotMainServer.posYG;
	    posYP.setText(posYPS);
	    orientP.setFont(police);
	    orientP.setPreferredSize(new Dimension(50, 30));
	    orientP.setForeground(Color.BLUE);
	    String orientPS=""+RobotMainServer.orientG;
	    orientP.setText(orientPS);
	    container.add(label);
	    top.add(posX);
	    top.add(label);
	    top.add(posY);
	    top.add(orient);
	    top.add(indScan);
	    top.add(posXP);
	    top.add(posYP);
	    top.add(orientP);
	    container.add(top, BorderLayout.CENTER);
//	    container.add(center, BorderLayout.CENTER);
	//    south.add(boutonActualise);
	//    south.add(boutonValide);

	    container.add(south, BorderLayout.SOUTH);

	    label.setFont(police);
	    label.setForeground(Color.blue);
	    label.setHorizontalAlignment(JLabel.CENTER);
	    container.add(label, BorderLayout.NORTH);
	    this.setContentPane(container);
	    this.setVisible(true);

	    int posX1= Integer.parseInt(posXP.getText());
	    int posY1= Integer.parseInt(posYP.getText());
	  int orient1= Integer.parseInt(orientP.getText());
  		PanneauGraphique.point(posX1,posY1,orient1);
  		graph.repaint();

	  }
	  /*
	
	  */
	  public int GetcurrentPosX() {
			// TODO Auto-generated method stub
		    int ang = Integer.parseInt(posX.getText());
			return ang;
		}
	  public int GetcurrentPosY() {
			// TODO Auto-generated method stub
		    int ang = Integer.parseInt(posY.getText());
			return ang;
		}
	  public int GetcurrentOrient() {
			// TODO Auto-generated method stub
		    int ang = Integer.parseInt(orient.getText());
			return ang;
		}

	
	public void SetcurrentPosX(String text) {
		// TODO Auto-generated method stub
	    posXP.setText(text);	
	}
	public void SetcurrentPosY(String text) {
		// TODO Auto-generated method stub
	    posYP.setText(text);	
	}
	public void SetcurrentOrientation(String text) {
		// TODO Auto-generated method stub
	    orientP.setText(text);	
	}
	public void SetnextPosX(String text) {
		// TODO Auto-generated method stub
	    posX.setText(text);	
	}
	public void SetnextPosY(String text) {
		// TODO Auto-generated method stub
	    posY.setText(text);	
	}
	public void SetnextOrientation(String text) {
		// TODO Auto-generated method stub
	    orient.setText(text);	
	}
	public void SetcurrentInd(String text) {
		// TODO Auto-generated method stub
	    indScan.setText(text);	
	}
	public void SetMove(String move, String angle) {
		// TODO Auto-generated method stub
	    mov=Integer.parseInt(move);	
	    ang=Integer.parseInt(angle);	
	    System.out.println("move:"+mov+" "+ang);
	}
	public void SetInitialPosition() {
	Connection conn = null;
	Statement stmtR1 = null;
	Statement stmtI1 = null;
//		Fenetre ihm = new Fenetre();
//		JFormattedTextField ids=indScan;
//	String ids =indScan.getText();
//	int indscan=Integer.parseInt(ids);
    indScan.setText(RobotMainServer.idscanG);
    int indscan=Integer.parseInt(RobotMainServer.idscanG);
	String posXP;
	String posYP;
	String orientP;
	  System.out.println("actualise:"+indscan);
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
int ndscan=Integer.parseInt(RobotMainServer.idscanG);
rs = stmtR1.executeQuery("SELECT * FROM scanResult WHERE idscan = "+indscan+"  ORDER by time desc limit 1"); 
while (rs.next()) {
  posXP = rs.getString("posX");
  posYP = rs.getString("posY");
  orientP = rs.getString("orientation");
  SetcurrentPosX( posXP);
  SetcurrentPosY( posYP);
  SetcurrentOrientation( orientP);
  System.out.println("x:"+posXP+" Y:"+posYP+" orient:"+orientP);
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
	public void TargetLocation(long ang2,long mov2, int posX,int posY,long orient){
		System.out.println("target location"+ang2+mov2+posX+posY+orient);
	orient=(orient+ang2)%360;
	String orientnext=""+orient;
	SetnextOrientation(orientnext);
	double degrees=orient;
	double radians=Math.toRadians(degrees);
	int posXnext=(int) (Math.cos(radians)*mov2);					
	int posYnext=(int) (Math.sin(radians)*mov2);
	posXnext=posXnext+posX;
	posYnext=posYnext+posY;
	String posXn=""+posXnext;
	SetnextPosX(posXn);
	String posYn=""+posYnext;
	SetnextPosY(posYn);
	}
	public void PosActualise(long ang2,long mov2) {
		// TODO Auto-generated method stub
		SetInitialPosition();
	    indScan.setText(RobotMainServer.idscanG);
	 	String ss =orientP.getText();
//	 	System.out.println("actualise "+ang2+" "+mov2 );
	  	int orient=Integer.parseInt(ss);
	    	ss =posXP.getText();
	    	System.out.println(ss);
	     int posXP=Integer.parseInt(ss);
	    	ss =posYP.getText();
	    	System.out.println(ss);
	    	int posYP=Integer.parseInt(ss);
	    	TargetLocation(ang2,mov2,posXP,posYP,orient);
	}
	public void ValidePosition() {
	      label.setText("Init posX, posY, orientation");   
		 	String ss =orient.getText();
		      SetcurrentOrientation(ss);
	//	 	System.out.println(ss);
		  	int orientN=Integer.parseInt(ss);
		    	ss =posX.getText();
			      SetcurrentPosX(ss);
		    	System.out.println(ss);
		     int posXN=Integer.parseInt(ss);
		    	ss =posY.getText();
			      SetcurrentPosY(ss);
		    	System.out.println(ss);
		    	int posYN=Integer.parseInt(ss);
		    	ss =indScan.getText();
		    	ss=RobotMainServer.idscanG;
		    	System.out.println(ss);
		    	int indsN=Integer.parseInt(ss);
	      InitPos initRobot = new InitPos();
	      initRobot.InitRobot(posXN, posYN, orientN, indsN);
	  		PanneauGraphique.point(posXN,posYN,orientN);
	  		graph.repaint();

}
	public void Refresh() {
		
	}
	
}