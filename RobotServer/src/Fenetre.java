import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.text.NumberFormat;
import java.io.UnsupportedEncodingException;
import java.net.*;
import java.sql.*;

import javax.swing.JButton;
import javax.swing.JFormattedTextField;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextField;
 
public class Fenetre extends JFrame{
 
  private Panneau pan = new Panneau();
  private JButton boutonStart = new JButton("Start");
  private JButton boutonStop = new JButton("Stop");
  private JButton boutonScan = new JButton("Scan");
  private JButton boutonMove = new JButton("Move");
  private JButton boutonInit = new JButton("Init");
  private JButton boutonRefresh = new JButton("Refresh");
  private JButton boutonAffEcho = new JButton("Aff Echo");
  private JPanel container = new JPanel();
  private JLabel label = new JLabel("Angle ° - Deplact en mn - Id2 du scan ");
  private JLabel label2 = new JLabel("Action >> ");
  private JFormattedTextField  angle = new JFormattedTextField(NumberFormat.getIntegerInstance());
  private JFormattedTextField  move = new JFormattedTextField(NumberFormat.getIntegerInstance());
  private JFormattedTextField  orient= new JFormattedTextField(NumberFormat.getIntegerInstance());
  private JFormattedTextField  idscan = new JFormattedTextField(NumberFormat.getIntegerInstance());
 // private JTextField move = new JTextField("0");
  private int compteur = 0;
  public int movIHM;
// 

//public int ang;
//public int mov;
//public int ids;
  public String robotStat=" ?";
  public String robotPower= " ?";
  public String robotDiag;
  public Fenetre(){
    this.setTitle("Fonction de base");
    this.setSize(600, 200);
    this.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
    this.setLocationRelativeTo(null);
    container.setBackground(Color.white);
    container.setLayout(new BorderLayout());
  //  container.add(pan, BorderLayout.CENTER);
        
    //Ce sont maintenant nos classes internes qui écoutent nos boutons 
    boutonInit.addActionListener(new BoutonInitListener()); 
    boutonStart.addActionListener(new BoutonStartListener());
    boutonStop.addActionListener(new BoutonStopListener());
    boutonScan.addActionListener(new BoutonScanListener()); 
    boutonMove.addActionListener(new BoutonMoveListener()); 
    boutonRefresh.addActionListener(new BoutonRefreshListener()); 
    boutonAffEcho.addActionListener(new BoutonAffEchoListener()); 
//  boutonInit.setBackground(Color.black);
//    boutonInit.setForeground(Color.white);
    JPanel south = new JPanel();
    JPanel top = new JPanel();
 //   top.setBackground(Color.green);
    south.setBackground(Color.gray);
 //   top.setLocation(20, 20);
  //  top.setSize(400, 200);
    Font police = new Font("Tahoma", Font.BOLD, 16);
    angle.setFont(police);
    angle.setPreferredSize(new Dimension(50, 30));
    angle.setForeground(Color.BLUE);
 //   angle.setLocation(200, 200);
    angle.setText("0");
    move.setFont(police);
    move.setPreferredSize(new Dimension(50, 30));
    move.setForeground(Color.BLUE);
    move.setText("0");
    idscan.setFont(police);
   idscan.setPreferredSize(new Dimension(50, 30));
//    idscan.setForeground(Color.BLUE);
    idscan.setForeground(Color.RED);
//    idscan.setBackground(Color.black);
    idscan.setText(RobotMainServer.idscanG);
    orient.setFont(police);
    orient.setPreferredSize(new Dimension(50, 30));
    orient.setForeground(Color.RED);
//    orient.setBackground(Color.black);
    String orientGS=""+RobotMainServer.orientG;
    orient.setText(orientGS);
//    container.add(label);
 //   south.add(label);
    boutonInit.setForeground(Color.RED);
    top.add(label2);
    top.add(angle);
    top.add(move);
    top.add(idscan);
    top.add(orient);
    label2.setHorizontalAlignment(JLabel.CENTER);
    south.add(label2);
    south.add(boutonStart);
    south.add(boutonStop);
    south.add(boutonInit);
    south.add(boutonRefresh);
    south.add(boutonScan);
    south.add(boutonMove);
    south.add(boutonAffEcho);
  

    label.setFont(police);
    label.setForeground(Color.blue);
    label.setHorizontalAlignment(JLabel.CENTER);
    container.add(top, BorderLayout.CENTER);
    container.add(south, BorderLayout.SOUTH);
    container.add(label, BorderLayout.NORTH);
    this.setContentPane(container);
    this.setVisible(true);

//    ang = Integer.parseInt(angle.getText());
 //   mov = Integer.parseInt(move.getText());
  //  ids = Integer.parseInt(idscan.getText());
    go();
  }
      
  private void go(){
      label.setText(RobotMainServer.stationStatus+" "+robotStat+"-"+robotPower+ "-"+robotDiag);   
     RobotMainServer.idscanG= idscan.getText();
    //Cette méthode ne change pas
  }
  class BoutonAffEchoListener implements ActionListener{
	    //Redéfinition de la méthode actionPerformed()
	    public void actionPerformed(ActionEvent arg0) {
			FenetreEchoLocation ihm4 = new FenetreEchoLocation();
			ihm4.SetInitialPosition();
			ihm4.repaint();
	      go();
	    }
	  }
  class BoutonRefreshListener implements ActionListener{
	    //Redéfinition de la méthode actionPerformed()
	    public void actionPerformed(ActionEvent arg0) {
	    	if (EchoRobot.pendingEcho>1)
	    	{
	    		robotStat="timout";
	    	}
	      go();
	    }
	  }

  class BoutonStartListener implements ActionListener{
    //Redéfinition de la méthode actionPerformed()
    public void actionPerformed(ActionEvent arg0) {
        RobotMainServer.idscanG= idscan.getText();
      label.setText("Demarrage du robot");   
      SendUDP snd = new SendUDP();
      snd.SendUDPStart();
      go();
    }
  }
      
  //Classe écoutant notre second bouton
  class BoutonStopListener implements ActionListener{

    public void actionPerformed(ActionEvent e) {
        RobotMainServer.idscanG= idscan.getText();
      label.setText("Arret du robot");
      SendUDP snd = new SendUDP();
      snd.SendUDPStop();
      go();
    }
  } 
  class BoutonScanListener implements ActionListener{
	    //Redéfinition de la méthode actionPerformed()
	    public void actionPerformed(ActionEvent arg0) {
	        RobotMainServer.idscanG= idscan.getText();
	      label.setText("Demarrage du scan");   

	      SendUDP snd = new SendUDP();
	      snd.SendUDPScan();
	      go();
	    }
	  }
  class BoutonMoveListener implements ActionListener{
	    //Redéfinition de la méthode actionPerformed()
	    public void actionPerformed(ActionEvent arg0) {
	        RobotMainServer.idscanG= idscan.getText();
	      label.setText("Move");   
//	      System.out.println("angle " + angle.getText());
//	      System.out.println("move " + move.getText());
//	      int ang = Integer.parseInt(angle.getText());
//	      String moveT=move.getText();
//	      Object mov = move.getValue();
	      Object ang = angle.getValue();
	      Object mov = move.getValue();
//	      System.out.println("angle2 " + ang);
	      if (!mov.equals(null) || !ang.equals(null))
	      {
	      SendUDP snd = new SendUDP();
	      snd.SendUDPMove((long)ang,(long) mov);
	      }
//	      Fenetre2 f2 = new Fenetre2();
//	      f2.SetcurrentInd(idscan.getText());
//	      f2.SetInitialPosition();
//	      f2.SetMove(move.getText(), angle.getText());
	      RobotMainServer.actStat=0x01;  //demande mov
	      go();

	    }
	  }
  class BoutonInitListener implements ActionListener{
	    //Redéfinition de la méthode actionPerformed()
	    public void actionPerformed(ActionEvent arg0) {
	        RobotMainServer.idscanG= idscan.getText();
	      label.setText("Init posX, posY, orientation");   
//	      System.out.println("posX " + angle.getText());
//	      System.out.println("posY " + move.getText());
//	      System.out.println("idscan " + idscan.getText());
	      int posX = Integer.parseInt(angle.getText());
	      int posY= Integer.parseInt(move.getText());
	      int ids= Integer.parseInt(idscan.getText());
	      int orien= Integer.parseInt(orient.getText());
	      InitPos initRobot = new InitPos();
	      initRobot.InitRobot(posX, posY, orien, ids);
	      SendUDP snd = new SendUDP();
	      snd.SendUDPInit();
	      go();
	    }
	  }
public int ang() {
	// TODO Auto-generated method stub
    int ang = Integer.parseInt(angle.getText());
	return ang;
}
public int ids() {
	// TODO Auto-generated method stub
    int ids = Integer.parseInt(idscan.getText());
	return ids;
}
public int mov() {
	// TODO Auto-generated method stub
	String nbS=move.getText();
	/*
	byte[] nbB = null;
	try {
		nbB = nbS.getBytes(nbS);
	} catch (UnsupportedEncodingException e) {
		// TODO Auto-generated catch block
		e.printStackTrace();
	}
	int i=0;
	for (i=0;i<3;i++)
	{
	 RobotMainServer.hexaPrint(nbB[i]);
	}
	*/
	String nb=nbS.replaceAll("[\\p{Cc}\\p{Cf}\\p{Co}\\p{Cn}]","");

	 int mov=0;
	try{
     mov = Integer.parseInt(nb);
	}
	catch (Exception e) {
		e.printStackTrace();
		} 
	finally
	{
		
	}
	return mov;
}
public String idsString() {
	// TODO Auto-generated method stub
    String ids = idscan.getText();
	return ids;
}
public void MajRobotStat(String mess) {
	// TODO Auto-generated method stub
	robotStat=mess;
    go();

}
public void MajRobotPower(String power1) {
	// TODO Auto-generated method stub
	robotPower=power1;
    go();

}
public void MajRobotDiag(String string) {
	// TODO Auto-generated method stub
	robotDiag=string;
    go();

}
public void RefreshStat() {
	// TODO Auto-generated method stub

    go();
}
}
