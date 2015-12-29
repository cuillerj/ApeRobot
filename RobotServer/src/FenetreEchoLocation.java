
import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.text.NumberFormat;
import java.net.*;
import java.sql.*;

import javax.swing.JButton;
import javax.swing.JFormattedTextField;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextField;
 
public class FenetreEchoLocation extends JFrame{
	public static int idscanI=0;
	
  private GraphiqueEchoLocation pan = new GraphiqueEchoLocation();
  private JButton boutonRefresh = new JButton("Refresh");
  private JPanel container = new JPanel();
  private JLabel label = new JLabel("Graph echo location");
  private JLabel label2 = new JLabel("Action >> ");
  private JFormattedTextField  idscan = new JFormattedTextField(NumberFormat.getIntegerInstance());

  public FenetreEchoLocation(){
    this.setTitle("Affichage");
    this.setSize(1000, 1000);
//    this.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
    this.setLocationRelativeTo(null);
    container.setBackground(Color.white);
    container.setLayout(new BorderLayout());
    container.add(pan, BorderLayout.CENTER);
    boutonRefresh.addActionListener(new BoutonRefreshListener()); 
    JPanel south = new JPanel();
    JPanel top = new JPanel();
 //   top.setBackground(Color.green);
    south.setBackground(Color.gray);
 //   top.setLocation(20, 20);
  //  top.setSize(400, 200);
    Font police = new Font("Tahoma", Font.BOLD, 16);
 //   angle.setLocation(200, 200);
    idscan.setFont(police);
   idscan.setPreferredSize(new Dimension(50, 30));
//    idscan.setForeground(Color.BLUE);
    idscan.setForeground(Color.RED);
//    idscan.setBackground(Color.black);
    idscan.setText(RobotMainServer.idscanG);
   //  top.add(label2);

    south.add(idscan);

    label2.setHorizontalAlignment(JLabel.CENTER);
    south.add(label2);

    south.add(boutonRefresh);
  

    label.setFont(police);
    label.setForeground(Color.blue);
    label.setHorizontalAlignment(JLabel.CENTER);
   // container.add(top, BorderLayout.CENTER);
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

     RobotMainServer.idscanG= idscan.getText();
    //Cette méthode ne change pas
  }
  class BoutonRefreshListener implements ActionListener{
	    //Redéfinition de la méthode actionPerformed()
	    public void actionPerformed(ActionEvent arg0) {
	      go();
	    }
	  }
  //Classe écoutant notre premier bouton
  
 

public int ids() {
	// TODO Auto-generated method stub
    int ids = Integer.parseInt(idscan.getText());
	return ids;
}

public void RefreshStat() {
	// TODO Auto-generated method stub

    go();
}

public void SetInitialPosition() {
	// TODO Auto-generated method stub
	
}
}
