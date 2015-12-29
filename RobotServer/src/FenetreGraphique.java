import javax.swing.JFrame;

public class FenetreGraphique extends JFrame {
 public FenetreGraphique(){   
   this.setTitle("Mouvement du robot");
   this.setSize(1000, 1000);
   this.setLocation(150,00);               
   this.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
   this.setContentPane(new PanneauGraphique());
   this.setVisible(true);
//   System.out.println("fenetre Je suis exécutée !");
 }


public void SetInitialPosition() {
	// TODO Auto-generated method stub
	
}     
public void SetPoint() {
	// TODO Auto-generated method stub

}     
}               

