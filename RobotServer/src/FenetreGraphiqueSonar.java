import javax.swing.JFrame;

public class FenetreGraphiqueSonar extends JFrame {
 public FenetreGraphiqueSonar(){   
   this.setTitle("Echo location");
   this.setSize(1000, 1000);
   this.setLocation(150,00);               
   this.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
   this.setContentPane(new PanneaugraphiqueSonar());
   this.setVisible(true);
//   System.out.println("fenetre echo loc Je suis exécutée !");
 }


public void SetInitialPosition() {
	// TODO Auto-generated method stub
	
}     
public void SetPoint() {
	// TODO Auto-generated method stub

}     
}               


