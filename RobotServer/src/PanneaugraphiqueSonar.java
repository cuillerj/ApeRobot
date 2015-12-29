
import java.awt.Color;
import java.awt.Font;
import java.awt.Graphics;

import javax.swing.JFrame;
import javax.swing.JPanel;
 
public class PanneaugraphiqueSonar extends JPanel { 
	public static int ptXFront=0;
	public static int ptYFront=0;
	public static int ptXBack=0;
	public static int ptYBack=0;
	public static int refAngleSonar=(int) Math.toRadians(65); // le sonar est aligne avec le robot a 75
  public void paintComponent(Graphics g){
    //Vous verrez cette phrase chaque fois que la méthode sera invoquée

    int center[]={this.getWidth()/2,this.getHeight()/2};
    System.out.println("center:"+center[0]+" "+center[1]);
    int ptXrefFront=ptXFront+this.getWidth()/2;
    int ptYrefFront=ptYFront+this.getHeight()/2;
    int ptXrefBack=ptXBack+this.getWidth()/2;
    int ptYrefBack=ptYBack+this.getHeight()/2;
    int dimpt=10;  // taille di point a dessiner
    System.out.println("panneau sonar Je suis exécutéex: front X"+ptXrefFront+" Y;"+ptYrefFront+" back X"+ptXrefBack+" Y;"+ptYrefBack); 
//    double deltaX=15+5*Math.cos(radian);
//    double deltaY=10+5*Math.sin(radian);
//    System.out.println(deltaX+"<->"+deltaY);

//    alpha=0;
//    int x0[]={ 0,20,30,20,0};
//    int y0[]={ 20,20,10,0,0};
//    int x0[]={ (int) (posXnRef+h*Math.cos(radian)),(int) (posXnRef+l*Math.sin(radian)),(int) (posXnRef+diag*Math.cos(gamma)),(int) (posXnRef+diag*Math.cos(beta)),(int) (posXnRef-l*Math.sin(radian))};
//    int y0[]={ (int) (posYnRef+h*Math.sin(radian)),(int) (posYnRef-l*Math.cos(radian)),(int) (posYnRef+diag*Math.sin(gamma)),(int) (posYnRef+diag*Math.sin(beta)),(int) (posYnRef+l*Math.cos(radian))};
    g.setColor(Color.orange);
    g.fillOval(ptXrefFront-dimpt/2,ptYrefFront-dimpt/2,dimpt,dimpt);
    g.setColor(Color.red);
    g.fillOval(ptXrefBack-dimpt/2,ptYrefBack-dimpt/2,dimpt,dimpt);


//    int x[]={(int) (center[0]+posX-deltaX),(int) (center[0]+posX),(int) (center[0]+posX+deltaPx),(int) (center[0]+posX),(int) (center[0]+posX-deltaX)};
//    int y[]={(int) (center[1]+posY+deltaY),(int) (center[1]+posY+deltaY),(int) (center[1]+posY+deltaPy),(int) (center[1]+posY-deltaY),(int) (center[1]+posY-deltaY)};
//    for (int i=0;i<=4;i++){
//    	System.out.println((x[i]-center[0])+","+(y[i]-center[1]));
//    }
//    g.fillPolygon(x, y,5);
   // g.fillRect(500+, 500+posY, 30, 20);

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
//        Font font2 = new Font("Courier", Font.BOLD, 12);
//        g.setFont(font2);
 //       g.drawString("v", this.getWidth()/2,this.getWidth()-40);
 //       g.drawString("v", this.getWidth()/2,this.getWidth()-30);
      }               
    

  public static void point(int posXi,int posYi, int orient, int distFront,int distBack, int angle){
	    //Vous verrez cette phrase chaque fois que la méthode sera invoquée
	    System.out.println("point Je suis appele x:"+posXi+" Y;"+posYi+ " orient:"+ orient+" distfront:"+distFront+ " distback:"+distBack);
	    ptXFront=(int) (posXi/10+distFront*Math.cos(Math.toRadians(orient)+Math.toRadians(angle)-refAngleSonar));
	    ptYFront=(int) (posYi/10+distFront*Math.sin(Math.toRadians(orient)+Math.toRadians(angle)-refAngleSonar));
	    ptXBack=(int) (posXi/10+distBack*Math.cos(Math.toRadians(orient)+Math.toRadians(angle)+Math.PI-refAngleSonar));
	    ptYBack=(int) (posYi/10+distBack*Math.sin(Math.toRadians(orient)+Math.toRadians(angle)+Math.PI-refAngleSonar));
	//   pointComponent(null);
	  }
 
}
