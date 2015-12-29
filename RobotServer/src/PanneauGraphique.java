

	import java.awt.Font;
import java.awt.Graphics;

	import javax.swing.JFrame;
import javax.swing.JPanel;
	 
	public class PanneauGraphique extends JPanel { 
		public static int posX=0;
		public static int posY=0;
		public static double orient=0;
	  public void paintComponent(Graphics g){
	    //Vous verrez cette phrase chaque fois que la méthode sera invoquée
	    System.out.println("panneau Je suis exécutéex:"+posX+" Y;"+posY); 
	    int center[]={this.getWidth()/2,this.getHeight()/2};
	    System.out.println("center:"+center[0]+" "+center[1]);
	    double radian=Math.toRadians(orient);
	    double echelle=400/1000;  // 400cm pour 1000 pts
//	    double deltaX=15+5*Math.cos(radian);
//	    double deltaY=10+5*Math.sin(radian);
//	    System.out.println(deltaX+"<->"+deltaY);
	    double deltaPx=20;
	    double deltaPy=0;
	    double l=15;
	    double L=30;
	    double h=10;
	    double alpha=Math.atan(L/l);
	    double diag=Math.sqrt(L*L+l*l);
	    double beta=radian+alpha+(Math.PI)/2;
	    double gamma=radian+(Math.PI)*3/2-alpha;
	    //double posXnRef=center[0]+posX*echelle;
	    double posXnRef=center[0]+posX/4;
	    double posYnRef=center[1]+posY/4;
	    int x0[]={ (int) (posXnRef+h*Math.cos(radian)),(int) (posXnRef+l*Math.sin(radian)),(int) (posXnRef+diag*Math.cos(gamma)),(int) (posXnRef+diag*Math.cos(beta)),(int) (posXnRef-l*Math.sin(radian))};
	    int y0[]={ (int) (posYnRef+h*Math.sin(radian)),(int) (posYnRef-l*Math.cos(radian)),(int) (posYnRef+diag*Math.sin(gamma)),(int) (posYnRef+diag*Math.sin(beta)),(int) (posYnRef+l*Math.cos(radian))};
	    g.fillPolygon(x0, y0,5);
	    System.out.println("orientation:"+radian+" sinus:"+Math.sin(radian)+" cosinus:"+Math.cos(radian)+ " alpha:"+Math.toDegrees(alpha)+ " beta:"+Math.toDegrees(beta)+" gamma:"+Math.toDegrees(gamma));
	    for (int i=0;i<=4;i++){
//    	System.out.println((x0[i]-center[0])+","+(y0[i]-center[1]));
    }
//	    int x[]={(int) (center[0]+posX-deltaX),(int) (center[0]+posX),(int) (center[0]+posX+deltaPx),(int) (center[0]+posX),(int) (center[0]+posX-deltaX)};
//	    int y[]={(int) (center[1]+posY+deltaY),(int) (center[1]+posY+deltaY),(int) (center[1]+posY+deltaPy),(int) (center[1]+posY-deltaY),(int) (center[1]+posY-deltaY)};
//	    for (int i=0;i<=4;i++){
//	    	System.out.println((x[i]-center[0])+","+(y[i]-center[1]));
//	    }
//	    g.fillPolygon(x, y,5);
	   // g.fillRect(500+posX, 500+posY, 30, 20);
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
	      }               
	    
	
	  public static void point(int posXi,int posYi, int orienti){
		    //Vous verrez cette phrase chaque fois que la méthode sera invoquée
//		    System.out.println("point Je suis appele x:"+posXi+" Y;"+posYi);
		    posX=posXi;
		    posY=posYi;
		    orient=orienti;
		//   pointComponent(null);
		  }
	 
	}
