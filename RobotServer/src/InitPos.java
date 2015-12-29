import java.sql.Connection;
import java.sql.DriverManager;
import java.sql.ResultSet;
import java.sql.SQLException;
import java.sql.Statement;


public class InitPos {
	public void InitPos() {
		// TODO Auto-generated method stub
		System.out.println("init" );

		try {

				
		} catch (Exception e) {
			e.printStackTrace();
		} 
	   
	   finally{};
	
	}

	public void InitRobot(int posX, int posY, int orien, int ids) {
		// TODO Auto-generated method stub
		System.out.println("init" );

		Connection connI = null;
		Statement stmtRI = null;
		Statement stmtII = null;

		try {

			Class.forName("com.mysql.jdbc.Driver").newInstance();
			String connectionUrl = "jdbc:mysql://jserver:3306/robot";
			String connectionUser = "jean";
			String connectionPassword = "manu7890";
			connI = DriverManager.getConnection(connectionUrl, connectionUser, connectionPassword);
//			conn.setAutoCommit(false);
			stmtRI = connI.createStatement();
			stmtII = connI.createStatement();

//		String sql="INSERT INTO IndValue VALUES ("+ID+","+IndValue+",now(),"+IndId+")";
		
			
//		String sql="INSERT INTO IndValue VALUES (2,7,now(),1)";

		
//			int posY=0;
			String sql="INSERT INTO scanResult VALUES ("+ids+",now(),"+posX+","+posY+","+0+","+0+","+0+","+orien+")";
			//System.out.println("ind id "+IndIdS+", pos " + IndPos + ", len: " + IndLen+" value"+IndValue);
			System.out.println(sql);
			stmtII.executeUpdate(sql);
				
		} catch (Exception e) {
			e.printStackTrace();
		} 
		finally {
			try { if (stmtRI != null) stmtRI.close(); } catch (SQLException e) { e.printStackTrace(); }
			try { if (stmtII != null) stmtII.close(); } catch (SQLException e) { e.printStackTrace(); }
			try { if (connI != null) connI.close(); } catch (SQLException e) { e.printStackTrace(); }
		}
		
	}
	}
		
