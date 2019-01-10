import java.io.IOException;
import java.io.PrintWriter;
import java.net.Socket;

class NetworkClient extends Thread{

    private FIeldGUI gui;

    private Socket connection;
    private PrintWriter output;
    private Scanner input;

    public NetworkClient(FieldGUI gui) {
        this.gui = gui;
    }

    public boolean connect() {
        try {
            connection = new Socket("roborio-192-frc.local", 5800);
            input = new Scanner(connection.getInputStream());
            output = new PrintWriter(connection.getOutputStream());
        } catch (IOException e) {
            e.printStackTrace();
            return false;
        }
        return true;
    }

    public void run() {
        while (true) {
            String data = input.nextLine();
            gui.recieveData(data);
        }
    }

    public void sendData(String data) {
        output.println(data);
    }

}