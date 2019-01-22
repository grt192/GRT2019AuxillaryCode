import java.io.IOException;
import java.io.PrintStream;
import java.net.Socket;
import java.util.Scanner;

class NetworkClient extends Thread{

    private FieldGUI gui;

    private Socket connection;
    private PrintStream output;
    private Scanner input;

    public NetworkClient(FieldGUI gui) {
        this.gui = gui;
    }

    public boolean connect() {
        try {
            connection = new Socket("10.1.92.2", 5800);
            input = new Scanner(connection.getInputStream());
            output = new PrintStream(connection.getOutputStream());
            return true;
        } catch (IOException e) {
            e.printStackTrace();
            return false;
        }
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