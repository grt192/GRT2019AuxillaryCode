import java.io.IOException;
import java.io.PrintStream;
import java.net.Socket;
import java.util.Scanner;
import java.io.InputStream;

class NetworkClient extends Thread{

    private FieldGUI gui;

    private Socket connection;
    private PrintStream output;
    private Scanner input;
    private InputStream Sinput;

    public NetworkClient(FieldGUI gui) {
        this.gui = gui;
    }

    public boolean connect() {
        try {
            connection = new Socket("roborio-192-frc.local", 5800);
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

    public String readLine() {
        try {
            if (input == null || Sinput.available() == 0)
                return "";
        } catch (IOException e) {
            e.printStackTrace();
            return "";
        }
        return input.nextLine();
    }

    public void sendData(String data) {
        output.println(data);
    }

}