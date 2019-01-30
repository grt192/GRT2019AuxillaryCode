import java.text.DecimalFormat;

import javafx.application.Application;
import javafx.scene.Scene;
import javafx.stage.Stage;
import javafx.scene.input.MouseEvent;
import javafx.geometry.HPos;
import javafx.scene.control.Label;
import javafx.scene.image.ImageView;
import javafx.scene.image.Image;
import javafx.scene.layout.GridPane;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;
import javafx.scene.layout.BackgroundFill;
import javafx.scene.layout.CornerRadii;
import javafx.geometry.Insets;
import javafx.geometry.VPos;
import javafx.scene.layout.Background;
import javafx.scene.text.Font;
import javafx.scene.layout.RowConstraints;
import javafx.scene.layout.ColumnConstraints;
import javafx.scene.shape.Rectangle;
import javafx.scene.shape.Circle;
import javafx.scene.transform.Rotate;
import javafx.scene.control.Button;
import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.scene.transform.Translate; 

/**
 *
 * @author audrey x
 */

 // so i don't foget: rectangle disappears at certain places on the screen
 // the locations change every time
 // has nothing to do with the actual image
 // has nothing to do with the width and height changing
 // has nothing to do with the background color
 // has nothing to do with the location of the robot (aka the robot does not disappear because it's on the blue tape rectangle)
 // has nothing to do with the type of pane that is being used

public class FieldGUI extends Application {
    
    private NetworkClient client;

    private Circle circle = new Circle(60, Color.RED);
    private double newX, newY;
    
    @Override
    public void start(Stage stage) throws Exception {
        client = new NetworkClient(this);
        if (!client.connect()) {
            System.out.println("FAILED TO CONNECT");
            System.exit(1);
        }
        client.start();

        Pane root = new Pane();

        DecimalFormat decFormat = new DecimalFormat("#.0");
        
        Label label = new Label("(x, y) --> ");
        label.setFont(new Font("Arial", 15));

        Image field = new Image(getClass().getResourceAsStream("fieldclear_1.png"));
        ImageView img = new ImageView(field);
        
        img.setPickOnBounds(true);
        img.setPreserveRatio(true);

        img.setFitWidth(870);
        img.setFitHeight(700);

        circle.setCenterX(newX * 5);
        circle.setCenterY(newY * 5);
        
        //root.setBackground(new Background(new BackgroundFill(Color.WHITE, CornerRadii.EMPTY, Insets.EMPTY)));
        root.getChildren().addAll(img, circle);

        img.setOnMouseClicked((MouseEvent e) -> {
            double x = e.getX() / (0.65 * img.getFitWidth());
            double y = e.getY() / img.getFitHeight();
            String coord = "(x, y) --> " + "(" + decFormat.format((x * 100)) + "%" + ", " + decFormat.format((y * 100)) + "%" + ")";

            client.sendData("move " + (x * 14 * 12) + " " + (y * 14 * 12));
            System.out.print(coord + "\n");
            label.setText(coord);
        });
        
        Scene scene = new Scene(root, 870, 700);
        
        stage.setScene(scene);
        stage.setTitle("Deep Space 2019 GUI -- audrey x.");
        stage.setResizable(false);
        stage.show();
        
    }

    //Called by NetworkClient when data is recieved
    public void recieveData(String data) {
        System.out.println(data);
        String[] message = data.split(" ");

        newX = Double.parseDouble(message[1]);
        newY = Double.parseDouble(message[2]);

        // circle.relocate(newX * 5, newY * 5);
        circle.setCenterX(newX * 5);
        circle.setCenterY(newY * 5);
    }
    
    public static void main(String[] args) {
        Application.launch(args);
    }
}
