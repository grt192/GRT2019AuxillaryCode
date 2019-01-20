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
import javafx.scene.transform.Rotate;
import javafx.scene.control.Button;
import javafx.event.ActionEvent;
import javafx.event.EventHandler;

/**
 *
 * @author audrey x
 */
public class FieldGUI extends Application {
    
    private NetworkClient client;
    
    @Override
    public void start(Stage stage) throws Exception {
        client = new NetworkClient(this);
        if (!client.connect()) {
            System.out.println("FAILED TO CONNECT");
            System.exit(1);
        }
        GridPane root = new GridPane();

        Button b = new Button("PAUSE"); 
        boolean running  = true;
        
        //b.setOnAction(buttonHandler);
        
        DecimalFormat decFormat = new DecimalFormat("#.0");
        
        Label label = new Label("(x, y) --> ");
        label.setFont(new Font("Arial", 15));

        Image field = new Image(getClass().getResourceAsStream("field3.png"));
        ImageView img = new ImageView(field);
        
        img.setPickOnBounds(true);
        img.setPreserveRatio(true);

        img.setFitWidth(810);
        img.setFitHeight(450);
        
        root.setBackground(new Background(new BackgroundFill(Color.WHITE, CornerRadii.EMPTY, Insets.EMPTY)));
        
        root.setPadding(new Insets(100, 100, 100, 100));
        
        RowConstraints rcIm = new RowConstraints();
        rcIm.setPercentHeight(80);
        rcIm.setValignment(VPos.CENTER);
        root.getRowConstraints().add(rcIm);
        
        RowConstraints rcLab = new RowConstraints();
        rcLab.setPercentHeight(20);
        rcLab.setValignment(VPos.CENTER);
        root.getRowConstraints().add(rcLab);
        
        ColumnConstraints cc = new ColumnConstraints();
        cc.setHalignment(HPos.CENTER);
        cc.setPercentWidth(100);
        root.getColumnConstraints().add(cc);
        
        root.add(img, 0, 0);
        root.add(label, 0, 1);
        
        Rectangle rectangle = new Rectangle(92, 130, 30, 20);
        rectangle.setManaged(false);
        root.add(rectangle, 0, 2);

        String input = client.readLine();
        while(input != ""){
            String[] message = input.split(" ");
            double x = Double.parseDouble(message[1]);
            double y = Double.parseDouble(message[2]);

            rectangle.setX(x);
            rectangle.setY(y);
        }
        
        img.setOnMouseClicked((MouseEvent e) -> {
            double x = e.getX() / (0.65 * img.getFitWidth());
            double y = e.getY() / img.getFitHeight();
            String coord = "(x, y) --> " + "(" + decFormat.format((x * 100)) + "%" + ", " + decFormat.format((y * 100)) + "%" + ")";

            client.sendData("move " + ((x - 0.28) * 4.2672/0.44) + " " + (y * 4.2672));
            System.out.print(coord + "\n");
            label.setText(coord);
        });
        
        //int numRows = 2;
        //BackgroundSize backgroundSize = new BackgroundSize(100, 20, true, true, true, false);
        //BackgroundImage backgroundImage = new BackgroundImage(field, BackgroundRepeat.NO_REPEAT, BackgroundRepeat.NO_REPEAT, BackgroundPosition.CENTER, backgroundSize);
        //Background background = new Background(backgroundImage);
        //root.setBackground(background);

        //StackPane stackPane = new StackPane();
        //stackPane.setPrefSize(810, 450);
        //stackPane.setAlignment(Pos.BOTTOM_CENTER);
        
        //stackPane.getChildren().add(img);
        
        //stackPane.getChildren().add(label);
        
        //stackPane.setBackground(new Background(new BackgroundFill(Color.WHITE, CornerRadii.EMPTY, Insets.EMPTY)));
        
        Scene scene = new Scene(root, 870, 700);
        
        stage.setScene(scene);
        stage.setTitle("Deep Space 2019 GUI -- audrey x.");
        stage.setResizable(false);
        stage.show();
        
    }

    //Called by NetworkClient when data is recieved
    public void recieveData(String data) {
        System.out.println(data);
    }

    // EventHandler<ActionEvent> buttonHandler = new EventHandler<ActionEvent>() {
    //     @Override
    //     public void handle(ActionEvent event) {
            
    //         if(running){
    //             b.setText("RESUME");
    //             client.sendData("paused");
    //             running = false;
    //         } else {
    //             b.setText("PAUSE");
    //             client.sendData("resumed");
    //             running = true;
    //         }
    //     }
    // };
   

    //EventHandler<MouseEvent> mouseHandler = new EventHandler<MouseEvent>() {
 
        //@Override
        //public void handle(MouseEvent mouseEvent) {
            
            //DecimalFormat decFormat = new DecimalFormat("#.0");
            
            //if ((mouseEvent.getX() - 33.5) > 0 && (mouseEvent.getX() - 33.5) < 788 && (mouseEvent.getY() - 31) > 0 && (mouseEvent.getY() - 31) < 385){
                //System.out.println("(X,Y) : " + "(" + decFormat.format(((mouseEvent.getX() - 33.5) / 1.5 )) + " , " + decFormat.format(((mouseEvent.getY() - 31) / 1.5 )) + ")" +"\n");
            //}
        //}  
    //};
    
    public static void main(String[] args) {
        Application.launch(args);
    }
}
