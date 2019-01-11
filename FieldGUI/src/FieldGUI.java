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
        
        DecimalFormat decFormat = new DecimalFormat("#.0");
        
        Label label = new Label("(x, y) --> ");
        label.setFont(new Font("Arial", 15));

        Image field = new Image(getClass().getResourceAsStream("field2.png"));
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
        root.add(rectangle, 0, 0);
        
        img.setOnMouseClicked((MouseEvent e) -> {
            
            String coord = "(x, y) --> " + "(" + decFormat.format(((e.getX() / 810) * 100)) + "%" + ", " + decFormat.format(((e.getY() / 450) * 100)) + "%" + ")";
            
            // this is sort of a dumb way to move the rectangle, but I can't find any other option //
            if(((e.getX() / 810) * 100) > 7.3 && ((e.getY() / 810) * 100) > 1.1 && ((e.getX() / 810) * 100) < 89 && ((e.getY() / 810) * 100) < 73.8){
                rectangle.setX(e.getX() + 35);
                rectangle.setY(e.getY() + 125);
                
            }
            
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
