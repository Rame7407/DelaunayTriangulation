package delauney;

import delauney.interf.IPoint;
import delauney.model.Point;
import javafx.application.Application;
import javafx.scene.Scene;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.layout.StackPane;
import javafx.scene.paint.Color;
import javafx.stage.Stage;
import java.util.Random;
import java.util.Scanner;
import javafx.scene.text.Font;
import javafx.scene.text.Text;

public class DelaunayVisualization extends Application {
    private static final int WIDTH = 1000;
    private static final int HEIGHT = 800;
    private Triangulator delaunay;

    // Элемент для вывода информации о времени
    private Text timeText;
    //кол-во точек
    public DelaunayVisualization() throws Exception {
        long startTime = System.currentTimeMillis(); // Засекаем время начала выполнения программы
        int n;
        System.out.println("Введите кол-во точек");
        Scanner sc = new Scanner(System.in);
        n = sc.nextInt();
        Random r = new Random();
        IPoint[] pointss = new IPoint[n];
    /*  pointss[0] = new Point(0.0, 1.0);
        pointss[1] = new Point(1.0, 1.0);
        pointss[2] = new Point(1.0, 2.0);
    */
        for(int i = 0;i < n;i++) {
            pointss[i] = new Point(WIDTH*r.nextDouble(),(HEIGHT)*r.nextDouble());
        }
        delaunay = new Triangulator(pointss);
        long endTime = System.currentTimeMillis(); // Засекаем время завершения выполнения программы
        long executionTime = (endTime - startTime); // Вычисляем время выполнения
        // Создаем текстовый элемент для отображения времени выполнения
        timeText = new Text("Кол-во точек: "+ n + "\n" +"Время выполнения: " + executionTime +"\n" +" миллисекунд");
        timeText.setFont(Font.font(16)); // Устанавливаем шрифт и размер текста
    }

    @Override
    public void start(Stage primaryStage) {
        primaryStage.setTitle("Delaunay Triangulation Visualization");

        StackPane root = new StackPane();
        Canvas canvas = new Canvas(WIDTH, HEIGHT);
        //root.getChildren().add(canvas);
        root.getChildren().addAll(canvas, timeText); // Добавляем текстовый элемент на экран

        GraphicsContext gc = canvas.getGraphicsContext2D();

        // Рисуем треугольник

        //gc.setStroke(Color.color(0.5, 0.5, 0.5));
        gc.setStroke(Color.GRAY);
        gc.setLineWidth(1.0);

        for (int i = 0; i < delaunay.getTriangles().length; i += 3) {
            double x1 = delaunay.getCoords()[2 * delaunay.getTriangles()[i]];
            double y1 = delaunay.getCoords()[2 * delaunay.getTriangles()[i] + 1];
            double x2 = delaunay.getCoords()[2 * delaunay.getTriangles()[i + 1]];
            double y2 = delaunay.getCoords()[2 * delaunay.getTriangles()[i + 1] + 1];
            double x3 = delaunay.getCoords()[2 * delaunay.getTriangles()[i + 2]];
            double y3 = delaunay.getCoords()[2 * delaunay.getTriangles()[i + 2] + 1];

            gc.strokeLine(x1, y1, x2, y2);
            gc.strokeLine(x2, y2, x3, y3);
            gc.strokeLine(x3, y3, x1, y1);
        }

        // Рисуем точки
        gc.setFill(Color.BLUE);
        //для лучшего отображения точек если их меньше 10к или больше 10к
        if(delaunay.getCoords().length <= 20000) {
            for (int i = 0; i < delaunay.getCoords().length; i += 2) {
                double x = delaunay.getCoords()[i];
                double y = delaunay.getCoords()[i + 1];
                gc.fillOval(x - 2, y - 2, 4, 4); // Рисует точку как серый круг
            }
        } else {
            for (int i = 0; i < delaunay.getCoords().length; i += 2) {
                double x = delaunay.getCoords()[i];
                double y = delaunay.getCoords()[i + 1];
                gc.fillOval(x - 0.5, y - 0.5, 1, 1); // Рисует точку как серый круг
            }
        }


        // Позиционируем текстовый элемент с информацией о времени выполнения
        timeText.setTranslateX(600); // X-позиция
        timeText.setTranslateY(350); // Y-позиция

        Scene scene = new Scene(root, WIDTH, HEIGHT);
        primaryStage.setScene(scene);
        primaryStage.show();
    }

    public static void main(String[] args) {
        launch(args);
    }
}

