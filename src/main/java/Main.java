import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;

import java.io.File;

import static org.opencv.imgcodecs.Imgcodecs.imread;

public class Main {

    public static void main(String[] args) {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

        BoardRecognizer recognizer = new BoardRecognizer();

        /*for(int i = 45; i<98; i++){
            //Mat totalOriginal = imread("D:/School/2MIT/DP/picsForDataset/"+i+".jpg", Imgcodecs.IMREAD_GRAYSCALE); //i<73
            //Mat totalOriginal = imread("D:/School/2MIT/DP/picsForDataset/v2/"+i+".jpg", Imgcodecs.IMREAD_GRAYSCALE); //i<46
            //Mat totalOriginal = imread("D:/School/2MIT/DP/picsForDataset/v3/"+i+".jpg", Imgcodecs.IMREAD_GRAYSCALE); //i<37
            //Mat totalOriginal = imread("D:/School/2MIT/DP/picsForDataset/v4/"+i+".jpg", Imgcodecs.IMREAD_GRAYSCALE); //i<35
            Mat totalOriginal = imread("D:/School/2MIT/DP/picsForDataset/v5/"+i+".jpg", Imgcodecs.IMREAD_GRAYSCALE); //i<98
            //Mat totalOriginal = imread("D:/School/2MIT/DP/picsForDataset/v6/"+i+".jpg", Imgcodecs.IMREAD_GRAYSCALE); //i<33

            String dir = System.getProperty("user.dir");
            System.out.println(dir);
            recognizer.processImage(totalOriginal);
        }*/

        String[] pathnames;

        String fig_type = "black_king";

        File f = new File("D:/School/2MIT/DP/data_next_level_1/"+fig_type);

        pathnames = f.list();

        for (String pathname : pathnames) {
            Mat original = imread("D:/School/2MIT/DP/data_next_level_1/" + fig_type + "/" + pathname);
            ImageReformator ir = new ImageReformator();
            ir.doIt(original);
            System.out.println(pathname);
        }
    }
}
